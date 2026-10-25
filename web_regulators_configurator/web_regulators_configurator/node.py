#!/usr/bin/env python3

# Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

"""
Веб-настройщик ПИД-регуляторов колёс jetbot_mirea.

АННОТАЦИЯ
ROS2-узел веб-настройщика ПИД-регуляторов jetbot_mirea. Раздаёт веб-страницу
(встроенный HTTP-сервер), передаёт на неё телеметрию ESP32 и отправляет
команды в /low_level/serial/cmd через serial_bridge_cpp: скорость базы,
скорости колёс, коэффициенты ПИД (в ОЗУ и во flash), запрос коэффициентов,
ступенчатый тест. Пока команда движения активна, узел повторяет её («мёртвая
рука»): прошивка останавливает робота через 1 с без команд, а при потере
связи со страницей узел сам отправляет остановку.

ANNOTATION
ROS2 node serving a web UI for tuning the jetbot_mirea wheel PID controllers.
"""

from http import HTTPStatus
import os
import signal
import threading
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from . import protocol
from .protocol import ProtocolError
from .step_test import StepTest
from .telemetry import DeadMan, SIGNALS as SIGNAL_NAMES, Telemetry
from .web_server import WebServer

PACKAGE = 'web_regulators_configurator'
MODES = ('base', 'wheels')


class WebConfiguratorNode(Node):

    def __init__(self):
        super().__init__('web_regulators_configurator')

        p = self.declare_parameter
        self._host = p('host', '0.0.0.0').value
        self._port = p('port', 8080).value
        self.telemetry_rate = float(p('telemetry_rate', 10.0).value)
        self._repeat_period = 1.0 / float(p('command_repeat_rate', 5.0).value)
        heartbeat_timeout = float(p('heartbeat_timeout', 1.0).value)
        self._max_linear = float(p('max_linear', 0.4).value)
        self._max_angular = float(p('max_angular', 4.0).value)
        self._max_wheel = float(p('max_wheel_speed', 0.4).value)
        self._wheel_base = float(p('wheel_base', 0.1775).value)
        self._step_max_duration = float(p('step_test_max_duration', 10.0).value)
        self._step_default_duration = float(p('step_test_default_duration', 3.0).value)
        step_tail = float(p('step_test_tail', 1.0).value)
        history_seconds = float(p('history_seconds', 30.0).value)
        cmd_topic = p('cmd_topic', '/low_level/serial/cmd').value
        feedback_topic = p('feedback_topic', '/low_level/serial/feedback').value
        self._log_odometry = bool(p('log_odometry_frames', False).value)
        static_dir = p('static_dir', '').value or os.path.join(
            get_package_share_directory(PACKAGE), 'static')

        self._t0 = time.monotonic()
        # 4 сигнала * 50 Гц * history_seconds
        self.telemetry = Telemetry(history_size=int(4 * 50 * history_seconds))
        self.deadman = DeadMan(heartbeat_timeout)
        self.step = StepTest(tail_duration=step_tail)

        # Все отправки команд и изменения активной команды - под одним замком,
        # чтобы повтор команды не мог «проскочить» после СТОП.
        self._cmd_lock = threading.Lock()
        self._last_sent = 0.0
        self._cmd_vel_publishers = 0
        self._last_pid_request = None
        self._last_setpoints = None
        self._last_setpoints_t = 0.0

        self._cmd_pub = self.create_publisher(String, cmd_topic, 10)
        # Все данные - из сырых кадров ESP32, которые публикует serial_bridge_cpp
        # (publish_raw_feedback:=true, по умолчанию так). Одна подписка на
        # лёгкий String вместо /odom, скоростей колёс и /PIDcoeffs: в rclpy
        # каждый колбэк и разбор Odometry стоят заметного CPU на Jetson.
        # Подписка ленивая: она есть, только пока открыта страница или робот
        # выполняет команду, иначе узел почти не тратит процессор.
        self._feedback_topic = feedback_topic
        self._feedback_sub = None
        self.create_timer(0.1, self._control_tick)        # 10 Гц: повтор, тест, сторож
        self.create_timer(0.5, self._graph_tick)

        self._server = WebServer(self, self._host, self._port, static_dir)
        self._server.start()
        host, port = self._server.address
        self.get_logger().info(
            f'Веб-настройщик ПИД: http://{host}:{port} (с другого устройства - '
            f'IP робота). Команды -> {cmd_topic}')
        self.telemetry.add_log(self.now(), 'info', 'Веб-настройщик запущен')
        self._record_setpoints()

    # ================================================================ время
    def now(self):
        return time.monotonic() - self._t0

    def log_debug(self, text):
        self.get_logger().debug(text)

    # ================================================================ ROS-колбэки
    def _record(self, name, value, t):
        self.telemetry.add_sample(t, name, value)
        self.step.add_sample(t, name, value)

    def _on_frame(self, msg):
        now = self.now()
        try:
            kind, values = protocol.parse_frame(msg.data)
        except ProtocolError:
            self.telemetry.add_log(now, 'rx', msg.data)
            return
        if kind == 1 and len(values) >= 7:
            # $1;x;y;theta;v;w;v_left;v_right;#
            self.telemetry.mark_frame(now)
            for name, value in zip(('v', 'w', 'vl', 'vr'), values[3:7]):
                self._record(name, value, now)
            if self._log_odometry:
                self.telemetry.add_log(now, 'rx', msg.data)
            return
        if kind == 2 and len(values) >= 6:
            # $2;KpL;KiL;KdL;KpR;KiR;KdR;#
            self.telemetry.set_pid(dict(zip(protocol.PID_KEYS, values[:6])))
        self.telemetry.add_log(now, 'rx', msg.data)

    # ================================================================ отправка
    def _send(self, text, log=True):
        """Отправить строку в порт ESP32. Вызывать под self._cmd_lock."""
        out = String()
        out.data = text
        self._cmd_pub.publish(out)
        self._last_sent = self.now()
        if log:
            self.telemetry.add_log(self._last_sent, 'tx', text)

    def _record_setpoints(self):
        """
        Записать уставки в общую историю телеметрии, если они изменились.

        Уставки хранятся на сервере, а не в браузере: тогда вкладка, открытая
        позже, видит их вместе с уже накопленными измерениями.
        """
        command = self.deadman.command()
        sp = self._setpoints(*command) if command else dict.fromkeys(SIGNAL_NAMES, 0.0)
        now = self.now()
        # Повтор раз в 5 с: запись не вытесняется из кольцевого буфера, и новая
        # вкладка всегда получает уставку вместе с историей измерений.
        if sp != self._last_setpoints or now - self._last_setpoints_t >= 5.0:
            self._last_setpoints = sp
            self._last_setpoints_t = now
            for name in SIGNAL_NAMES:
                self.telemetry.add_sample(now, 'sp_' + name, sp[name])

    def _command_text(self, mode, a, b):
        return (protocol.cmd_base_speed(a, b) if mode == 'base'
                else protocol.cmd_wheel_speeds(a, b))

    def _stop(self, reason):
        with self._cmd_lock:
            was_active = self.deadman.clear()
            aborted = self.step.abort(reason)
            self._send(protocol.cmd_stop())
            self._record_setpoints()
        if was_active or aborted:
            self.telemetry.add_log(self.now(), 'info', f'Остановка: {reason}')

    # ================================================================ таймеры
    def _control_tick(self):
        now = self.now()
        action = self.step.tick(now)
        with self._cmd_lock:
            if action == 'stop':
                self.deadman.clear()
                self._send(protocol.cmd_stop())
            command, timed_out = self.deadman.check(now)
            if timed_out:
                self.step.abort('нет связи со страницей')
                self._send(protocol.cmd_stop())
            # минус полшага таймера: иначе повтор раз в 0.2 с при таймере 0.1 с
            # иногда сдвигается на следующий тик и частота падает до ~4 Гц
            elif command is not None and now - self._last_sent >= self._repeat_period - 0.05:
                self._send(self._command_text(*command), log=False)
            self._record_setpoints()
        if action == 'stop':
            self.telemetry.add_log(now, 'info', 'Ступенчатый тест: ступенька завершена')
        elif action == 'done':
            self.telemetry.add_log(now, 'info', 'Ступенчатый тест: результат готов')
        if timed_out:
            self.telemetry.add_log(now, 'info', 'Остановка: страница не отвечает')
            self.get_logger().warn('Нет heartbeat от страницы - робот остановлен')

    def _update_subscription(self):
        needed = (self._server.clients > 0 or self.deadman.command() is not None or
                  self.step.active())
        if needed and self._feedback_sub is None:
            self._feedback_sub = self.create_subscription(
                String, self._feedback_topic, self._on_frame, 10)
        elif not needed and self._feedback_sub is not None:
            self.destroy_subscription(self._feedback_sub)
            self._feedback_sub = None

    def _graph_tick(self):
        self._update_subscription()
        # Другие источники /cmd_vel (Nav2, teleop) будут перебивать команды
        self._cmd_vel_publishers = self.count_publishers('/cmd_vel')
        # Пока коэффициенты ПИД неизвестны, а ESP32 на связи - запрашиваем их
        # (раз в 2 с), чтобы страница сразу показала текущие значения.
        now = self.now()
        age = self.telemetry.last_frame_age(now)
        if (self.telemetry.current_pid() is None and age is not None and age < 1.0 and
                (self._last_pid_request is None or now - self._last_pid_request >= 2.0)):
            self._last_pid_request = now
            with self._cmd_lock:
                self._send(protocol.cmd_request_pid())

    # ================================================================ уставки
    def _clamp_command(self, mode, a, b):
        if mode == 'base':
            return protocol.clamp(a, self._max_linear), protocol.clamp(b, self._max_angular)
        return protocol.clamp(a, self._max_wheel), protocol.clamp(b, self._max_wheel)

    def _setpoints(self, mode, a, b):
        """Уставки всех 4 сигналов (колёса ограничены как в прошивке)."""
        if mode == 'base':
            vl, vr = protocol.wheel_setpoints(a, b, self._wheel_base)
            return {'v': a, 'w': b,
                    'vl': protocol.clamp(vl, self._max_wheel),
                    'vr': protocol.clamp(vr, self._max_wheel)}
        v, w = protocol.base_setpoints(a, b, self._wheel_base)
        return {'v': v, 'w': w, 'vl': a, 'vr': b}

    # ================================================================ API для веб-сервера
    def config(self):
        return {
            'limits': {'max_linear': self._max_linear, 'max_angular': self._max_angular,
                       'max_wheel_speed': self._max_wheel},
            'wheel_base': self._wheel_base,
            'heartbeat_timeout': self.deadman._timeout,
            'step_test': {'max_duration': self._step_max_duration,
                          'default_duration': self._step_default_duration},
            'pid': self.telemetry.current_pid(),
            'log_odometry_frames': self._log_odometry,
        }

    def telemetry_cursor_start(self):
        return 0   # новая страница сразу получает накопленную историю

    def telemetry_payload(self, cursor):
        cursor, samples, log, pid = self.telemetry.since(cursor)
        now = self.now()
        return cursor, {
            't': now,
            'samples': samples,
            'log': log,
            'pid': pid,
            'status': self._status(now),
        }

    def _status(self, now):
        command = self.deadman.command()
        age = self.telemetry.last_frame_age(now)
        return {
            'esp_rate': round(self.telemetry.frame_rate(now), 1),
            'frame_age': None if age is None else round(age, 2),
            'clients': self._server.clients,
            'cmd_vel_publishers': self._cmd_vel_publishers,
            'command': None if command is None else
            {'mode': command[0], 'a': command[1], 'b': command[2]},
            'setpoints': self._setpoints(*command) if command else
            dict.fromkeys(SIGNAL_NAMES, 0.0),
            'step': {k: v for k, v in self.step.status(now).items()},
        }

    def step_recording(self):
        return {'status': self.step.status(self.now()), 'samples': self.step.recording()}

    def step_csv(self):
        if self.step.status(self.now())['state'] != 'done':
            return None
        return self.step.to_csv()

    def handle_api(self, name, body):
        """Команда со страницы. Возвращает (HTTP-код, JSON-ответ)."""
        now = self.now()
        try:
            if name == 'heartbeat':
                self.deadman.heartbeat(now)
                return HTTPStatus.OK, {'ok': True}

            if name == 'stop':
                self._stop(body.get('reason', 'кнопка СТОП'))
                return HTTPStatus.OK, {'ok': True}

            if name == 'command':
                mode = body.get('mode')
                if mode not in MODES:
                    raise ProtocolError("mode: ожидается 'base' или 'wheels'")
                a = protocol.to_float(body.get('a'), 'a')
                b = protocol.to_float(body.get('b'), 'b')
                a, b = self._clamp_command(mode, a, b)
                with self._cmd_lock:
                    if self.step.active():
                        return HTTPStatus.CONFLICT, {'error': 'идёт ступенчатый тест'}
                    if a == 0.0 and b == 0.0:
                        self.deadman.clear()
                    else:
                        self.deadman.set_command(mode, a, b, now)
                    self._send(self._command_text(mode, a, b))
                    self._record_setpoints()
                return HTTPStatus.OK, {'ok': True, 'mode': mode, 'a': a, 'b': b}

            if name in ('pid/set', 'pid/flash'):
                pid = protocol.validate_pid(body.get('pid'))
                text = (protocol.cmd_set_pid(pid) if name == 'pid/set'
                        else protocol.cmd_write_pid_flash(pid))
                with self._cmd_lock:
                    self._send(text)
                    # ESP32 сам не подтверждает запись - сразу запрашиваем текущие
                    self._send(protocol.cmd_request_pid())
                return HTTPStatus.OK, {'ok': True}

            if name == 'pid/request':
                with self._cmd_lock:
                    self._send(protocol.cmd_request_pid())
                return HTTPStatus.OK, {'ok': True}

            if name == 'step_test/start':
                mode = body.get('mode')
                if mode not in MODES:
                    raise ProtocolError("mode: ожидается 'base' или 'wheels'")
                a = protocol.to_float(body.get('a'), 'a')
                b = protocol.to_float(body.get('b'), 'b')
                duration = protocol.to_float(body.get('duration'), 'duration')
                if not 0.2 <= duration <= self._step_max_duration:
                    raise ProtocolError(
                        f'длительность: от 0.2 до {self._step_max_duration:g} с')
                a, b = self._clamp_command(mode, a, b)
                if a == 0.0 and b == 0.0:
                    raise ProtocolError('уставка ступеньки не может быть нулевой')
                with self._cmd_lock:
                    if self.step.active():
                        return HTTPStatus.CONFLICT, {'error': 'тест уже идёт'}
                    self.step.start(mode, a, b, duration, self._setpoints(mode, a, b), now)
                    self.deadman.set_command(mode, a, b, now)
                    self._send(self._command_text(mode, a, b))
                    self._record_setpoints()
                self.telemetry.add_log(
                    now, 'info', f'Ступенчатый тест: {mode} {a:g}, {b:g} на {duration:g} с')
                return HTTPStatus.OK, {'ok': True, 'mode': mode, 'a': a, 'b': b,
                                       'duration': duration}

            if name == 'step_test/abort':
                self._stop('тест прерван')
                return HTTPStatus.OK, {'ok': True}

        except ProtocolError as e:
            return HTTPStatus.BAD_REQUEST, {'error': str(e)}
        return HTTPStatus.NOT_FOUND, {'error': f'неизвестная команда {name}'}

    # ================================================================ завершение
    def shutdown(self):
        try:
            self._stop('узел остановлен')
        finally:
            self._server.stop()


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # Сигналы обрабатываем сами: обработчик rclpy закрывает контекст раньше,
    # чем мы успеваем отправить роботу команду остановки.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    node = WebConfiguratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown()      # отправляет $1;0;0;# и останавливает сервер
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()

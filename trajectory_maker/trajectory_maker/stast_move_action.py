# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
АННОТАЦИЯ
Управляет автономной навигацией мобильного робота через ROS 2 Action Client с
поддержкой ручного ввода целей и автоматических маршрутов. Обрабатывает
взаимодействие с навигационным стеком Nav2, обеспечивает отмену задач и
мониторинг выполнения. Критичные ограничения: предопределенный набор точек
пути, зависимость от доступности ROS 2 Action Server.

ANNOTATION
Controls mobile robot autonomous navigation through ROS 2 Action Client with
support for manual goal input and automated routes. Handles interaction with
Nav2 navigation stack, provides task cancellation and progress monitoring.
Critical limitations: hardcoded waypoint sequence, dependency on ROS 2 Action
Server availability.
'''

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math
import threading
import time

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        
        # Action Client для NavigateToPose
        self.navigate_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Подписчик для позиции робота (опционально, для информации)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', 
                               self.pose_callback, 10)
        
        # Переменные состояния
        self.current_mode = None
        self.current_goal_handle = None
        self.current_pose = None
        self.sequence_index = 0
        self.shutdown_requested = False
        
        # Предопределенная последовательность точек
        self.waypoint_sequence = [
            (-2.0, 0.0, 1.57),      # x, y, theta
            (0.0, 2.0, 0.0),        # x, y, theta (90 градусов)
            (2.0, 0.0, -1.57),      # x, y, theta (180 градусов)
            (0.0, -2.0, 3.14),      # x, y, theta (-90 градусов)
        ]
        
        self.get_logger().info('Goal publisher node started with Action Interface')
        
        # Запускаем выбор режима
        self.choose_mode()

    def choose_mode(self):
        """Выбор режима работы"""
        print("\n=== Выбор режима работы ===")
        print("1 - Ввод целевой точки с клавиатуры")
        print("2 - Публикация набора точек по очереди")
        
        while True:
            try:
                choice = input("Выберите режим (1 или 2): ").strip()
                if choice == '1':
                    self.current_mode = 'keyboard'
                    self.get_logger().info('Режим: ввод с клавиатуры')
                    self.start_keyboard_mode()
                    break
                elif choice == '2':
                    self.current_mode = 'sequence'
                    self.get_logger().info('Режим: последовательность точек')
                    self.start_sequence_mode()
                    break
                else:
                    print("Неверный выбор. Введите 1 или 2.")
            except Exception as e:
                self.get_logger().error(f'Ошибка: {str(e)}')

    def start_keyboard_mode(self):
        """Запуск режима ввода с клавиатуры"""
        self.input_thread = threading.Thread(target=self.read_keyboard_input)
        self.input_thread.daemon = True
        self.input_thread.start()

    def start_sequence_mode(self):
        """Запуск режима последовательности точек"""
        self.get_logger().info(f'Загружено {len(self.waypoint_sequence)} точек')
        print("\nПоследовательность точек:")
        for i, (x, y, theta) in enumerate(self.waypoint_sequence):
            print(f"{i+1}. x={x}, y={y}, theta={theta:.2f}")
        
        # Запускаем публикацию первой точки
        self.publish_next_waypoint()

    def read_keyboard_input(self):
        """Чтение ввода с клавиатуры"""
        while rclpy.ok() and not self.shutdown_requested:
            try:
                user_input = input("\nВведите цель (x y [theta]) или 'q' для выхода, 'c' для отмены: ").strip()
                
                if user_input.lower() == 'q':
                    self.get_logger().info('Завершение работы...')
                    self.shutdown_requested = True
                    return
                elif user_input.lower() == 'c':
                    self.cancel_navigation()
                    continue
                
                coords = user_input.split()
                if len(coords) < 2:
                    self.get_logger().warn('Нужно как минимум 2 координаты (x y)')
                    continue
                
                x = float(coords[0])
                y = float(coords[1])
                theta = float(coords[2]) if len(coords) > 2 else 0.0

                self.send_navigation_goal(x, y, theta)
                
            except ValueError:
                self.get_logger().error('Неверный ввод. Используйте числа')
            except Exception as e:
                self.get_logger().error(f'Ошибка: {str(e)}')

    def publish_next_waypoint(self):
        """Публикация следующей точки в последовательности"""
        if self.sequence_index < len(self.waypoint_sequence):
            x, y, theta = self.waypoint_sequence[self.sequence_index]
            self.send_navigation_goal(x, y, theta)
            self.get_logger().info(f'Отправлена точка {self.sequence_index + 1}/{len(self.waypoint_sequence)}')
        else:
            self.get_logger().info('Все точки последовательности достигнуты!')
            print("\nВсе точки достигнуты! Завершение работы...")
            self.shutdown_requested = True

    def send_navigation_goal(self, x, y, theta):
        """Отправка цели навигации через Action"""
        # Отменяем текущую цель если есть
        if self.current_goal_handle:
            self.cancel_navigation()
            time.sleep(0.5)  # Даем время на отмену
        
        # Создаем цель
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(x, y, theta)
        
        # Ждем сервер если нужно
        if not self.navigate_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return
        
        # Отправляем цель
        self.get_logger().info(f'Отправка цели: x={x}, y={y}, theta={theta:.2f}')
        
        self._send_goal_future = self.navigate_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def create_pose_stamped(self, x, y, theta):
        """Создание PoseStamped сообщения"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Ориентация из угла theta (в радианах)
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        
        return pose

    def goal_response_callback(self, future):
        """Обработка ответа на цель"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Цель отклонена сервером')
            # В режиме последовательности продолжаем со следующей точкой даже если текущая отклонена
            if self.current_mode == 'sequence':
                self.sequence_index += 1
                time.sleep(2.0)
                self.publish_next_waypoint()
            return
        
        self.get_logger().info('Цель принята сервером')
        self.current_goal_handle = goal_handle
        
        # Запрашиваем результат
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Обработка результата навигации"""
        try:
            result = future.result().result
            self.get_logger().info('Навигация до выбранной точки завершена')
            
            # Сбрасываем handle
            self.current_goal_handle = None
            
            # В режиме последовательности переходим к следующей точке
            if self.current_mode == 'sequence':
                self.sequence_index += 1
                # Пауза перед следующей точкой
                time.sleep(2.0)
                self.publish_next_waypoint()
            elif self.current_mode == 'keyboard':
                self.get_logger().info('Готов к приему новой цели')
                
        except Exception as e:
            self.get_logger().error(f'Ошибка при обработке результата: {str(e)}')
            self.current_goal_handle = None

    def feedback_callback(self, feedback_msg):
        """Обработка фидбека навигации"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Прогресс: расстояние до цели: {feedback.distance_remaining:.2f}m, '
            f'время навигации: {feedback.navigation_time.sec}сек',
            throttle_duration_sec=2.0  # Логируем не чаще чем раз в 2 секунды
        )

    def cancel_navigation(self):
        """Отмена текущей навигации"""
        if self.current_goal_handle:
            self.get_logger().info('Отмена текущей навигации')
            future = self.current_goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        """Обработка завершения отмены"""
        try:
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
                self.get_logger().info('Навигация успешно отменена')
            else:
                self.get_logger().info('Не удалось отменить навигацию')
        except Exception as e:
            self.get_logger().error(f'Ошибка при отмене: {str(e)}')
        finally:
            self.current_goal_handle = None

    def pose_callback(self, msg):
        """Колбэк для получения текущей позиции робота"""
        self.current_pose = msg.pose.pose

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    try:
        # Используем spin_once для проверки флага завершения
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Получен сигнал KeyboardInterrupt')
    finally:
        node.cancel_navigation()
        node.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()
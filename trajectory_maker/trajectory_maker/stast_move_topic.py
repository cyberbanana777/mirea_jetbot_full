# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

'''
АННОТАЦИЯ
Публикует целевые точки навигации через ROS 2 topics с ручным и автоматическим
режимами. Реализует мониторинг достижения целей по позиции и ориентации с
заданными допусками. Критичные ограничения: хардкод последовательности точек,
базовый алгоритм проверки достижения цели без интеграции с навигационным стеком.

ANNOTATION
Publishes navigation goal points via ROS 2 topics with manual and automatic
modes. Implements goal achievement monitoring for position and orientation
with specified tolerances. Critical limitations: hardcoded waypoint sequence,
basic goal achievement checking without navigation stack integration.
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math
import threading
import time

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        
        # Публикаторы
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Подписчики
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', 
                               self.pose_callback, 10)
        
        # Переменные состояния
        self.current_mode = None
        self.waiting_for_goal_reached = False
        self.current_pose = None
        self.current_goal = None
        self.sequence_index = 0
        self.position_tolerance = 0.05  # Допуск в метрах
        self.orientation_tolerance = 0.5  # Допуск в радианах
        
        # Таймер для проверки достижения цели
        self.goal_check_timer = self.create_timer(1.0, self.check_goal_reached_timer)
        
        # Предопределенная последовательность точек
        self.waypoint_sequence = [
            (-2.0, 0.0, 1.57),      # x, y, theta
            (0.0, 2.0, 0),     # x, y, theta (90 градусов)
            (2.0, 0.0, -1.57),     # x, y, theta (180 градусов)
            (0.0, -2.0, 3.14),    # x, y, theta (-90 градусов)
        ]
        
        self.get_logger().info('Goal publisher node started')
        
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
        while rclpy.ok():
            try:
                if self.waiting_for_goal_reached:
                    # Ждем достижения цели
                    time.sleep(0.5)
                    continue
                    
                user_input = input("\nВведите цель (x y [theta]) или 'q' для выхода: ").strip()
                
                if user_input.lower() == 'q':
                    self.get_logger().info('Завершение работы...')
                    rclpy.shutdown()
                    return
                
                coords = user_input.split()
                if len(coords) < 2:
                    self.get_logger().warn('Нужно как минимум 2 координаты (x y)')
                    continue
                
                x = float(coords[0])
                y = float(coords[1])
                theta = float(coords[2]) if len(coords) > 2 else 0.0

                self.publish_goal(x, y, theta)
                self.waiting_for_goal_reached = True
                
            except ValueError:
                self.get_logger().error('Неверный ввод. Используйте числа')
            except Exception as e:
                self.get_logger().error(f'Ошибка: {str(e)}')

    def publish_next_waypoint(self):
        """Публикация следующей точки в последовательности"""
        if self.sequence_index < len(self.waypoint_sequence):
            x, y, theta = self.waypoint_sequence[self.sequence_index]
            self.publish_goal(x, y, theta)
            self.waiting_for_goal_reached = True
            self.get_logger().info(f'Опубликована точка {self.sequence_index + 1}/{len(self.waypoint_sequence)}')
        else:
            self.get_logger().info('Все точки последовательности опубликованы!')
            print("\nВсе точки достигнуты! Завершение работы...")
            rclpy.shutdown()

    def publish_goal(self, x, y, theta):
        """Публикация целевой точки"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Позиция
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        
        # Ориентация из угла theta (в радианах)
        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)
        
        self.publisher.publish(msg)
        self.current_goal = msg.pose  # Сохраняем текущую цель для проверки
        self.get_logger().info(f'Опубликована цель: x={x}, y={y}, theta={theta:.2f}')

    def pose_callback(self, msg):
        """Колбэк для получения текущей позиции робота"""
        self.current_pose = msg.pose.pose

    def check_goal_reached_timer(self):
        """Периодическая проверка достижения цели по таймеру"""
        if not self.waiting_for_goal_reached or not self.current_goal or not self.current_pose:
            return
            
        if self.is_goal_reached():
            self.handle_goal_reached()

    def is_goal_reached(self):
        """Проверка достижения цели по позиции и ориентации"""
        # Проверка позиции
        dx = self.current_goal.position.x - self.current_pose.position.x
        dy = self.current_goal.position.y - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Проверка ориентации (yaw угол)
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        goal_yaw = self.quaternion_to_yaw(self.current_goal.orientation)
        yaw_diff = abs(self.normalize_angle(current_yaw - goal_yaw))
        
        position_reached = distance < self.position_tolerance
        orientation_reached = yaw_diff < self.orientation_tolerance
        
        return position_reached and orientation_reached

    def quaternion_to_yaw(self, quat):
        """Преобразование кватерниона в угол yaw"""
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw

    def normalize_angle(self, angle):
        """Нормализация угла в диапазон [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def handle_goal_reached(self):
        """Обработка достижения цели"""
        self.get_logger().info('Цель достигнута!')
        self.waiting_for_goal_reached = False
        
        if self.current_mode == 'sequence':
            self.sequence_index += 1
            # Даем небольшую паузу перед отправкой следующей точки
            time.sleep(2.0)
            self.publish_next_waypoint()
        elif self.current_mode == 'keyboard':
            self.get_logger().info('Готов к приему новой цели')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
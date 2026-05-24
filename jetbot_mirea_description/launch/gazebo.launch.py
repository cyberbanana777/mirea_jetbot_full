# Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 1. Объявляем аргументы
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Запускать RViz2?',
        choices=['true', 'false']
    )
    
    model_use_arg = DeclareLaunchArgument(
        'model_use',
        default_value='ideal',
        description='Позволяет выбрать между реальной и идеальной моделью jetbot?',
        choices=['ideal', 'real']
    )
    
    preset_arg = DeclareLaunchArgument(
        'preset_id',

        default_value='sensors_2',
        description='конфиг rviz: sensors - все датчики робота, mapping - картографирование, navigation - навигация по карте' \
        'и ' \
        'ID полигона: 1 - Г210, 2 - Г210 с препятствиями, 3 - складской.',
        choices=['sensors_1', 'sensors_2',
                 'mapping_1', 'mapping_2',
                 'navigation_1', 'navigation_2',
                 'simple_labirint_3']
    )
    
    # 2. Возвращаем описание запуска
    return LaunchDescription([
        launch_rviz_arg,
        model_use_arg,
        preset_arg,
        # Используем OpaqueFunction для отложенного выполнения
        OpaqueFunction(function=launch_setup)
    ])


def launch_setup(context, *args, **kwargs):
    """Функция, которая выполняется при запуске и имеет доступ к значениям аргументов"""
    launch_actions = []
    
    # 3. Получаем ЗНАЧЕНИЕ аргумента (как строку)
    preset_id = context.launch_configurations.get('preset_id', 'sensors_2')
    launch_rviz = context.launch_configurations.get('launch_rviz', 'true')
    model_use = context.launch_configurations.get('model_use', 'ideal')

    
    # 4. Маппинг миров и их конфигов
    world_configs = {
        'sensors_1': {
            'world_file': 'G210.world',
            'rviz_config': 'sensors.rviz',
            'height_spawn': '1.5',
            'x_spawn': '0.0',
        },
        'sensors_2': {
            'world_file': 'G210_with_boxes.world',
            'rviz_config': 'sensors.rviz',
            'height_spawn': '1.5',
            'x_spawn': '-1.0',
        },
        
        'mapping_1': {
            'world_file': 'G210.world',
            'rviz_config': 'mapping.rviz',
            'height_spawn': '1.5',
            'x_spawn': '0.0',
        },
        'mapping_2': {
            'world_file': 'G210_with_boxes.world',
            'rviz_config': 'mapping.rviz',
            'height_spawn': '1.5',
            'x_spawn': '-1.0',
        },
        'navigation_1': {
            'world_file': 'G210.world',
            'rviz_config': 'navigation.rviz',
            'height_spawn': '1.5',
            'x_spawn': '0.0',
        },
        'navigation_2': {
            'world_file': 'G210_with_boxes.world',
            'rviz_config': 'navigation.rviz',
            'height_spawn': '1.5',
            'x_spawn': '-1.0',
        },
        'simple_labirint_3': {
            'world_file': 'simple_labirint.world',
            'rviz_config': 'gazebo_set_labirint.rviz',
            'height_spawn': '0.1',
            'x_spawn': '0.0',
        },

    }
    
    # 5. Выбираем конфигурацию на основе world_id
    selected_config = world_configs.get(preset_id, world_configs['sensors_2'])
    
    # 6. Получаем пути к файлам пакета
    pkg_path = get_package_share_directory('jetbot_mirea_description')
    
    # 7. Формируем пути
    world_path = os.path.join(pkg_path, 'worlds', selected_config['world_file'])
    rviz_config_path = os.path.join(pkg_path, 'rviz', selected_config['rviz_config'])
    
    if model_use.lower() == 'real':
        urdf_path = os.path.join(pkg_path, 'urdf', 'jetbot_mirea_simple_gazebo_real.urdf.xacro')
    else:
        urdf_path = os.path.join(pkg_path, 'urdf', 'jetbot_mirea_simple_gazebo_ideal.urdf.xacro')

    
    # 8. Запуск Gazebo через стандартный launch файл
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': world_path,
            'verbose': 'true'
        }.items()
    )
    launch_actions.append(gazebo_launch)
    
    # 9. Публикатор состояния робота
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': True
        }]
    )
    launch_actions.append(robot_state_publisher)
    
    # 10. Спавн модели в Gazebo (с небольшой задержкой)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'jetbot',
            '-topic', 'robot_description',
            '-x', f'{selected_config["x_spawn"]}', '-y', '0', '-z', f'{selected_config["height_spawn"]}',
        ],
        output='screen',
    )
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_entity])
    launch_actions.append(delayed_spawn)
    
    # 11. Запуск RViz2 (только если нужно)
    if launch_rviz.lower() == 'true':
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}]
        )
        launch_actions.append(rviz_node)
    
    return launch_actions



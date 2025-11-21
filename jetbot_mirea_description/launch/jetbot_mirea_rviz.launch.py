import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz_description',
        default_value='False',
        description='Запускает / не запускает RViz2.',
        choices=['True', 'False']
    )

    rviz_condition = PythonExpression([
        '"', LaunchConfiguration('launch_rviz_description'), '" == "True"'
    ])

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'jetbot_mirea_description'
    file_subpath = 'urdf/jetbot_mirea_detalied.urdf.xacro'

    rviz_config_dir = os.path.join(get_package_share_directory('jetbot_mirea_description'), 'rviz', 'urdf.rviz')

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    # xacro_path = os.path.join(get_package_share_directory('jetbot_mirea_description'), 'urdf')
    robot_description_raw = xacro.process_file(
        xacro_file,
        mappings={
            'use_nominal_extrinsics': 'false',
            'add_plug': 'false',
            'use_mesh': 'true'
        }
    ).toxml()

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(rviz_condition),
    )


    # Run the node
    return LaunchDescription([
        launch_rviz_arg,
        node_robot_state_publisher,
        joint_state_publisher_node,
        rviz_node
    ])
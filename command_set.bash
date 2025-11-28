# Start slam_toolbox mapping
ros2 launch slam_toolbox online_sync_launch.py slam_params_file:=/home/banana-killer/mirea_jetbots/jetbot_full_ws/src/completed_scripts_jetbot/config/mapper_params_online_async.yaml
ros2 launch slam_toolbox online_sync_launch.py slam_params_file:=/home/banana-killer/mirea_jetbots/jetbot_full_ws/src/completed_scripts_jetbot/config/mapping.yaml

# Start slma_toolbox localization
ros2 launch slam_toolbox localization_launch.py slam_params_file:=/home/banana-killer/mirea_jetbots/jetbot_full_ws/src/completed_scripts_jetbot/config/localization.yaml 

# Start navigation
ros2 launch nav2_bringup navigation_launch.py params_file:=/home/banana-killer/mirea_jetbots/jetbot_full_ws/src/completed_scripts_jetbot/config/nav_param.yaml

# Save map
ros2 run nav2_map_server map_saver_cli -f /home/banana-killer/mirea_jetbots/jetbot_full_ws/src/completed_scripts_jetbot/maps/map
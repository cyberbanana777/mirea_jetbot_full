^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package completed_scripts_jetbot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2026-10-25)
------------------
* Камера RealSense D435i переведена на пакет ``realsense_d435i_configs``
  (пакет ``realsense_pkg`` удалён).
* ``jetbot_sensors_and_motors.launch.py``: камера запускается через
  ``realsense_d435i_configs/launch/d435i.launch.py`` вместо ``rs_launch.py``
  с переменной окружения ``REALSENSE_CONFIG_FILE``. ``rs_launch.py`` эту
  переменную не читает, поэтому раньше настройки камеры не применялись.
* ``jetbot_sensors_and_motors.launch.py``: новый аргумент ``camera_mode``
  (``rgb_usb3`` / ``rgb_usb2`` / ``rgb_depth`` / ``rgb_depth_pointcloud``,
  по умолчанию ``rgb_depth_pointcloud``). Камера, как и раньше, включается
  аргументом ``enable_front_camera:=True``.
* ``jetbot_sensors_and_motors.launch.py`` и
  ``rviz_and_robot_description.launch.py``: статический TF камеры исправлен
  с ``realsense_link -> camera_link`` на ``d435_link -> camera_link``.
  Кадра ``realsense_link`` в текущем URDF нет, поэтому дерево TF камеры не
  было связано с ``base_link``.
* Удалён неиспользуемый ``config/realsense_config.yaml``.
* ``package.xml``: добавлена зависимость ``realsense_d435i_configs``.

1.1.0 (2026-10-25)
------------------
* ``jetbot_sensors_and_motors.launch.py``: низкоуровневые узлы переведены
  на C++. Вместо ``serial_bridge_package`` запускается ``serial_bridge_cpp``
  (одна нода вместо трёх), вместо ``lidar_filter`` - ``lidar_filter_cpp``.
  Аргументы launch-файла не изменились.
* ``package.xml``: зависимости ``serial_bridge_package`` и ``lidar_filter``
  заменены на ``serial_bridge_cpp`` и ``lidar_filter_cpp``. Python-пакеты
  ``serial_bridge_package`` и ``lidar_filter`` удалены из репозитория.
* ``setup.cfg``: ``script-dir`` / ``install-scripts`` заменены на
  ``script_dir`` / ``install_scripts`` (предупреждение setuptools).

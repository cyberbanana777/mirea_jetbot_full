^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package completed_scripts_jetbot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2026-10-16)
------------------
* ``jetbot_sensors_and_motors.launch.py``: новый аргумент ``use_cpp``
  (по умолчанию ``True``). При ``True`` запускаются C++-узлы
  ``serial_bridge_cpp`` и ``lidar_filter_cpp``, при ``False`` - прежние
  Python-узлы ``serial_bridge_package`` и ``lidar_filter``.
* ``package.xml``: добавлены зависимости ``serial_bridge_cpp`` и
  ``lidar_filter_cpp``.
* ``setup.cfg``: ``script-dir`` / ``install-scripts`` заменены на
  ``script_dir`` / ``install_scripts`` (предупреждение setuptools).

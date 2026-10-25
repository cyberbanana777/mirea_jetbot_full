^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense_d435i_configs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-10-25)
------------------
* Первая версия. Заменяет ``realsense_pkg`` (удалён).
* Четыре пресета для ``realsense2_camera``: ``rgb_usb3``, ``rgb_usb2``,
  ``rgb_depth``, ``rgb_depth_pointcloud``; IMU выключен.
* Launch-файл ``d435i.launch.py``: выбор пресета аргументом ``mode``
  (по умолчанию ``rgb_depth_pointcloud``), а также ``camera_name``,
  ``camera_namespace``, ``serial_no``, ``config_file``.
* В пресетах ключ ``/**``: параметры применяются при любом имени и namespace
  ноды. В старом ``realsense_pkg`` ключ ``/camera/camera`` не совпадал с именем
  ноды, и конфиг не применялся.
* Пресет ``rgb_depth_pointcloud`` настроен под малую задержку и менее шумное
  облако: прореживание 2x, пространственный и временной фильтры,
  ``clip_distance: 3.0``, без ``align_depth``, неорганизованное облако.

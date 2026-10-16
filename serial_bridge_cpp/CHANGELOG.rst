^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package serial_bridge_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-10-16)
------------------
* Первая версия. Один C++-узел ``serial_bridge_node`` заменяет три
  Python-узла ``serial_bridge_package`` (``twist_to_command``,
  ``serial_bridge_node``, ``feedback_processor``).
* Топики и форматы совпадают с Python-версией: ``/cmd_vel`` и
  ``serial/cmd`` -> UART; UART -> ``/odom``, TF ``odom -> base_link``,
  ``/sensors/wheel/{left,right}/speed``, ``/PIDcoeffs``,
  ``serial/feedback``. ``simple_PID_configurator``, Nav2 и SLAM работают
  без изменений.
* Работа с портом через POSIX termios + ``poll()`` без внешних зависимостей;
  поток чтения спит до прихода данных.
* Автоматическое переоткрытие порта при его потере (переподключение USB),
  период задаётся параметром ``reconnect_period``.
* Параметры: ``serial_port``, ``baudrate``, ``frame_id``, ``child_frame_id``,
  ``publish_tf``, ``publish_raw_feedback``, ``reconnect_period``.
* Узел зарегистрирован как компонент (``serial_bridge_cpp::SerialBridgeNode``)
  и может загружаться в component container.
* Юнит-тесты разбора кадров (``test/test_frame_parser.cpp``).
* Совместим с Galactic (Jetson) и Humble и новее.

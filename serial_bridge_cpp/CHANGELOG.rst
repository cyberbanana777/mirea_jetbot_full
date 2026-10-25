^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package serial_bridge_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2026-10-25)
------------------
* В ``/odom`` заполняются ковариации позы и скорости. Новые параметры
  ``pose_covariance_diagonal`` и ``twist_covariance_diagonal`` (по 6 значений).
  Значения по умолчанию - заглушки: ``1e6`` для осей, которые одометрия не
  измеряет, и небольшие условные значения для x, y, рыскания, vx, wz.
  Раньше ковариации были нулевыми (для EKF это «абсолютно точно»).
  Неверный параметр (не 6 значений, отрицательное или не число) останавливает
  запуск с понятной ошибкой.
* Юнит-тесты ковариаций (``test/test_covariance.cpp``).
* Поддерживает одометрию 50 Гц от прошивки ESP32 2.1.0 без изменений
  в протоколе.

1.0.0 (2026-10-25)
------------------
* Первая версия. Один C++-узел ``serial_bridge_node`` заменяет три
  Python-узла пакета ``serial_bridge_package`` (удалён):
  ``twist_to_command``, ``serial_bridge_node``, ``feedback_processor``.
* Топики и форматы совпадают с прежними: ``/cmd_vel`` и
  ``/low_level/serial/cmd`` -> UART; UART -> ``/odom``, TF
  ``odom -> base_link``, ``/sensors/wheel/{left,right}/speed``,
  ``/PIDcoeffs``, ``/low_level/serial/feedback``.
  ``simple_PID_configurator``, Nav2 и SLAM работают без изменений.
* Исправлена 100% загрузка ядра CPU прежнего Python-моста (опрос порта
  в цикле без ожидания). Работа с портом через POSIX termios + ``poll()``
  без внешних зависимостей: поток чтения спит до прихода данных.
  Замер на эмуляторе ESP32: 93% одного ядра (Python) -> 0.4% (C++).
* Разбор кадров ``$...#`` как в прошивке ESP32: новый ``$`` начинает кадр
  заново, текст вне кадров игнорируется, кадр длиннее 256 символов
  отбрасывается.
* Автоматическое переоткрытие порта при его потере (переподключение USB),
  период задаётся параметром ``reconnect_period``.
* Параметры: ``serial_port``, ``baudrate``, ``frame_id``, ``child_frame_id``,
  ``publish_tf``, ``publish_raw_feedback``, ``reconnect_period``.
* Узел зарегистрирован как компонент (``serial_bridge_cpp::SerialBridgeNode``)
  и может загружаться в component container.
* Конфиг RViz ``rviz/show_all_stats.rviz`` перенесён из
  ``serial_bridge_package`` и устанавливается в ``share/serial_bridge_cpp/rviz``.
* Юнит-тесты разбора кадров (``test/test_frame_parser.cpp``).
* Совместим с Galactic, Humble и новее.

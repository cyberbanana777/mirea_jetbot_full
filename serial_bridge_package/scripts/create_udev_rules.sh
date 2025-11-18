#!/bin/bash

echo "remap the device serial port(ttyUSBX) to esp32"
echo "esp32 usb connection as /dev/esp32 , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy esp32.rules to  /etc/udev/rules.d/"
colcon_cd serial_bridge_package
sudo cp scripts/esp32.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload --reload-rules 
sudo service udev restart
echo "finish"
echo "<=== ! Please physical reconnect your device to computer ! ===>"

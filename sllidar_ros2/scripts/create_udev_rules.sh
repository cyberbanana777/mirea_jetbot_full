#!/bin/bash

echo "remap the device serial port(ttyUSBX / ttyACMX) to  rplidar"
echo "rplidar usb connection as /dev/rplidar , check it using the command : ls -l /dev|grep -E 'ttyUSB|ttyACM|rplidar'"
echo "start copy rplidar.rules to  /etc/udev/rules.d/"
sudo cp scripts/rplidar.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload --reload-rules 
sudo service udev restart
echo "finish "
echo "<=== ! Please physical reconnect your device to computer ! ===>"

#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to  esp32"
echo "sudo rm   /etc/udev/rules.d/esp32.rules"
sudo rm   /etc/udev/rules.d/esp32.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"

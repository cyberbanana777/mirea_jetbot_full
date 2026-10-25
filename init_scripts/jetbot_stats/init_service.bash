#!/bin/bash

sudo apt isntall python3-serial

SERVICE_NAME='jetbot_telemetria' 

sudo cp $SERVICE_NAME.service /etc/systemd/system/
sudo cp $SERVICE_NAME.timer /etc/systemd/system/

sudo systemctl daemon-reload

# sudo systemctl start $SERVICE_NAME.service
sudo systemctl start $SERVICE_NAME.timer

# sudo systemctl enable $SERVICE_NAME.service
sudo systemctl enable $SERVICE_NAME.timer
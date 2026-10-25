#!/bin/bash

ABSOLUTE_PATH="$(pwd)"

# Имя службы: второй аргумент или значение по умолчанию
SERVICE_NAME="jetbot_telemetria"

# Имя файла службы (с расширением .service)
SERVICE_FILE="${SERVICE_NAME}.service"

# Генерируем содержимое файла службы
cat > "$SERVICE_FILE" << EOF
[Unit]
Description=Jetbot telemetry (send Wi-Fi info to UART)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=$ABSOLUTE_PATH/jetbot_stats.py
User=root
Group=root
StandardOutput=journal
StandardError=journal
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

echo "Файл службы создан: $SERVICE_FILE"
#!/bin/sh
# Проверка, что вся прошивка компилируется (на заглушках Arduino, без платы).
# Использование: UPID_DIR=/путь/к/uPID ./syntax_check.sh
# Это НЕ замена сборке в Arduino IDE: ESP32-специфика и реальные библиотеки не проверяются.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
FW="$HERE/../../esp32_firmware"
UPID_DIR=${UPID_DIR:-$HOME/Arduino/libraries/uPID}
FLAGS="-std=c++17 -Wall -Wextra -Wno-unused-parameter -fsyntax-only -I$HERE/stubs -I$UPID_DIR -I$FW"
for f in "$FW"/*.cpp; do
  echo "== $(basename "$f")"
  g++ $FLAGS "$f"
done
echo "== esp32_firmware.ino"
g++ $FLAGS -x c++ "$FW/esp32_firmware.ino"
echo "OK: всё компилируется"

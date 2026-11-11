#!/bin/sh
# Проверка синтаксиса прошивки на заглушках Arduino БЕЗ библиотеки uPID.
#
# Файлы, которым нужна uPID, помечаются SKIP: без неё они не проверяются.
# Любая другая ошибка компиляции — FAIL.
#
# Это НЕ замена сборке в Arduino IDE: ESP32-специфика и реальные библиотеки
# не проверяются. Полная проверка — ./syntax_check.sh с установленной uPID.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
FW="$HERE/../../esp32_firmware"
FLAGS="-std=c++17 -Wall -Wextra -Wno-unused-parameter -fsyntax-only -I$HERE/stubs -I$FW"
checked=0
skipped=0
failed=0

check_one() {
  name="$1"
  shift
  out=$(g++ $FLAGS "$@" 2>&1)
  rc=$?
  if [ "$rc" -eq 0 ]; then
    echo "OK   $name"
    checked=$((checked + 1))
    return 0
  fi
  # Пропускаем только ошибку вида "fatal error: uPID.h: No such file or directory".
  if echo "$out" | grep -q 'uPID\.h' &&
     ! echo "$out" | grep 'error:' | grep -v 'uPID\.h' | grep -q .; then
    echo "SKIP $name (нужна библиотека uPID)"
    skipped=$((skipped + 1))
    return 0
  fi
  echo "FAIL $name"
  echo "$out" | sed 's/^/     /'
  failed=$((failed + 1))
}

for f in "$FW"/*.cpp; do
  check_one "$(basename "$f")" "$f"
done
check_one "esp32_firmware.ino" -x c++ "$FW/esp32_firmware.ino"

echo "Проверено: $checked, пропущено (нужна uPID): $skipped, ошибок: $failed"
[ "$failed" -eq 0 ] || exit 1

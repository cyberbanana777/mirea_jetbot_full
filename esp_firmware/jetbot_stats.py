#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import serial
import sys
import os
import glob

# Имя интерфейса, который нас интересует
WLAN_IFACE = "wlan0"

def is_wlan0_up():
    """Проверяет, существует ли интерфейс wlan0 и находится ли он в состоянии UP."""
    try:
        result = subprocess.run(
            ['ip', 'link', 'show', WLAN_IFACE],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            universal_newlines=True,
            check=True,
            timeout=5
        )
        # Ищем строку с состоянием UP
        return "state UP" in result.stdout
    except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired):
        return False

def get_wlan0_ip():
    """Возвращает IPv4-адрес интерфейса wlan0 или None, если адреса нет."""
    try:
        result = subprocess.run(
            ['ip', '-4', 'addr', 'show', WLAN_IFACE],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            universal_newlines=True,
            check=True,
            timeout=5
        )
        for line in result.stdout.splitlines():
            if 'inet ' in line:
                # Пример строки: inet 192.168.1.100/24 brd ...
                ip = line.strip().split()[1].split('/')[0]
                return ip
    except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired):
        pass
    return None

def get_ssid(iface=WLAN_IFACE):
    """Возвращает SSID сети для указанного интерфейса (по умолчанию wlan0)."""
    # Способ 1: iwgetid с явным указанием интерфейса
    try:
        result = subprocess.run(
            ['iwgetid', iface, '-r'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            universal_newlines=True,
            check=True,
            timeout=5
        )
        ssid = result.stdout.strip()
        if ssid:
            return ssid
    except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired):
        pass

    # Способ 2: nmcli (интерфейс не указываем, но он выберет активный)
    try:
        result = subprocess.run(
            ['nmcli', '-t', '-f', 'active,ssid,device', 'dev', 'wifi'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            universal_newlines=True,
            check=True,
            timeout=5
        )
        for line in result.stdout.splitlines():
            if line.startswith('yes:') and f':{iface}' in line:
                # Формат: yes:SSID:интерфейс:...
                parts = line.split(':')
                if len(parts) >= 2:
                    return parts[1].strip()
    except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired):
        pass

    return "Unknown"

def get_password_from_files(ssid):
    """Извлекает пароль Wi-Fi из конфигурационных файлов NetworkManager (требует root)."""
    if ssid == "Unknown" or not ssid:
        return "No Password (SSID unknown)"

    connections_dir = "/etc/NetworkManager/system-connections/"
    if not os.path.exists(connections_dir):
        return "Password not found (no connections dir)"

    files = glob.glob(os.path.join(connections_dir, "*"))

    for file_path in files:
        try:
            # Проверяем, содержит ли файл строку с нужным SSID
            grep_cmd = ['grep', '-q', f'ssid={ssid}', file_path]  # sudo не нужен, если скрипт от root
            result = subprocess.run(grep_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            if result.returncode == 0:
                cat_cmd = ['cat', file_path]
                cat_result = subprocess.run(
                    cat_cmd,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    universal_newlines=True,
                    check=True,
                    timeout=5
                )
                for line in cat_result.stdout.splitlines():
                    if line.strip().startswith('psk='):
                        password = line.split('=', 1)[1].strip().strip('"')
                        if password:
                            return password
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired, OSError):
            continue

    return "Password not found"

def get_password(ssid):
    """Основная функция получения пароля."""
    password = get_password_from_files(ssid)
    if password not in ["Password not found", "No Password (SSID unknown)"]:
        return password

    # Резервный метод через nmcli (если файловый не сработал)
    try:
        cmd = ['nmcli', '--show-secrets', 'connection', 'show', ssid]
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            universal_newlines=True,
            check=True,
            timeout=5
        )
        for line in result.stdout.splitlines():
            if '802-11-wireless-security.psk:' in line or 'psk:' in line:
                password = line.split(':', 1)[1].strip()
                if password and password != '(null)':
                    return password
    except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired):
        pass

    return password

def main():
    # 1. Проверяем, активен ли wlan0
    if not is_wlan0_up():
        print(f"❌ Интерфейс {WLAN_IFACE} не активен или отсутствует. Данные не отправлены.")
        sys.exit(1)

    # 2. Получаем IP именно с wlan0
    ip = get_wlan0_ip()
    if not ip:
        print(f"❌ Не удалось получить IP-адрес для {WLAN_IFACE}. Данные не отправлены.")
        sys.exit(1)

    # 3. Получаем SSID для wlan0
    ssid = get_ssid()
    if ssid == "Unknown":
        print(f"⚠️ Не удалось определить SSID для {WLAN_IFACE}. Продолжаем с 'Unknown'.")

    # 4. Получаем пароль (если возможно)
    password = get_password(ssid)

    # Формируем сообщение
    message = f"$6;{ssid};{password};{ip};#\n"
    print(f"Отправляем: {message.strip()}")

    # Настройки порта
    port = "/dev/ttyTHS1"
    baudrate = 115200
    timeout = 1

    # Подсказка, если пароль не найден
    if "not found" in password.lower():
        print("⚠️  Не удалось получить пароль. Убедитесь, что:")
        print("   - Скрипт запущен с sudo (или от root)")
        print("   - В системе есть сохранённые сети Wi-Fi")
        print("   - SSID определён корректно")

    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        ser.write(message.encode('utf-8'))
        print("✅ Сообщение отправлено успешно")
    except serial.SerialException as e:
        print(f"❌ Ошибка порта {port}: {e}")
        print("   Попробуйте: sudo chmod 777 /dev/ttyTHS1")
        sys.exit(1)
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Порт закрыт")

if __name__ == "__main__":
    main()
#!/bin/bash

# Проверка, что скрипт еще не выполнялся
LOCK_FILE="/tmp/usb_device_analyzer.lock"

if [ -f "$LOCK_FILE" ]; then
    echo "╔══════════════════════════════════════════════════╗"
    echo "║                 ПРЕДУПРЕЖДЕНИЕ                  ║"
    echo "╠══════════════════════════════════════════════════╣"
    echo "║ Скрипт уже был выполнен ранее!                  ║"
    echo "║ Для повторного выполнения удалите файл:         ║"
    echo "║ $LOCK_FILE     ║"
    echo "╚══════════════════════════════════════════════════╝"
    exit 1
fi

# Создаем lock-файл
touch "$LOCK_FILE"

# Функция для красивого вывода
print_header() {
    echo "╔══════════════════════════════════════════════════╗"
    echo "║           АНАЛИЗ USB УСТРОЙСТВ                  ║"
    echo "╠══════════════════════════════════════════════════╣"
}

print_success() {
    echo "║  ✓ $1"
}

print_info() {
    echo "║  ℹ $1"
}

print_warning() {
    echo "║  ⚠ $1"
}

print_footer() {
    echo "╠══════════════════════════════════════════════════╣"
    echo "║               ВЫПОЛНЕНИЕ ЗАВЕРШЕНО              ║"
    echo "╚══════════════════════════════════════════════════╝"
}

print_separator() {
    echo "╠══════════════════════════════════════════════════╣"
}

# Основной код
print_header
print_info "Запуск анализа USB устройств..."
print_separator

# Используем diff для нахождения уникальных строк
found_device=false

diff --new-line-format='' --unchanged-line-format='' with_device.txt without_device.txt | \
grep -o 'ID [0-9a-f:]\+' | cut -d' ' -f2 | while read device_id; do
    if [ -n "$device_id" ] && [ "$found_device" = false ]; then
        vendor_id=$(echo "$device_id" | cut -d':' -f1)
        product_id=$(echo "$device_id" | cut -d':' -f2)
        
        print_success "Обнаружено новое USB устройство!"
        print_info "Vendor ID:  $vendor_id"
        print_info "Product ID: $product_id"
        print_info "Полный ID:  $device_id"
        print_separator
        
        # Создаем правило для udev
        print_info "Добавление правила в /etc/udev/rules.d/99-esp32.rules"
        
        # Создаем правило
        RULE_STRING='KERNEL=="ttyUSB*", ATTRS{idVendor}=="'"$vendor_id"'", ATTRS{idProduct}=="'"$product_id"'", MODE:="0777", SYMLINK+="esp32"'
        
        # Записываем правило с использованием tee и sudo
        echo "$RULE_STRING" | sudo tee -a /etc/udev/rules.d/99-esp32.rules > /dev/null
        
        if [ $? -eq 0 ]; then
            print_success "Правило успешно добавлено!"
        else
            print_warning "Ошибка при добавлении правила!"
        fi
        
        print_separator
        print_info "Перезагрузка службы udev..."
        
        # Перезагружаем udev
        sudo service udev reload --reload-rules
        sudo service udev restart
        
        print_success "Служба udev перезагружена"
        print_separator
        print_warning "ВНИМАНИЕ: Физически переподключите устройство к компьютеру!"
        print_separator
        
        found_device=true
    fi
done

print_footer

# Финальное сообщение
echo ""
echo "Для повторного запуска скрипта выполните:"
echo "sudo rm $LOCK_FILE"
echo ""
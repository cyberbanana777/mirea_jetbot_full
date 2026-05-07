#include "display_manager.h"

DisplayManager::DisplayManager(unsigned long interval, unsigned long blinkPeriod)
    : _interval(interval)
    , _blinkPeriod(blinkPeriod)
    , _lastUpdate(0)
    , _lastBlink(0)
    , _lowBatteryMode(false)
    , _showWarning(false)
{}

bool DisplayManager::begin(const char* version) {
    _oled.init();
    _oled.clear();
    _oled.home();
    _oled.println("ESP32 ready");
    _oled.print(version);
    _oled.update();
    delay(1000);
    return true;
}

void DisplayManager::update(const String& ssid, const String& ip, const String& password,
                            float busVoltage, float current, int batteryPercent) {
    // Определяем, перешли ли мы в режим низкого заряда
    bool low = (batteryPercent < 20);
    if (low != _lowBatteryMode) {
        _lowBatteryMode = low;
        _showWarning = false;          // начинаем с обычной страницы
        _lastBlink = millis();
        _lastUpdate = 0;               // принудительно обновить экран сразу
    }

    if (_lowBatteryMode) {
        // Мигающий режим: переключаем страницу по своему таймеру
        if (millis() - _lastBlink >= _blinkPeriod) {
            _showWarning = !_showWarning;
            _lastBlink = millis();
            _render(ssid, ip, password, busVoltage, current, batteryPercent);
        }
    } else {
        // Обычный режим: обновляем экран по основному интервалу
        if (millis() - _lastUpdate >= _interval) {
            _lastUpdate = millis();
            _showWarning = false;      // всегда показываем обычную страницу
            _render(ssid, ip, password, busVoltage, current, batteryPercent);
        }
    }
}

// Приватная функция, выполняющая фактический вывод на дисплей
void DisplayManager::_render(const String& ssid, const String& ip, const String& password,
                             float busVoltage, float current, int batteryPercent) {
    _oled.clear();
    _oled.home();

    if (_showWarning) {
        // Страница предупреждения о низком заряде
        // Используем две строки: первая – предупреждение, вторая – текущее напряжение
        _oled.setScale(1);              // стандартный шрифт
        _oled.println("LOW BATTERY!");
        _oled.print("Voltage: ");
        _oled.print(busVoltage);
        _oled.print("V ");
        _oled.print(batteryPercent);
        _oled.println("%");
        _oled.println("");
        _oled.print("CHARGE ME, PLS!");

        // Ток и процент можно не выводить, чтобы не загромождать
    } else {
        // Обычная информационная страница (как было раньше)
        _oled.setScale(1);
        _oled.print("SSID: ");
        _oled.println(ssid.substring(0, 15));

        _oled.print("IP: ");
        _oled.println(ip);

        _oled.print("PWD: ");
        String pwdShort = password.substring(0, 14);
        if (password.length() > 14) pwdShort += "...";
        _oled.println(pwdShort);

        _oled.print("Bat: ");
        _oled.print(busVoltage);
        _oled.print("V ");
        _oled.print(batteryPercent);
        _oled.print("% ");
        _oled.print(current);
        _oled.println("A");
    }

    _oled.update();
}
// flash_storage.cpp
#include "flash_storage.h"

FlashStorageManager::FlashStorageManager(const char* ns)
    : _namespace(ns)
{}

bool FlashStorageManager::begin(float kp_L, float ki_L, float kd_L, 
                                float kp_R, float ki_R, float kd_R) {
    // Смотрим, есть ли флаг инициализации
    _prefs.begin(_namespace, true); // режим "только чтение"
    bool initialized = _prefs.isKey("nvsInit");
    _prefs.end();

    if (!initialized) {
        // Первый запуск – записываем заводские настройки
        if (!_prefs.begin(_namespace, false)) // режим "чтение/запись"
            return false;
        _prefs.putFloat("Kp_R", kp_R);
        _prefs.putFloat("Ki_R", ki_R);
        _prefs.putFloat("Kd_R", kd_R);
        _prefs.putFloat("Kp_L", kp_L);
        _prefs.putFloat("Ki_L", ki_L);
        _prefs.putFloat("Kd_L", kd_L);
        _prefs.putBool("nvsInit", true);
        _prefs.end();
    }
    return true;
}

bool FlashStorageManager::isInitialized() {
    _prefs.begin(_namespace, true);
    bool init = _prefs.isKey("nvsInit");
    _prefs.end();
    return init;
}

void FlashStorageManager::loadPID(float &Kp_L, float &Ki_L, float &Kd_L,
                                  float &Kp_R, float &Ki_R, float &Kd_R) {
    _prefs.begin(_namespace, true);
    // Если ключа нет, getFloat вернёт заводское значение (второй аргумент)
    Kp_R = _prefs.getFloat("Kp_R");
    Ki_R = _prefs.getFloat("Ki_R");
    Kd_R = _prefs.getFloat("Kd_R");
    Kp_L = _prefs.getFloat("Kp_L");
    Ki_L = _prefs.getFloat("Ki_L");
    Kd_L = _prefs.getFloat("Kd_L");
    _prefs.end();
}

void FlashStorageManager::savePID(float Kp_L, float Ki_L, float Kd_L,
                                  float Kp_R, float Ki_R, float Kd_R) {
    _prefs.begin(_namespace, false);
    _prefs.putFloat("Kp_R", Kp_R);
    _prefs.putFloat("Ki_R", Ki_R);
    _prefs.putFloat("Kd_R", Kd_R);
    _prefs.putFloat("Kp_L", Kp_L);
    _prefs.putFloat("Ki_L", Ki_L);
    _prefs.putFloat("Kd_L", Kd_L);
    // nvsInit не трогаем – он уже есть после первого begin()
    _prefs.end();
}
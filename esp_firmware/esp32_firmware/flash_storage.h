// flash_storage.h
#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include <Preferences.h>

class FlashStorageManager {
public:
    /**
     * @param ns Имя пространства имён в NVS (до 15 символов).
     *           По умолчанию используется "Memory1", как было в оригинале.
     */
    FlashStorageManager(const char* ns = "Memory1");

    /**
     * Инициализирует NVS: при первом вызове записывает дефолтные PID-коэффициенты
     * (из config.h) и устанавливает флаг nvsInit.
     * @return true, если операция прошла успешно.
     */
    bool begin(float kpL, float kiL, float kdL, 
               float kpR, float kiR, float kdR);

    /**
     * Загружает коэффициенты из NVS в переданные переменные.
     * Если ключ отсутствует, подставляет заводские значения из config.h.
     */
    void loadPID(float &Kp_L, float &Ki_L, float &Kd_L,
                 float &Kp_R, float &Ki_R, float &Kd_R);

    /**
     * Сохраняет текущие коэффициенты в NVS.
     */
    void savePID(float Kp_L, float Ki_L, float Kd_L,
                 float Kp_R, float Ki_R, float Kd_R);

    /**
     * Проверяет, был ли уже выполнен первый запуск (наличие флага nvsInit).
     */
    bool isInitialized();

private:
    Preferences _prefs;
    const char* _namespace;
};

#endif
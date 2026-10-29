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
     * Загружает параметры feedforward (u_ff = sign(v) * (deadzone + gain * |v|)).
     * Если в NVS их ещё нет (прошивка обновлена поверх старой), остаются значения,
     * с которыми переменные были переданы (заводские из config.h).
     */
    void loadFF(float &gainL, float &deadzoneL, float &gainR, float &deadzoneR);

    /**
     * Сохраняет параметры feedforward в NVS.
     */
    void saveFF(float gainL, float deadzoneL, float gainR, float deadzoneR);

    /**
     * Загружает/сохраняет постоянную времени фильтра уставки, с.
     * Если ключа нет, остаётся переданное значение (заводское из config.h).
     */
    void loadRefTau(float &tauS);
    void saveRefTau(float tauS);

    /**
     * Загружает/сохраняет выбранный регулятор (0 - ПИ, 1 - LADRC) и параметры LADRC.
     * Если ключей нет, остаются переданные значения (заводские из config.h).
     */
    void loadCtrlMode(int &mode);
    void saveCtrlMode(int mode);
    void loadAdrc(float &wc, float &wo, float &b0L, float &b0R, float &ffMode);
    void saveAdrc(float wc, float wo, float b0L, float b0R, float ffMode);

    /**
     * Проверяет, был ли уже выполнен первый запуск (наличие флага nvsInit).
     */
    bool isInitialized();

private:
    Preferences _prefs;
    const char* _namespace;
};

#endif
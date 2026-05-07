#ifndef SERIAL_RECEIVER_H
#define SERIAL_RECEIVER_H

#include <Arduino.h>

class SerialReceiver {
public:
    /**
     * @param stream   – ссылка на Serial-порт (Serial, Serial1 и т.д.)
     * @param startChar – символ начала сообщения (по умолчанию '$')
     * @param endChar   – символ конца сообщения (по умолчанию '#')
     */
    SerialReceiver(Stream &stream, char startChar = '$', char endChar = '#');

    /** Вызывайте в loop() для обработки поступающих байтов */
    void update();

    /** Возвращает true, если получено завершённое сообщение (ещё не прочитанное) */
    bool available();

    /** Возвращает сообщение БЕЗ обрамляющих символов и сбрасывает флаг */
    String getMessage();

    /** Принудительный сброс состояния (например, при ошибке) */
    void reset();

private:
    Stream &_stream;
    char _startChar;
    char _endChar;
    String _buffer;
    bool _receiving;
    bool _complete;
};

#endif
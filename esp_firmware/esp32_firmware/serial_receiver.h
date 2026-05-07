#ifndef SERIAL_RECEIVER_H
#define SERIAL_RECEIVER_H

#include <Arduino.h>

class SerialReceiver {
public:
    // Максимальная длина одного сообщения, включая $ и #
    static const size_t BUFFER_SIZE = 128;  // заведомо достаточно для протокола

    SerialReceiver(Stream &stream, char startChar = '$', char endChar = '#');

    void update();

    bool available() const;                // true, когда полное сообщение принято

    // Возвращает указатель на внутренний буфер с сообщением (без $ и #)
    const char* getMessage();

    // Сбрасывает состояние, можно вызвать после обработки или при ошибке
    void reset();

    // Удобный метод для копирования сообщения в пользовательский буфер
    void copyMessage(char* dest, size_t maxLen) const;

private:
    Stream &_stream;
    char _startChar;
    char _endChar;
    char _buffer[BUFFER_SIZE];  // статический приёмный буфер
    size_t _index;              // текущая позиция записи
    bool _receiving;
    bool _complete;
};

#endif
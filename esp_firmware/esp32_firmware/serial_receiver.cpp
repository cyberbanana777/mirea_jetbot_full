#include "serial_receiver.h"

SerialReceiver::SerialReceiver(Stream &stream, char startChar, char endChar)
    : _stream(stream)
    , _startChar(startChar)
    , _endChar(endChar)
    , _index(0)
    , _receiving(false)
    , _complete(false)
{
    memset(_buffer, 0, BUFFER_SIZE);
}

void SerialReceiver::update() {
    while (_stream.available()) {
        char c = _stream.read();

        if (c == _startChar) {
            // Начало нового сообщения: сбрасываем предыдущее, даже если не закончено
            _receiving = true;
            _complete = false;
            _index = 0;
            _buffer[0] = _startChar;  // сохраняем '$'
            _index = 1;
        } else if (_receiving) {
            if (_index < BUFFER_SIZE - 1) {
                _buffer[_index++] = c;
                if (c == _endChar) {
                    _buffer[_index] = '\0';  // завершаем строку
                    _receiving = false;
                    _complete = true;
                }
            } else {
                // Переполнение буфера – сбрасываем приём
                _receiving = false;
                _index = 0;
            }
        }
        // символы вне рамок сообщения игнорируются
    }
}

bool SerialReceiver::available() const {
    return _complete;
}

const char* SerialReceiver::getMessage() {
    _complete = false;   // сообщение будет считаться прочитанным
    // Возвращаем указатель НАЧИНАЯ ПОСЛЕ '$' и ДО '#'
    // Но проще вернуть весь буфер, а парсер адаптируется.
    // Сейчас парсер ждёт строку с $ и #, поэтому вернём всю строку.
    return _buffer;
}

void SerialReceiver::reset() {
    _receiving = false;
    _complete = false;
    _index = 0;
    _buffer[0] = '\0';
}

void SerialReceiver::copyMessage(char* dest, size_t maxLen) const {
    if (_complete) {
        strncpy(dest, _buffer, maxLen);
        dest[maxLen - 1] = '\0';
    } else {
        dest[0] = '\0';
    }
}
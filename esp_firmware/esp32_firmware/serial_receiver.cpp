#include "serial_receiver.h"

SerialReceiver::SerialReceiver(Stream &stream, char startChar, char endChar)
    : _stream(stream)
    , _startChar(startChar)
    , _endChar(endChar)
    , _buffer("")
    , _receiving(false)
    , _complete(false)
{}

void SerialReceiver::update() {
    while (_stream.available()) {
        char c = _stream.read();
        if (c == _startChar) {
            _receiving = true;
            _buffer = _startChar;        // начинаем с символа '$'
        } else if (_receiving) {
            _buffer += c;
            if (c == _endChar) {        // встретили '#'
                _receiving = false;
                _complete = true;
            }
        }
    }
}

bool SerialReceiver::available() {
    return _complete;
}

String SerialReceiver::getMessage() {
    _complete = false;
    return _buffer;
}

void SerialReceiver::reset() {
    _buffer = "";
    _complete = false;
    _receiving = false;
}
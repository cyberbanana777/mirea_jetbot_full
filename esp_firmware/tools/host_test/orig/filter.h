// filter.h
#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

template <uint8_t N>
class MovingAverage {
public:
    MovingAverage() : _index(0), _count(0), _sum(0.0f) {}

    // Добавить новое значение, вернуть текущее среднее
    float add(float value) {
        if (_count < N) {
            _buffer[_index] = value;
            _sum += value;
            _index = (_index + 1) % N;
            _count++;
        } else {
            _sum -= _buffer[_index];
            _buffer[_index] = value;
            _sum += value;
            _index = (_index + 1) % N;
        }
        return getAverage();
    }

    // Получить среднее без добавления
    float getAverage() const {
        if (_count == 0) return 0.0f;
        return _sum / static_cast<float>(_count);
    }

    // Сброс накопленных данных
    void reset() {
        _index = 0;
        _count = 0;
        _sum = 0.0f;
    }

private:
    float _buffer[N];
    uint8_t _index;
    uint8_t _count;
    float _sum;
};

#endif
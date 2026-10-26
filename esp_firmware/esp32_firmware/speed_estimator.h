// Copyright (c) 2026 Alice Zenina, Alexander Grachev and Ilya Monakhov RTU MIREA (Russia)
// SPDX-License-Identifier: MIT
// Details in the LICENSE file in the root of the package.

// Оценка скорости по фронтам квадратурного энкодера методом M/T.
// Без зависимостей от Arduino: логику можно проверить тестом на компьютере
// (см. tools/host_test).
//
// Идея. Прерывание энкодера на каждом фронте увеличивает счётчик позиции pos и
// запоминает время этого фронта tLast (обе величины читаются вместе, атомарно).
// На каждом такте контура estimator получает (pos, tLast, now):
//
//     ω = N · α / (tLast − tAnchor),
//
// где N - сколько фронтов набежало между "якорем" (последний фронт, учтённый на
// прошлом шаге) и последним фронтом сейчас, α - угол одного фронта. Интервал
// берётся между фронтами, а не между границами тактов, поэтому ошибка
// квантования не зависит от периода контура:
//   - на высокой скорости в окне десятки фронтов - усреднение без задержки;
//   - на низкой скорости в окне 0-1 фронт - метод сам превращается в T-метод.
//
// Если новых фронтов нет, скорость не может быть выше α/(now − tLast): оценка
// плавно спадает при торможении, а не залипает на старом значении. Дольше
// timeoutUs без фронтов - скорость 0, а первый фронт после остановки только
// переносит якорь (интервал до него включал бы простой).
//
// minEdges > 1 растягивает окно на несколько тактов, пока не наберётся достаточно
// фронтов: сглаживает неидеальность квадратуры ценой задержки на низких скоростях.

#ifndef SPEED_ESTIMATOR_H
#define SPEED_ESTIMATOR_H

#include <stdint.h>
#include <math.h>

struct MTSpeedParams {
  float revPerEdge;   // оборотов на один фронт (1 / (метки * 4))
  uint32_t timeoutUs; // нет фронтов дольше - скорость 0, мкс
  int32_t minEdges;   // минимум фронтов в окне (>= 1); см. config.h
  float maxSane;      // |ω| выше этого (об/с) считается помехой и отбрасывается
  float filterK;      // 1 = без фильтра; 0 < k < 1: ω_f += k (ω - ω_f) на такте контура
};

class MTSpeedEstimator {
public:
  explicit MTSpeedEstimator(const MTSpeedParams &p) : _p(p) {}

  // Забыть историю (при старте и после паузы): следующий вызов update() только "якорится"
  void reset() {
    _valid = false;
    _stopped = false;
    _raw = 0.0f;
    _filt = 0.0f;
  }

  /**
   * @param pos    - накопленная позиция, фронтов (со знаком; переполнение допустимо)
   * @param tLast  - время последнего фронта, мкс (счётчик может переполняться)
   * @param now    - текущее время, мкс. Брать ПОСЛЕ снимка (pos, tLast)
   * @return скорость, об/с (знак - направление счёта pos)
   */
  float update(int32_t pos, uint32_t tLast, uint32_t now) {
    if (!_valid) {
      _anchorPos = pos;
      _anchorT = tLast;
      _valid = true;
      _stopped = false;
      _raw = 0.0f;
      _filt = 0.0f;
      return 0.0f;
    }

    // Возраст последнего фронта. Знаковый: now и tLast читаются не одновременно.
    int32_t age = (int32_t)(now - tLast);
    if (age < 0) age = 0;

    // Фронтов с момента якоря (знак = направление). Через uint32: переполнение определено.
    const int32_t n = (int32_t)((uint32_t)pos - (uint32_t)_anchorPos);
    const int32_t absN = n < 0 ? -n : n;

    if ((uint32_t)age > _p.timeoutUs) {
      // Давно нет фронтов: стоим. Якорь устарел, его заменит первый же новый фронт.
      _raw = 0.0f;
      _stopped = true;
    } else if (_stopped) {
      if (n != 0) {
        // Первый фронт после остановки: интервал до него включает простой, скорость по нему
        // не считается. Якорь переносим на этот фронт; скорость появится со следующим окном.
        _anchorPos = pos;
        _anchorT = tLast;
        _stopped = false;
      }
    } else if (absN >= _p.minEdges) {
      const uint32_t dt = tLast - _anchorT;
      if (dt > 0) {
        const float w = (float)n * _p.revPerEdge / ((float)dt * 1e-6f);
        if (fabsf(w) <= _p.maxSane) _raw = w;
      }
      _anchorPos = pos;
      _anchorT = tLast;
    }

    // Результат этого такта. _raw хранит последнее измеренное окно и ограничением не портится:
    // иначе один долгий интервал из-за неидеальной квадратуры занижал бы скорость до следующего окна.
    float out = _raw;
    if (!_stopped && age > 0) {
      // Следующий фронт не пришёл за age: скорость не выше одного фронта за age.
      // Запас BOUND_MARGIN: у неидеальной квадратуры интервалы между фронтами неравны,
      // и без запаса оценка проваливалась бы перед каждым "длинным" интервалом.
      const float bound = BOUND_MARGIN * _p.revPerEdge * 1e6f / (float)age;
      if (out > bound) out = bound;
      else if (out < -bound) out = -bound;
    }

    _filt += _p.filterK * (out - _filt);
    return _filt;
  }

  float speed() const { return _filt; }

private:
  static constexpr float BOUND_MARGIN = 1.5f;
  MTSpeedParams _p;
  bool _valid = false;
  bool _stopped = false;  // нет фронтов дольше timeoutUs; якорь устарел
  int32_t _anchorPos = 0;
  uint32_t _anchorT = 0;
  float _raw = 0.0f;
  float _filt = 0.0f;
};

#endif

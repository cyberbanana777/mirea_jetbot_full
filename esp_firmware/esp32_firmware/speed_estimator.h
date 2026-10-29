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
//
// Выравнивание по меткам (alignQuad, updateAligned). У реального энкодера 4 фронта одной метки
// (подъём/спад каналов A и B) стоят неравномерно: опыт noisetest показал сдвиг одного из них
// на ~0.5 интервала. Окно, которое начинается на фронте одного типа и кончается на другом,
// "видит" этот сдвиг: ошибка ~0.5 фронта на окно, то есть иглы 1.5-7 % скорости. Поэтому окно
// всегда берётся из целого числа меток: от фронта одного типа до последнего фронта того же
// типа (тип = номер фронта по модулю 4). Для этого прерывание хранит время последнего фронта
// каждого из 4 типов. Цена - окно кончается до 3 фронтов раньше (1 мс на 2.6 об/с).
//
// Номер фронта (id): переход позиции k -> k+1 и k+1 -> k - это один и тот же физический фронт,
// его id = k+1 (большая из двух позиций). Время, когда позиция стала P: при движении вперёд -
// время фронта id = P, назад - id = P + 1.

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
  bool alignQuad = false;  // окно из целого числа меток (только updateAligned)
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

  /**
   * Оценка с окном из целого числа меток (см. "Выравнивание по меткам" выше).
   * @param pos     - накопленная позиция, фронтов
   * @param phaseUs - время последнего фронта каждого типа: phaseUs[id & 3], мкс
   * @param lastDir - направление последнего фронта (+1/-1)
   * @param now     - текущее время, мкс (после снимка)
   * При alignQuad = false сводится к обычному update() по времени последнего фронта.
   */
  float updateAligned(int32_t pos, const uint32_t phaseUs[4], int8_t lastDir, uint32_t now) {
    const int8_t dir = (lastDir < 0) ? -1 : 1;
    if (!_p.alignQuad) return update(pos, _reachT(phaseUs, pos, dir), now);

    const uint32_t tLast = _reachT(phaseUs, pos, dir);
    if (!_valid) {
      _anchorPos = pos;
      _anchorT = tLast;
      _restartPos = pos;
      _valid = true;
      _stopped = false;
      _raw = 0.0f;
      _filt = 0.0f;
      return 0.0f;
    }

    int32_t age = (int32_t)(now - tLast);
    if (age < 0) age = 0;
    const int32_t n = (int32_t)((uint32_t)pos - (uint32_t)_anchorPos);
    // Окно - целое число меток (n округляется к нулю до кратного 4), не меньше minEdges
    const int32_t nA = n - n % 4;
    const int32_t absA = nA < 0 ? -nA : nA;
    const int32_t need = ((_p.minEdges + 3) / 4) * 4;

    if ((uint32_t)age > _p.timeoutUs) {
      _raw = 0.0f;
      _stopped = true;
    } else if (_stopped) {
      if (n != 0) {
        _anchorPos = pos;
        _anchorT = tLast;
        _restartPos = pos;
        _stopped = false;
      }
    } else if (absA >= need && absA >= 4) {
      const int8_t wdir = nA > 0 ? 1 : -1;
      if (wdir == dir) {   // при дребезге около нуля (направление сменилось внутри метки) окно пропускаем
        const int32_t endPos = (int32_t)((uint32_t)_anchorPos + (uint32_t)nA);
        const uint32_t tEnd = _reachT(phaseUs, endPos, wdir);
        const uint32_t dt = tEnd - _anchorT;
        if (dt > 0 && dt < 0x80000000u) {
          const float w = (float)nA * _p.revPerEdge / ((float)dt * 1e-6f);
          if (fabsf(w) <= _p.maxSane) _raw = w;
        }
        _anchorPos = endPos;
        _anchorT = tEnd;
      }
    }

    float out = _raw;
    if (!_stopped && age > 0) {
      // Следующий фронт ещё не пришёл. Если с перезапуска прошла целая метка, ограничение тоже
      // по целой метке: от фронта того же типа, что следующий, 4 фронта назад, до "сейчас" -
      // без влияния неравномерности квадратуры. Иначе - по одному фронту с запасом, как в update().
      const int32_t sinceRestart = (int32_t)((uint32_t)pos - (uint32_t)_restartPos);
      float bound;
      if (sinceRestart >= 4 || sinceRestart <= -4) {
        const uint32_t tMark = _reachT(phaseUs, (int32_t)((uint32_t)pos - (uint32_t)(3 * dir)), dir);
        int32_t ageMark = (int32_t)(now - tMark);
        if (ageMark < 1) ageMark = 1;
        bound = MARK_MARGIN * 4.0f * _p.revPerEdge * 1e6f / (float)ageMark;
      } else {
        bound = BOUND_MARGIN * _p.revPerEdge * 1e6f / (float)age;
      }
      if (out > bound) out = bound;
      else if (out < -bound) out = -bound;
    }

    _filt += _p.filterK * (out - _filt);
    return _filt;
  }

  float speed() const { return _filt; }

  // Отладка (команда $18): выравнивание окна по меткам на ходу
  void setAlignQuad(bool on) {
    _p.alignQuad = on;
    _valid = false;
  }
  bool alignQuad() const { return _p.alignQuad; }

  // Отладка (команда $18): сменить окно оценки на ходу. Следующий вызов update() начнёт окно заново.
  void setMinEdges(int32_t n) {
    _p.minEdges = (n < 1) ? 1 : n;
    _valid = false;
  }
  int32_t minEdges() const { return _p.minEdges; }

private:
  static constexpr float BOUND_MARGIN = 1.5f;
  static constexpr float MARK_MARGIN = 1.1f;   // для ограничения по целой метке: только дрожание меток времени
  // Время, когда позиция стала p (при движении в направлении dir)
  static uint32_t _reachT(const uint32_t phaseUs[4], int32_t p, int8_t dir) {
    const uint32_t id = (uint32_t)p + (dir < 0 ? 1u : 0u);
    return phaseUs[id & 3u];
  }
  MTSpeedParams _p;
  int32_t _restartPos = 0;   // позиция последнего перезапуска (старт, выход из остановки)
  bool _valid = false;
  bool _stopped = false;  // нет фронтов дольше timeoutUs; якорь устарел
  int32_t _anchorPos = 0;
  uint32_t _anchorT = 0;
  float _raw = 0.0f;
  float _filt = 0.0f;
};

#endif

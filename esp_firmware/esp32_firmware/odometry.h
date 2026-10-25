// Copyright (c) 2026 Alice Zenina, Alexander Grachev and Ilya Monakhov RTU MIREA (Russia)
// SPDX-License-Identifier: MIT
// Details in the LICENSE file in the root of the package.

// Одометрия дифференциального привода по пройденному колёсами пути.
// Без зависимостей от Arduino: логику можно проверить тестом на компьютере.
//
// Позиция интегрируется по приращениям пути колёс (из тиков энкодеров),
// а не по отфильтрованным скоростям. Поэтому на позу не влияет задержка
// скользящего среднего ПИД-регуляторов и неточность периода таймера.

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <math.h>

class DiffDriveOdometry {
public:
  explicit DiffDriveOdometry(double wheelBase) : _wheelBase(wheelBase) {}

  /**
   * @brief Обновить позу по приращениям пути колёс за период.
   * @param dLeft  - путь левого колеса, м (вперёд > 0)
   * @param dRight - путь правого колеса, м (вперёд > 0)
   */
  void update(double dLeft, double dRight) {
    const double ds = (dRight + dLeft) / 2.0;              // путь центра базы
    const double dHeading = (dRight - dLeft) / _wheelBase; // поворот базы
    // Интегрирование по средней ориентации за период (метод средней точки):
    // точнее, чем по ориентации в начале периода, особенно на поворотах.
    const double midHeading = _heading + dHeading / 2.0;
    _x += ds * cos(midHeading);
    _y += ds * sin(midHeading);
    _heading += dHeading;
  }

  void reset() { _x = 0.0; _y = 0.0; _heading = 0.0; }

  double x() const { return _x; }
  double y() const { return _y; }
  double heading() const { return _heading; }

private:
  double _wheelBase;
  double _x = 0.0;
  double _y = 0.0;
  double _heading = 0.0;
};

#endif

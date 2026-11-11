#include "control_loop_service.h"
#include "config.h"
#include "protocol_wrapper.h"

float ControlLoopService::convertTicksToFreq(long ticks, long timeout_micros, char dir)
{
  // 4 импульса на 1 шаг, 270 шагов на оборот (ENC_EDGES_PER_REV из config.h)
  const float turn_per_tick = 1.0 / ENC_EDGES_PER_REV;
  const double micros_to_sec = 1.0 / 1000000.0;
  float timeout_sec = timeout_micros * micros_to_sec;
  float result = turn_per_tick * ticks / timeout_sec;

  // для левого колеса инвертируем частоту
  if (dir == 'l')
  {
    result = result * (-1);
  }
  else if (dir == 'r')
  {
    result = result * 1;
  }

  return result;
}

/**
 * @brief Путь колеса по числу тиков энкодера, м (вперёд > 0).
 * Использует convert_ticks_to_freq с периодом 1 с: частота за 1 с = число
 * оборотов, поэтому константы энкодера и знак колеса задаются в одном месте.
 */
float ControlLoopService::convertTicksToDistance(long ticks, char dir)
{
  const long one_second_micros = 1000000;
  float turns = convertTicksToFreq(ticks, one_second_micros, dir);
  return turns * 2 * Pi * WHEEL_RADIUS;
}

/**
 * @brief Один такт контура скорости (100 Гц): оценка скорости колёс по фронтам
 * энкодеров и обновление регуляторов (feedforward + ПИ или LADRC, см. $208).
 */
void ControlLoopService::controlStep()
{
  // Снимок (позиция, время последнего фронта) для каждого колеса, затем время "сейчас":
  // возраст последнего фронта получается неотрицательным
  // (позиция, время последнего фронта каждого из 4 типов, направление) для каждого колеса,
  // затем время "сейчас": возраст фронтов получается неотрицательным
  int32_t posR, posL;
  uint32_t phR[4], phL[4];
  int8_t dirR, dirL;
  ctx_.encoders.snapshotRightPhases(posR, phR, dirR);
  ctx_.encoders.snapshotLeftPhases(posL, phL, dirL);
  const uint32_t now = micros();

  // об/с -> м/с; знак колеса приводится так, чтобы вперёд было > 0 у обоих
  const float revR = ENC_SIGN_R * ctx_.speedR.updateAligned(posR, phR, dirR, now);
  const float revL = ENC_SIGN_L * ctx_.speedL.updateAligned(posL, phL, dirL, now);
  const float rightLinVel = revR * 2 * Pi * WHEEL_RADIUS;
  const float leftLinVel = revL * 2 * Pi * WHEEL_RADIUS;

  // Регуляторы выдают сигнал на моторы
  ctx_.controller.update(leftLinVel, rightLinVel);

  // Скорости для отправки на Jetson
  ctx_.rightVelocity = ctx_.controller.getFilteredRight();
  ctx_.leftVelocity = ctx_.controller.getFilteredLeft();
  ctx_.linearVelocity = (ctx_.rightVelocity + ctx_.leftVelocity) / 2.0;
  ctx_.angularVelocity = (ctx_.rightVelocity - ctx_.leftVelocity) / WHEEL_BASE; // для дифференциального привода

  if (ctx_.telemetryOn && ++ctx_.telemetryDiv >= TELEMETRY_DECIMATION)
  {
    ctx_.telemetryDiv = 0;
    telemetryPublish(now,
                     ctx_.controller.getRefLeft(), ctx_.leftVelocity, ctx_.controller.getOutputLeft(),
                     ctx_.controller.getRefRight(), ctx_.rightVelocity, ctx_.controller.getOutputRight());
  }
}

void ControlLoopService::odometryStep()
{
  int32_t rightTicks = ctx_.encoders.getRightAndReset();
  int32_t leftTicks = ctx_.encoders.getLeftAndReset();

  ctx_.odometry.update(convertTicksToDistance(leftTicks, 'l'),
                       convertTicksToDistance(rightTicks, 'r'));

  if (!ctx_.telemetryOn)
  {
    odomPublish(ctx_.odometry.x(), ctx_.odometry.y(), ctx_.odometry.heading(),
                ctx_.linearVelocity, ctx_.angularVelocity,
                ctx_.leftVelocity, ctx_.rightVelocity);
  }
}

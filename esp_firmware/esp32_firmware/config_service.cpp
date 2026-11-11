#include "command_dispatcher.h"
#include "command_policy.h"
#include "protocol_wrapper.h"
#include <math.h>

void handleConfigCommand(const ParsedCommand& cmd, ConfigContext& ctx) {
  switch (cmd.type) {
    case Command::SET_PID_TEMP:
      if (CommandPolicy::validPid(cmd.args[0], cmd.args[1], cmd.args[2],
                                  cmd.args[3], cmd.args[4], cmd.args[5]))
      {
        ctx.KP_L = cmd.args[0];
        ctx.KI_L = cmd.args[1];
        ctx.KD_L = cmd.args[2];
        ctx.KP_R = cmd.args[3];
        ctx.KI_R = cmd.args[4];
        ctx.KD_R = cmd.args[5];
        ctx.controller.setPID(ctx.KP_L, ctx.KI_L, ctx.KD_L, ctx.KP_R, ctx.KI_R, ctx.KD_R);
      }
      break;

    case Command::SET_PID_FLASH:
      if (CommandPolicy::validPid(cmd.args[0], cmd.args[1], cmd.args[2],
                                  cmd.args[3], cmd.args[4], cmd.args[5]))
      {
        ctx.KP_L = cmd.args[0];
        ctx.KI_L = cmd.args[1];
        ctx.KD_L = cmd.args[2];
        ctx.KP_R = cmd.args[3];
        ctx.KI_R = cmd.args[4];
        ctx.KD_R = cmd.args[5];
        ctx.controller.setPID(ctx.KP_L, ctx.KI_L, ctx.KD_L, ctx.KP_R, ctx.KI_R, ctx.KD_R);
        ctx.flashStorage.savePID(ctx.KP_L, ctx.KI_L, ctx.KD_L, ctx.KP_R, ctx.KI_R, ctx.KD_R);
      }
      break;

    case Command::REQUEST_PID:
      regulatorsCoefficientsPublish(ctx.KP_L, ctx.KI_L, ctx.KD_L, ctx.KP_R, ctx.KI_R, ctx.KD_R);
      break;

    case Command::SET_FF:
      if (cmd.count >= 4 && isfinite(cmd.args[0]) && isfinite(cmd.args[1]) &&
          isfinite(cmd.args[2]) && isfinite(cmd.args[3]))
      {
        ctx.FF_GAIN_L = constrain(cmd.args[0], 0.0f, 20.0f);
        ctx.FF_DEADZONE_L = constrain(cmd.args[1], 0.0f, 0.9f);
        ctx.FF_GAIN_R = constrain(cmd.args[2], 0.0f, 20.0f);
        ctx.FF_DEADZONE_R = constrain(cmd.args[3], 0.0f, 0.9f);
        ctx.controller.setFeedforward(ctx.FF_GAIN_L, ctx.FF_DEADZONE_L, ctx.FF_GAIN_R, ctx.FF_DEADZONE_R);
        if (cmd.count >= 5 && cmd.args[4] >= 0.5f)
          ctx.flashStorage.saveFF(ctx.FF_GAIN_L, ctx.FF_DEADZONE_L, ctx.FF_GAIN_R, ctx.FF_DEADZONE_R);
      }
      break;

    case Command::REQUEST_FF:
      feedforwardPublish(ctx.FF_GAIN_L, ctx.FF_DEADZONE_L, ctx.FF_GAIN_R, ctx.FF_DEADZONE_R);
      refFilterPublish(ctx.REF_TAU_S);
      adrcPublish(ctx.CTRL_MODE, ctx.ADRC_WC, ctx.ADRC_WO, ctx.ADRC_B0_L, ctx.ADRC_B0_R, (int)ctx.ADRC_FF);
      break;

    case Command::SET_REF_FILTER:
      if (cmd.count >= 1 && isfinite(cmd.args[0]))
      {
        ctx.REF_TAU_S = constrain(cmd.args[0], 0.0f, 2.0f);
        ctx.controller.setRefFilter(ctx.REF_TAU_S);
        if (cmd.count >= 2 && cmd.args[1] >= 0.5f)
          ctx.flashStorage.saveRefTau(ctx.REF_TAU_S);
      }
      break;

    case Command::SET_CTRL_MODE:
      // $208;mode;save#  0 - ПИ, 1 - LADRC. Переключение безударное, можно на ходу
      if (cmd.count >= 1 && isfinite(cmd.args[0]))
      {
        ctx.CTRL_MODE = (cmd.args[0] >= 0.5f) ? 1 : 0;
        ctx.controller.setControlMode(ctx.CTRL_MODE == 1 ? ControlMode::kLadrc : ControlMode::kPi);
        if (cmd.count >= 2 && cmd.args[1] >= 0.5f)
          ctx.flashStorage.saveCtrlMode(ctx.CTRL_MODE);
      }
      break;

    case Command::SET_ADRC:
      // $203;wc;wo;b0_L;b0_R;ff;save#  ff: 0 - без прямой подачи, 1 - мёртвая зона, 2 - полный FF (ff и save необязательны, по умолчанию 0)
      if (cmd.count >= 4 && isfinite(cmd.args[0]) && isfinite(cmd.args[1]) &&
          isfinite(cmd.args[2]) && isfinite(cmd.args[3]))
      {
        ctx.ADRC_WC = constrain(cmd.args[0], 0.5f, 50.0f);
        ctx.ADRC_WO = constrain(cmd.args[1], 1.0f, 100.0f);   // wo*dt <= 1 при 10 мс (рекомендуется <= 0.5)
        ctx.ADRC_B0_L = constrain(cmd.args[2], 0.5f, 200.0f);
        ctx.ADRC_B0_R = constrain(cmd.args[3], 0.5f, 200.0f);
        ctx.ADRC_FF = (cmd.count >= 5 && isfinite(cmd.args[4])) ? constrain(roundf(cmd.args[4]), 0.0f, 2.0f) : 0.0f;
        ctx.controller.setAdrc(ctx.ADRC_WC, ctx.ADRC_WO, ctx.ADRC_B0_L, ctx.ADRC_B0_R, (uint8_t)ctx.ADRC_FF);
        if (cmd.count >= 6 && cmd.args[5] >= 0.5f)
          ctx.flashStorage.saveAdrc(ctx.ADRC_WC, ctx.ADRC_WO, ctx.ADRC_B0_L, ctx.ADRC_B0_R, ctx.ADRC_FF);
      }
      break;

    case Command::REQUEST_ADRC:
      adrcPublish(ctx.CTRL_MODE, ctx.ADRC_WC, ctx.ADRC_WO, ctx.ADRC_B0_L, ctx.ADRC_B0_R, (int)ctx.ADRC_FF);
      break;

    case Command::SET_SPEED_WINDOW:
      // $404;n;align#  окно оценки скорости (отладка шума, tools/noisetest.py), не сохраняется;
      // n - минимум фронтов (1..8), align - 1: окно из целых меток, 0: как раньше. $404;# - запрос
      if (cmd.count >= 1 && isfinite(cmd.args[0]))
      {
        const int32_t n = constrain((int32_t)lroundf(cmd.args[0]), (int32_t)1, (int32_t)8);
        ctx.speedL.setMinEdges(n);
        ctx.speedR.setMinEdges(n);
      }
      if (cmd.count >= 2 && isfinite(cmd.args[1]))
      {
        ctx.speedL.setAlignQuad(cmd.args[1] >= 0.5f);
        ctx.speedR.setAlignQuad(cmd.args[1] >= 0.5f);
      }
      speedWindowPublish(ctx.speedL.minEdges(), ctx.speedL.alignQuad() ? 1 : 0);
      break;

    case Command::REQUEST_PID_TX:
      transactionalPidPublish((uint16_t)cmd.args[0],
          ctx.KP_L, ctx.KI_L, ctx.KD_L, ctx.KP_R, ctx.KI_R, ctx.KD_R);
      break;

    case Command::SET_PID_TX: {
      const uint16_t seq = (uint16_t)cmd.args[0];
      if (!CommandPolicy::validTransactionalPid(cmd)) {
        ++ctx.rejected_commands;
        --ctx.accepted_commands;
        configNackPublish(seq, 210, 1);
      } else {
        ctx.KP_L = cmd.args[1]; ctx.KI_L = cmd.args[2]; ctx.KD_L = cmd.args[3];
        ctx.KP_R = cmd.args[4]; ctx.KI_R = cmd.args[5]; ctx.KD_R = cmd.args[6];
        ctx.controller.setPID(ctx.KP_L, ctx.KI_L, ctx.KD_L,
                              ctx.KP_R, ctx.KI_R, ctx.KD_R);
        configAckPublish(seq, 210);
      }
      break;
    }

    default:
      break;
  }
}

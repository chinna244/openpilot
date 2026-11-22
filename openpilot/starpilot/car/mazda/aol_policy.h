#pragma once

#include "selfdrive/pandad/aol_protocol.h"

inline bool mazda_aol_param(uint16_t param) {
  // 19/27: the alpha long words with the Smart CAN Filter bit (16), as starpilot/car/mazda/aol.py admits
  return (param == 0U) || (param == 2U) || (param == 3U) || (param == 10U) || (param == 11U) || (param == 19U) || (param == 27U);
}

inline constexpr AolSafetyProfile MAZDA_AOL_PROFILE{13U, mazda_aol_param, true};

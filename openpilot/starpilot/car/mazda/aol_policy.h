#pragma once

#include "selfdrive/pandad/aol_protocol.h"

inline bool mazda_aol_param(uint16_t param) {
  return (param == 0U) || (param == 2U) || (param == 3U) || (param == 10U) || (param == 11U);
}

inline constexpr AolSafetyProfile MAZDA_AOL_PROFILE{13U, mazda_aol_param, true};

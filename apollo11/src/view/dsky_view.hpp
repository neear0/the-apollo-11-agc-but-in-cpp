#pragma once

#include "agc/dsky.hpp"
#include "ui/ui.h"

// draws an agc::c_dsky with CelosUI and presses its keys
namespace view
{
    ImVec2 dsky_size();                    // the lights, the display and the keypad under them
    void dsky(agc::c_dsky& dsky, ImVec2 pos);
    void keyboard(agc::c_dsky& dsky);      // the pc keyboard as the keypad: digits, v, n, +, -, c, p, r, k and enter
}

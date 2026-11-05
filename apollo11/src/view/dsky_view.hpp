#pragma once

#include <string_view>

#include "agc/dsky.hpp"

// draws an agc::c_dsky with CelosUI and presses its keys
namespace view
{
    // into the window being filled, under the titlebar, with what's running and for how long under the keypad
    void dsky(agc::c_dsky& dsky, std::string_view status, double seconds);
    void keyboard(agc::c_dsky& dsky); // the pc keyboard as the keypad: digits, v, n, +, -, c, p, r, k and enter
}

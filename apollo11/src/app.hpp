#pragma once

#include <string>

#include "agc/computer.hpp"
#include "lm/lunar_module.hpp"

// what the views draw: the computer running Luminary 099, in the lunar module
namespace app
{
    agc::c_computer& computer();
    lm::c_lunar_module& lunar_module();
    const std::string& status(); // the program running, or why none is
}

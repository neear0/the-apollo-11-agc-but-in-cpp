#pragma once

#include <string>

#include "agc/computer.hpp"

namespace app
{
    // the computer the window shows, running Luminary 099 assembled from the repo's sources at startup (see main.cpp)
    agc::c_computer& computer();
    const std::string& status(); // what's running, or why nothing is
}

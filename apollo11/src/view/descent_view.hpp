#pragma once

#include "lm/lunar_module.hpp"
#include "ui/ui.h"

// the landing's side of the panel: the descent so far against the landing site, what the lunar module is doing over
// the surface (height, rates, thrust, propellant), the contact light, and the switches the crew lands with: engine
// arm and engine stop. and the scenario's own: start the landing or power on again, and how fast time goes
namespace view
{
    ImVec2 descent_size();
    void descent(lm::c_lunar_module& lunar_module, ImVec2 pos, float height);
}

#pragma once

#include "lm/lunar_module.hpp"
#include "ui/ui.h"

// the lunar module's side of the panel: the attitude ball (fdai) the imu's gimbals drive, what the spacecraft is doing,
// and the crew's switches and hand controller
namespace view
{
    ImVec2 spacecraft_size();
    void spacecraft(lm::c_lunar_module& lunar_module, ImVec2 pos, float height);
    void hand_controller(lm::c_lunar_module& lunar_module); // the arrows pitch and roll, q and e yaw, while held
}

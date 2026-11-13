#pragma once

#include <string>
#include <vector>

#include "agc/computer.hpp"
#include "lm/landing.hpp"
#include "lm/lunar_module.hpp"

// what the views draw: the computer running Luminary 099, in the lunar module
namespace app
{
    agc::c_computer& computer();
    lm::c_lunar_module& lunar_module();
    const std::string& status(); // the program running, or why none is

    // how many times faster than real time the computer and the lunar module run
    inline int time_warp = 1;

    // a point of the descent: how far the landing site is ahead, along the surface, and the height over it
    struct profile_point_t
    {
        double downrange = 0.0; // m
        double height = 0.0;    // m
    };

    // apollo 11's landing: the computer and the lunar module started over in the descent orbit, 8 minutes before powered
    // descent (lm::plan_landing), the imu turned on and the uplink made. the crew's part is left to the dsky and the
    // switches
    struct landing_t
    {
        lm::landing_start_t start;
        double uplinked_at = 0.0;           // the computer's time() at the uplink
        std::vector<profile_point_t> track; // every 2 s since
    };

    void start_landing();                   // the next frame starts it
    void power_on();                        // the next frame starts over as at power-on, on the pad load alone
    const landing_t* landing();             // the one being flown, null if none

    double mission_time();                  // s since launch on the landing, since power-on otherwise
    profile_point_t profile_point();        // where the lunar module is now, on the landing
}

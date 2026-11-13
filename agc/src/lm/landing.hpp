#pragma once

#include "lm/vector.hpp"

namespace agc
{
    class c_computer;
}

namespace yul
{
    struct assembly_t;
}

// the start of the landing: the lunar module in its descent orbit, minutes before powered descent, with what the
// ground and the crew had given the computer by then
namespace lm
{
    class c_lunar_module;

    // the moon's orientation as Luminary works it out (MOONMX in PLANETARY_INERTIAL_ORIENTATION.agc): the matrix from
    // the basic reference coordinates to moon fixed ones, seconds after TEPHEM's epoch (the time since launch plus
    // TEPHEM)
    mat3_t moon_matrix(double seconds);

    // a moon fixed vector in the basic reference coordinates (RP-TO-R): libration is 504LM, radians
    vec3_t moon_fixed_to_reference(const vec3_t& moon_fixed, double seconds, const vec3_t& libration);

    struct landing_start_t
    {
        double time = 0.0;      // seconds since launch, when the uplink is made
        vec3_t position;        // basic reference coordinates, moon centred, m
        vec3_t velocity;        // m/s
        mat3_t refsmmat;        // the landing site alignment: reference to stable member
        vec3_t site;            // the landing site at TLAND, reference coordinates, m
        double tland = 0.0;     // s
        double ignition = 0.0;  // s, where the plan puts it: TLAND - GUIDDURN - ZOOMTIME
    };

    // from the pad load in the computer's erasable (TLAND, RLS, TEPHEM, 504LM, RIGNX, RIGNZ, VIGN, ZOOMTIME): an
    // orbit that is where Luminary's ignition algorithm wants it at TLAND - GUIDDURN (RIGN from the landing site,
    // horizontal at VIGN, in the site's plane), the time lead seconds before ignition. the moon a point mass (MUM)
    landing_start_t plan_landing(const agc::c_computer& computer, const yul::assembly_t& assembly, double lead);

    // what the crew's alignment would have done, before the imu is turned on: the vehicle turned to the landing site
    // alignment, so the stable member cages to it
    void align_for_landing(c_lunar_module& lunar_module, const landing_start_t& start);

    // what the ground's uplinks would have done, once the imu is on: the clock set to start.time, the lunar module's
    // state vector, the landing site alignment as REFSMMAT, and the flags that say so. the lunar module put there
    void uplink_landing(agc::c_computer& computer, c_lunar_module& lunar_module, const landing_start_t& start, const yul::assembly_t& assembly);
}

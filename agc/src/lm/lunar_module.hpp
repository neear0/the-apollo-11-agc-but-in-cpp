#pragma once

#include <cstdint>

#include "lm/imu.hpp"
#include "lm/vector.hpp"

namespace agc
{
    class c_computer;
}

// the lunar module around the computer: a rigid body turned by its 16 rcs jets (channels 5 and 6), the imu, and the
// switches and hand controller the crew works (channels 30 and 31). its axes: x up through the engine, y to the
// right, z forward out of the hatch, with yaw about x, pitch about y and roll about z
namespace lm
{
    // the pgns mode control switch
    enum class mode_control : std::uint8_t { off, attitude_hold, automatic };

    // the attitude controller, -1 to 1 each way
    struct hand_controller_t
    {
        double pitch = 0.0;
        double yaw = 0.0;
        double roll = 0.0;
    };

    // the descent engine's trim gimbal, radians of bell motion: channel 12 drives it at 0.2 deg/s, to a stop at 6
    // degrees each way
    struct engine_trim_t
    {
        double pitch = 0.0;
        double roll = 0.0;
    };

    class c_lunar_module
    {
    public:
        c_lunar_module();

        // the computer and the vehicle a millisecond at a time, so a jet fires for as long as the computer meant
        void run(agc::c_computer& computer, double seconds);
        void step(agc::c_computer& computer, double dt);

        // the crew's side
        bool guidance_pgns = true;                       // guidance control on pgns: the computer flies, not the ags
        mode_control mode = lm::mode_control::automatic;
        hand_controller_t hand_controller;

        c_imu& imu();
        const c_imu& imu() const;
        const mat3_t& attitude() const;                  // inertial to the vehicle's axes
        const vec3_t& rates() const;                     // about the vehicle's axes, radians a second
        std::uint16_t jets() const;                      // the jets firing: channel 5's bits, then channel 6's shifted up 8
        const engine_trim_t& engine_trim() const;

    private:
        void set_switches(agc::c_computer& computer);
        void drive_trim(const agc::c_computer& computer, double dt);

        mat3_t m_attitude{};
        vec3_t m_rates{};
        c_imu m_imu;
        std::uint16_t m_jets = 0;
        engine_trim_t m_engine_trim;
    };
}

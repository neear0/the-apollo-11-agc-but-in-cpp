#pragma once

#include <cstdint>

#include "lm/imu.hpp"
#include "lm/vector.hpp"

namespace agc
{
    class c_computer;
}

// the lunar module around the computer: a rigid body in orbit around the moon, turned by its 16 rcs jets (channels 5
// and 6) and pushed by its descent engine (channel 11 on and off, the THRUST counter's pulses to the throttle, channel
// 12 to the trim gimbal), the imu, and the switches and hand controller the crew works (channels 30 and 31). its axes:
// x up through the engine, y to the right, z forward out of the hatch, with yaw about x, pitch about y and roll about z
namespace lm
{
    // the moon as the computer sees it (MUM in Luminary's constants), and its mean radius
    inline constexpr double moon_mu = 4.9027780e12;   // m^3/s^2
    inline constexpr double moon_radius = 1737.4e3;   // m

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

    // the descent engine: the computer's throttle command is the pulses it counted out, 12.53 n each, which the manual
    // throttle at its minimum (10%) bounds from below and the throttle's stop (FMAX, 43.5 kn) from above. thrust follows
    // the command 0.2 s behind, while the engine is armed and commanded on
    struct descent_engine_t
    {
        int throttle_pulses = 0;  // the command, 0 up to where it saturates (FMAXODD, 3841)
        double thrust = 0.0;      // n
        double propellant = 0.0;  // kg
        bool on = false;
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
        bool engine_armed = false;                       // the engine arm switch on descent
        bool auto_throttle = true;                       // the throttle on the computer, not the crew's

        // where it is, moon-centred and inertial, the axes the computer's (its basic reference coordinates)
        void set_orbit(const vec3_t& position, const vec3_t& velocity);
        const vec3_t& position() const;                  // m
        const vec3_t& velocity() const;                  // m/s
        double altitude() const;                         // above the moon's mean radius, m
        double mass() const;                             // kg

        c_imu& imu();
        const c_imu& imu() const;
        const mat3_t& attitude() const;                  // inertial to the vehicle's axes
        const vec3_t& rates() const;                     // about the vehicle's axes, radians a second
        std::uint16_t jets() const;                      // the jets firing: channel 5's bits, then channel 6's shifted up 8
        const engine_trim_t& engine_trim() const;
        const descent_engine_t& descent_engine() const;

    private:
        void set_switches(agc::c_computer& computer);
        void drive_trim(const agc::c_computer& computer, double dt);
        void run_engine(agc::c_computer& computer, double dt);

        mat3_t m_attitude{};
        vec3_t m_rates{};
        vec3_t m_position{};
        vec3_t m_velocity{};
        double m_dry_mass = 0.0;                         // everything but the descent propellant
        c_imu m_imu;
        std::uint16_t m_jets = 0;
        engine_trim_t m_engine_trim;
        descent_engine_t m_descent_engine;
    };
}

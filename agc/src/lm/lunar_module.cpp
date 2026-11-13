#include "lm/lunar_module.hpp"

#include <algorithm>
#include <cmath>


#include "agc/computer.hpp"

// the lunar module just after undocking from the command module, with both stages and their propellant. the moments
// of inertia are the ones the autopilot works out for itself after a fresh start (from LEMMASS, as 1JACC, 1JACCQ and
// 1JACCR, the acceleration one jet gives): with them its jets stop the vehicle where it means them to
static constexpr double mass = 15100.0;                             // kg
static constexpr lm::vec3_t inertia{ 17100.0, 15660.0, 19420.0 };  // kg m^2 about x, y, z
static constexpr double jet_thrust = 445.0;                         // n, 100 pounds
static constexpr double jet_arm = 1.68;                             // m, from each axis to the jet quads

// a jet: which way it pushes the vehicle, and which way (and how many arms' worth) it turns it
struct jet_t
{
    lm::vec3_t force;
    lm::vec3_t torque;
};

// the up and down firing jets on channel 5 sit at the corners, a jet arm off both the pitch and the roll axis, so
// each turns the vehicle about u or v, between the two; the sideways firing jets on channel 6 turn it about x (yaw).
// which jet does what is from the autopilot's own tables (ALLJETS and TYPEPOLY in Q_R-AXIS_RCS_AUTOPILOT.agc, JETSALL
// and TYPEP in P-AXIS_RCS_AUTOPILOT.agc)
static constexpr lm::vec3_t axis_u{ 0.0, 1.0, 1.0 };
static constexpr lm::vec3_t axis_v{ 0.0, -1.0, 1.0 };
static constexpr lm::vec3_t axis_p{ 1.0, 0.0, 0.0 };
static constexpr lm::vec3_t up{ 1.0, 0.0, 0.0 }, right{ 0.0, 1.0, 0.0 }, forward{ 0.0, 0.0, 1.0 };

static constexpr std::array<jet_t, 16> rcs_jets = { {
    // channel 5, bits 1-8: jets 1, 2, 5, 6, 9, 10, 13, 14
    { -up, axis_v }, { up, -axis_v }, { -up, axis_u }, { up, -axis_u },
    { -up, -axis_v }, { up, axis_v }, { -up, -axis_u }, { up, axis_u },
    // channel 6, bits 1-8
    { forward, axis_p }, { -forward, -axis_p }, { -forward, axis_p }, { forward, -axis_p },
    { right, axis_p }, { -right, -axis_p }, { -right, axis_p }, { right, -axis_p },
} };

// channel 30, active low
static constexpr std::uint16_t lgc_has_control = 001000;
// channel 31, active low
static constexpr std::uint16_t attitude_hold_mode = 010000, auto_mode = 020000, out_of_detent = 040000;
// channel 13: the autopilot asks for the hand controller's angles in the counters
static constexpr std::uint16_t rhc_counter_enable = 000200, rhc_read = 000400;
// channel 12: the descent engine's trim gimbal, by bell motion
static constexpr std::uint16_t trim_minus_pitch = 000400, trim_plus_pitch = 001000, trim_minus_roll = 002000, trim_plus_roll = 004000;
static constexpr double trim_rate = 0.2 * lm::degrees, trim_limit = 6.0 * lm::degrees;

lm::c_lunar_module::c_lunar_module() = default;

void lm::c_lunar_module::run(agc::c_computer& computer, double seconds)
{
    static constexpr double slice = 0.001;
    const double end = computer.time() + seconds;
    while (computer.time() < end)
    {
        const double start = computer.time();
        computer.run_until(std::min(end, start + slice));
        if (computer.time() <= start)
            break; // less than a memory cycle left
        step(computer, computer.time() - start);
    }
}

void lm::c_lunar_module::step(agc::c_computer& computer, double dt)
{
    if (dt <= 0.0)
        return;

    set_switches(computer);
    drive_trim(computer, dt);

    // the jets the computer has on, the torque and thrust they give
    m_jets = static_cast<std::uint16_t>((computer.channel(005) & 0377) | ((computer.channel(006) & 0377) << 8));
    lm::vec3_t torque, force;
    for (std::size_t i = 0; i < rcs_jets.size(); ++i)
    {
        if (!(m_jets & (1u << i)))
            continue;
        force += rcs_jets[i].force * jet_thrust;
        torque += rcs_jets[i].torque * (jet_thrust * jet_arm);
    }

    // euler's equations, the rates in the vehicle's axes
    const lm::vec3_t momentum{ inertia.x * m_rates.x, inertia.y * m_rates.y, inertia.z * m_rates.z };
    const lm::vec3_t net = torque - lm::cross(m_rates, momentum);
    m_rates += lm::vec3_t{ net.x / inertia.x, net.y / inertia.y, net.z / inertia.z } * dt;

    const double rate = lm::length(m_rates);
    if (rate > 1e-12)
        m_attitude = lm::orthonormalized(lm::frame_rotation(m_rates / rate, rate * dt) * m_attitude);

    m_imu.step(computer, m_attitude, force / mass * dt, dt);
}

lm::c_imu& lm::c_lunar_module::imu()
{
    return m_imu;
}

const lm::c_imu& lm::c_lunar_module::imu() const
{
    return m_imu;
}

const lm::mat3_t& lm::c_lunar_module::attitude() const
{
    return m_attitude;
}

const lm::vec3_t& lm::c_lunar_module::rates() const
{
    return m_rates;
}

std::uint16_t lm::c_lunar_module::jets() const
{
    return m_jets;
}

const lm::engine_trim_t& lm::c_lunar_module::engine_trim() const
{
    return m_engine_trim;
}

void lm::c_lunar_module::drive_trim(const agc::c_computer& computer, double dt)
{
    const std::uint16_t channel12 = computer.channel(012);
    const auto drive = [&](double& angle, std::uint16_t plus, std::uint16_t minus) {
        const int direction = ((channel12 & plus) ? 1 : 0) - ((channel12 & minus) ? 1 : 0);
        angle = std::clamp(angle + direction * trim_rate * dt, -trim_limit, trim_limit);
    };
    drive(m_engine_trim.pitch, trim_plus_pitch, trim_minus_pitch);
    drive(m_engine_trim.roll, trim_plus_roll, trim_minus_roll);
}

// a counter's value from -1 to 1 of the hand controller's travel, 42 counts at full deflection, in ones' complement
static int hand_controller_counts(double deflection)
{
    const int counts = static_cast<int>(std::lround(std::clamp(deflection, -1.0, 1.0) * 42.0));
    return counts < 0 ? 077777 & ~(-counts) : counts;
}

void lm::c_lunar_module::set_switches(agc::c_computer& computer)
{
    computer.set_input(030, lgc_has_control, guidance_pgns ? 0 : lgc_has_control);

    const bool deflected = std::abs(hand_controller.pitch) > 0.02 || std::abs(hand_controller.yaw) > 0.02 || std::abs(hand_controller.roll) > 0.02;
    std::uint16_t channel31 = attitude_hold_mode | auto_mode | out_of_detent;
    if (mode == lm::mode_control::attitude_hold)
        channel31 &= static_cast<std::uint16_t>(~attitude_hold_mode);
    else if (mode == lm::mode_control::automatic)
        channel31 &= static_cast<std::uint16_t>(~auto_mode);
    if (deflected)
        channel31 &= static_cast<std::uint16_t>(~out_of_detent);
    computer.set_input(031, attitude_hold_mode | auto_mode | out_of_detent, channel31);

    // read into the counters while the autopilot has them enabled
    const std::uint16_t channel13 = computer.channel(013);
    if ((channel13 & rhc_counter_enable) && (channel13 & rhc_read))
    {
        computer.set_counter(agc::counter_q_rhc, hand_controller_counts(hand_controller.pitch));
        computer.set_counter(agc::counter_p_rhc, hand_controller_counts(hand_controller.yaw));
        computer.set_counter(agc::counter_r_rhc, hand_controller_counts(hand_controller.roll));
    }
}

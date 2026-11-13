#include "lm/imu.hpp"

#include <algorithm>
#include <cmath>

#include "agc/computer.hpp"

static constexpr int cdu_counts = 32768;                  // a revolution, the cdus count in 2^-15 revolutions
static constexpr double gyro_pulse = lm::two_pi / 2097152.0; // 2^-21 revolutions, about 0.62 arc seconds
static constexpr double coarse_pulse = lm::two_pi / 8192.0;  // 4 cdu counts, 2^-13 revolutions
static constexpr double coarse_rate = 60.0 * lm::degrees;    // how fast the gimbal servos drive in coarse align, per second
static constexpr double cdu_rate = 6400.0;                   // counts a second, the cdus' fast rate

// channel 12: what the computer tells the imu
static constexpr std::uint16_t coarse_align_enable = 000010, zero_cdus = 000020, error_counters_enable = 000040, turn_on_delay_complete = 040000;
// channel 30: what the imu tells the computer, active low
static constexpr std::uint16_t imu_operate = 000400, imu_cage = 002000, turn_on_requested = 020000;

lm::mat3_t lm::gimbal_matrix(const lm::gimbal_angles_t& angles)
{
    return lm::frame_rotation_x(angles.outer) * lm::frame_rotation_z(angles.middle) * lm::frame_rotation_y(angles.inner);
}

lm::gimbal_angles_t lm::gimbal_angles(const lm::mat3_t& m)
{
    // m = x(o) z(m) y(i), row 0 is [cos m cos i, sin m, -cos m sin i], column 1 is [sin m, cos o cos m, -sin o cos m]
    lm::gimbal_angles_t angles;
    angles.middle = std::asin(std::clamp(m.at(0, 1), -1.0, 1.0));
    angles.inner = std::atan2(-m.at(0, 2), m.at(0, 0));
    angles.outer = std::atan2(-m.at(2, 1), m.at(1, 1));
    return angles;
}

// a to b the short way round
static double angle_difference(double a, double b)
{
    return std::remainder(b - a, lm::two_pi);
}

static double drive_toward(double angle, double target, double step)
{
    const double difference = angle_difference(angle, target);
    return angle + std::clamp(difference, -step, step);
}

void lm::c_imu::step(agc::c_computer& computer, const lm::mat3_t& body, const lm::vec3_t& delta_v, double dt)
{
    const std::uint16_t channel12 = computer.channel(012);

    // switched on, the imu asks for its turn-on sequence until the computer says the delay is over
    if (operate && !m_was_operating)
        m_turn_on_request = true;
    if (!operate || (channel12 & turn_on_delay_complete))
        m_turn_on_request = false;
    m_was_operating = operate;

    const std::uint16_t status = static_cast<std::uint16_t>((operate ? 0 : imu_operate) | (cage ? 0 : imu_cage) | (m_turn_on_request ? 0 : turn_on_requested));
    computer.set_input(030, imu_operate | imu_cage | turn_on_requested, status);

    const std::array<int, 3> gyro = computer.take_gyro_pulses();
    const std::array<int, 3> drive = computer.take_cdu_drive_pulses();

    const bool coarse = operate && (cage || (channel12 & coarse_align_enable));
    if (!operate)
    {
        // nothing holds the stable member: the gimbals stay where they are and it turns with the vehicle
        m_stable_member = lm::gimbal_matrix(m_gimbals).transposed() * body;
    }
    else if (coarse)
    {
        // the gimbal servos drive to the angles the cdus are told, zero while they're zeroed or the imu caged
        if (!m_coarse)
            m_coarse_target = m_gimbals;
        if (cage || (channel12 & zero_cdus))
            m_coarse_target = {};
        else if (channel12 & error_counters_enable)
        {
            m_coarse_target.outer += drive[0] * coarse_pulse;
            m_coarse_target.inner += drive[1] * coarse_pulse;
            m_coarse_target.middle += drive[2] * coarse_pulse;
        }

        const double step = coarse_rate * dt;
        m_gimbals.outer = drive_toward(m_gimbals.outer, m_coarse_target.outer, step);
        m_gimbals.inner = drive_toward(m_gimbals.inner, m_coarse_target.inner, step);
        m_gimbals.middle = std::clamp(drive_toward(m_gimbals.middle, m_coarse_target.middle, step), -89.9 * lm::degrees, 89.9 * lm::degrees);
        m_stable_member = lm::gimbal_matrix(m_gimbals).transposed() * body;
    }
    else
    {
        // inertial: the gyros hold the stable member still in space, but for the torquing, which turns it about its own axes
        for (int axis = 0; axis < 3; ++axis)
        {
            if (gyro[static_cast<std::size_t>(axis)] == 0)
                continue;
            const lm::vec3_t axis_vector{ axis == 0 ? 1.0 : 0.0, axis == 1 ? 1.0 : 0.0, axis == 2 ? 1.0 : 0.0 };
            m_stable_member = lm::frame_rotation(axis_vector, gyro[static_cast<std::size_t>(axis)] * gyro_pulse) * m_stable_member;
        }
        m_stable_member = lm::orthonormalized(m_stable_member);
        m_gimbals = lm::gimbal_angles(body * m_stable_member.transposed());
    }
    m_coarse = coarse;

    if (operate)
    {
        count_cdus(computer, (channel12 & zero_cdus) != 0, dt);
        count_pipas(computer, body, delta_v);
    }
}

const lm::mat3_t& lm::c_imu::stable_member() const
{
    return m_stable_member;
}

const lm::gimbal_angles_t& lm::c_imu::gimbals() const
{
    return m_gimbals;
}

bool lm::c_imu::coarse_align() const
{
    return m_coarse;
}

bool lm::c_imu::turn_on_request() const
{
    return m_turn_on_request;
}

// the cdus follow the gimbals and count the change into the computer, unless they're being zeroed. they only count
// differences: the computer zeroes its counters when it zeroes the cdus, which is how the two start out agreeing
void lm::c_imu::count_cdus(agc::c_computer& computer, bool zero, double dt)
{
    const std::array<double, 3> angles = { m_gimbals.outer, m_gimbals.inner, m_gimbals.middle };
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        if (zero)
        {
            m_cdu_counts[axis] = 0;
            m_cdu_credit[axis] = 0.0;
            continue;
        }

        const int target = static_cast<int>(std::lround(angles[axis] / lm::two_pi * cdu_counts)) & (cdu_counts - 1);
        int difference = (target - m_cdu_counts[axis]) & (cdu_counts - 1);
        if (difference >= cdu_counts / 2)
            difference -= cdu_counts;

        m_cdu_credit[axis] = std::min(m_cdu_credit[axis] + cdu_rate * dt, cdu_rate * 0.01);
        const int pulses = std::min(std::abs(difference), static_cast<int>(m_cdu_credit[axis]));
        m_cdu_credit[axis] -= pulses;

        const agc::counter_pulse pulse = difference > 0 ? agc::counter_pulse::pcdu : agc::counter_pulse::mcdu;
        for (int i = 0; i < pulses; ++i)
            computer.count(agc::counter_cdux + static_cast<int>(axis), pulse);
        m_cdu_counts[axis] = (m_cdu_counts[axis] + (difference > 0 ? pulses : -pulses)) & (cdu_counts - 1);
    }
}

// the accelerometers on the stable member count each cm/s of velocity the vehicle's thrust gives it along their axes
void lm::c_imu::count_pipas(agc::c_computer& computer, const lm::mat3_t& body, const lm::vec3_t& delta_v)
{
    m_pipa_residual += m_stable_member * (body.transposed() * delta_v) * 100.0;

    const std::array<double*, 3> residuals = { &m_pipa_residual.x, &m_pipa_residual.y, &m_pipa_residual.z };
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        double& residual = *residuals[axis];
        const int counter = agc::counter_pipax + static_cast<int>(axis);
        for (; residual >= 1.0; residual -= 1.0)
            computer.count(counter, agc::counter_pulse::pinc);
        for (; residual <= -1.0; residual += 1.0)
            computer.count(counter, agc::counter_pulse::minc);
    }
}

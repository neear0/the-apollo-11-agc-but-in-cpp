#include "lm/lunar_module.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>


#include "agc/computer.hpp"

// Apollo 11's lunar module at descent orbit insertion, the mass the pad load gives (MASS, 33664 lb), with the descent
// stage's propellant (lm::descent_propellant)
static constexpr double initial_mass = 15269.7;                     // kg
static constexpr double jet_thrust = 445.0;                         // n, 100 pounds
static constexpr double jet_arm = 1.68;                             // m, from each axis to the jet quads

// the autopilot's own curve fits of the vehicle against its mass, descent stage on (AOSTASK_AND_AOSJOB.agc): A/(mass
// + C) + B, the mass scaled at 2^16 kg. they give one jet's angular acceleration about p, q and r (1JACC, 1JACCQ,
// 1JACCR, scaled at pi/4 rad/s^2), and the gimbal pivot's distance below the centre of mass (L,PVT-CG, at 8 ft). the
// moments of inertia here are what make one jet's torque give that acceleration: the jets turn the vehicle as the
// autopilot expects, as the propellant goes
struct curve_fit_t
{
    double a, b, c;
    constexpr double at(double mass) const { return a / (mass / 65536.0 + c) + b; }
};
static constexpr std::array<curve_fit_t, 3> jet_acceleration_fits = { {
    { 0.0059347674, 0.002989, 0.008721 },
    { 0.0014979264, 0.018791, -0.068163 },
    { 0.0010451889, 0.021345, -0.066027 },
} };
static constexpr curve_fit_t pivot_below_cg_fit{ 0.0410511917, 0.155044, -0.025233 };

static lm::vec3_t inertia_for(double mass)
{
    const double torque = jet_thrust * jet_arm;
    const auto inertia = [&](std::size_t axis) { return torque / (jet_acceleration_fits[axis].at(mass) * std::numbers::pi / 4.0); };
    return { inertia(0), inertia(1), inertia(2) };
}

static double pivot_below_cg_for(double mass)
{
    return pivot_below_cg_fit.at(mass) * 8.0 * 0.3048;
}

// the descent engine, as Luminary's throttle routines see it (CONTROLLED_CONSTANTS.agc): FEXTRA is 4096 pulses for
// 51331 n, FMAX the throttle's stop, FMAXODD where the command saturates, THROTLAG the engine's lag, DPSVEX its
// exhaust velocity. the minimum is the manual throttle's, 10% of the rated 46.7 kn
static constexpr double thrust_per_pulse = 51330.902 / 4096.0;      // n
static constexpr double minimum_thrust = 4670.0;                    // n
static constexpr double maximum_thrust = 43454.677;                 // n
static constexpr int saturated_pulses = 3841;
static constexpr double engine_lag = 0.2;                           // s
static constexpr double exhaust_velocity = 2955.88868;              // m/s

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
static constexpr std::uint16_t engine_armed_signal = 000004, auto_throttle_signal = 000020, lgc_has_control = 001000;
// channel 11: the descent engine commanded on and off
static constexpr std::uint16_t engine_on = 010000, engine_off = 020000;
// channel 31, active low
static constexpr std::uint16_t attitude_hold_mode = 010000, auto_mode = 020000, out_of_detent = 040000;
// channel 33, active low: the landing radar's antenna, its data good, its range scale. channel 12: the computer
// commands the antenna to position 2
static constexpr std::uint16_t lr_range_good = 000020, lr_position_1 = 000040, lr_position_2 = 000100;
static constexpr std::uint16_t lr_velocity_good = 000200, lr_range_low_scale = 000400;
static constexpr std::uint16_t lr_position_2_command = 010000;
// channel 13: a radar reading under way
static constexpr std::uint16_t radar_activity = 000010;
// channel 13: the autopilot asks for the hand controller's angles in the counters
static constexpr std::uint16_t rhc_counter_enable = 000200, rhc_read = 000400;
// channel 12: the descent engine's trim gimbal, by bell motion
static constexpr std::uint16_t trim_minus_pitch = 000400, trim_plus_pitch = 001000, trim_minus_roll = 002000, trim_plus_roll = 004000;
static constexpr double trim_rate = 0.2 * lm::degrees, trim_limit = 6.0 * lm::degrees;

// a circular orbit 110 km up, until set_orbit() says otherwise
lm::c_lunar_module::c_lunar_module()
    : m_dry_mass(initial_mass - lm::descent_propellant)
{
    m_descent_engine.propellant = lm::descent_propellant;
    const double radius = lm::moon_radius + 110.0e3;
    set_orbit({ radius, 0.0, 0.0 }, { 0.0, std::sqrt(lm::moon_mu / radius), 0.0 });
}

// the vehicle moves on to wherever the computer's clock has got to. that includes the memory cycles the counters
// took (each pulse the pipas, cdus and the rest send it steals one, as in the hardware), which pass while the vehicle
// is being stepped: left out, its time fell behind the computer's, 0.4% under full thrust, and the computer's
// navigation ran 2.5 km ahead of it by touchdown
void lm::c_lunar_module::run(agc::c_computer& computer, double seconds)
{
    static constexpr double slice = 0.001;
    if (m_simulated_until < 0.0)
        m_simulated_until = computer.time();
    const double end = computer.time() + seconds;
    while (computer.time() < end)
    {
        computer.run_until(std::min(end, computer.time() + slice));
        const double now = computer.time();
        if (now <= m_simulated_until)
            break; // less than a memory cycle left
        step(computer, now - m_simulated_until);
        m_simulated_until = now;
    }
}

void lm::c_lunar_module::step(agc::c_computer& computer, double dt)
{
    if (dt <= 0.0)
        return;

    set_switches(computer, dt);
    drive_trim(computer, dt);
    run_engine(computer, dt);

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

    // the descent engine pushes up through the vehicle, its bell tilted by the trim gimbal: pitched, the thrust leans
    // along z and turns the vehicle about y; rolled, along y and about z. the pivot below the centre of mass is the arm.
    // which way is which is what the autopilot's trim control (GTS) needs to null the torque rather than feed it: the
    // other way round, flying P63 it ran the gimbal into its stops and lost the attitude
    if (m_descent_engine.thrust > 0.0)
    {
        const double thrust = m_descent_engine.thrust;
        const lm::vec3_t engine{ thrust, thrust * std::sin(m_engine_trim.roll), -thrust * std::sin(m_engine_trim.pitch) };
        force += engine;
        torque += lm::cross(lm::vec3_t{ -pivot_below_cg_for(mass()), 0.0, 0.0 }, engine);
    }

    // euler's equations, the rates in the vehicle's axes. landed, it turns with the moon, whatever the jets do
    if (m_touchdown.landed)
        m_rates = m_attitude * moon_rotation;
    else
    {
        const lm::vec3_t inertia = inertia_for(mass());
        const lm::vec3_t momentum{ inertia.x * m_rates.x, inertia.y * m_rates.y, inertia.z * m_rates.z };
        const lm::vec3_t net = torque - lm::cross(m_rates, momentum);
        m_rates += lm::vec3_t{ net.x / inertia.x, net.y / inertia.y, net.z / inertia.z } * dt;
    }

    const double rate = lm::length(m_rates);
    if (rate > 1e-12)
        m_attitude = lm::orthonormalized(lm::frame_rotation(m_rates / rate, rate * dt) * m_attitude);

    // the orbit: the moon's pull, which the pipas don't feel, and the thrust, which they do. semi-implicit euler, a
    // millisecond at a time
    lm::vec3_t delta_v = force / mass() * dt;
    const double r = lm::length(m_position);
    const lm::vec3_t gravity = m_position * (-lm::moon_mu / (r * r * r));
    if (!m_touchdown.landed)
    {
        m_velocity += gravity * dt + m_attitude.transposed() * delta_v;
        m_position += m_velocity * dt;
    }
    touch_down(-gravity, dt, delta_v);

    m_time += dt;
    m_imu.step(computer, m_attitude, delta_v, dt);
}

// the gear: the probes 67.2 in below the footpads, the footpads (a guess) 3.5 m below the centre of mass
static constexpr double probe_length = 67.2 * 0.0254;                     // m
static constexpr double pads_below_cg = 3.5;                              // m

double lm::c_lunar_module::height() const
{
    return lm::length(m_position) - surface_radius - pads_below_cg;
}

// the contact light as the probes touch, the footpads taking it, and from then sitting on the turning moon: what the
// pipas feel is the surface holding it up (support, the specific force, against gravity)
void lm::c_lunar_module::touch_down(const lm::vec3_t& support, double dt, lm::vec3_t& delta_v)
{
    touchdown_t& touchdown = m_touchdown;
    const lm::vec3_t vertical = m_position / lm::length(m_position);
    if (!touchdown.landed)
    {
        touchdown.contact = touchdown.contact || height() <= probe_length;
        if (height() > 0.0)
            return;

        const lm::vec3_t over_surface = m_velocity - lm::cross(moon_rotation, m_position);
        touchdown.landed = touchdown.contact = true;
        touchdown.time = m_time;
        touchdown.descent_rate = -lm::dot(over_surface, vertical);
        touchdown.drift = lm::length(over_surface - vertical * lm::dot(over_surface, vertical));
        m_position = vertical * (surface_radius + pads_below_cg);
        touchdown.position = m_position;
    }

    m_position += lm::cross(moon_rotation, m_position) * dt;
    m_velocity = lm::cross(moon_rotation, m_position);
    delta_v = m_attitude * support * dt;
}

const lm::touchdown_t& lm::c_lunar_module::touchdown() const
{
    return m_touchdown;
}

void lm::c_lunar_module::run_engine(agc::c_computer& computer, double dt)
{
    descent_engine_t& engine = m_descent_engine;

    // the throttle command counts the computer's pulses while the throttle is on auto, and is gone when the engine
    // isn't armed
    const int pulses = computer.take_thrust_pulses();
    if (!engine_armed)
        engine.throttle_pulses = 0;
    else if (auto_throttle)
        engine.throttle_pulses = std::clamp(engine.throttle_pulses + pulses, 0, saturated_pulses);

    const std::uint16_t channel11 = computer.channel(011);
    engine.on = engine_armed && !engine_stop && (channel11 & engine_on) && !(channel11 & engine_off) && engine.propellant > 0.0;

    const double commanded = engine.on ? std::clamp(engine.throttle_pulses * thrust_per_pulse, minimum_thrust, maximum_thrust) : 0.0;
    engine.thrust += (commanded - engine.thrust) * std::min(1.0, dt / engine_lag);
    if (engine.thrust < 1.0 && !engine.on)
        engine.thrust = 0.0;

    engine.propellant = std::max(0.0, engine.propellant - engine.thrust / exhaust_velocity * dt);
}

void lm::c_lunar_module::set_orbit(const lm::vec3_t& position, const lm::vec3_t& velocity)
{
    m_position = position;
    m_velocity = velocity;
}

const lm::vec3_t& lm::c_lunar_module::position() const
{
    return m_position;
}

const lm::vec3_t& lm::c_lunar_module::velocity() const
{
    return m_velocity;
}

void lm::c_lunar_module::set_attitude(const lm::mat3_t& attitude)
{
    m_attitude = attitude;
    m_rates = {};
}

double lm::c_lunar_module::altitude() const
{
    return lm::length(m_position) - lm::moon_radius;
}

double lm::c_lunar_module::mass() const
{
    return m_dry_mass + m_descent_engine.propellant;
}

const lm::descent_engine_t& lm::c_lunar_module::descent_engine() const
{
    return m_descent_engine;
}

const lm::landing_radar_t& lm::c_lunar_module::landing_radar() const
{
    return m_landing_radar;
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

// the landing radar as Luminary decodes it (P20-P25.agc, SERVICER.agc, CONTROLLED_CONSTANTS.agc): the antenna turned
// from the vehicle's axes as the stable member is from them by gimbal angles, outer LRALPHA and inner LRBETA (the pad
// load's, position 1 and 2); the range beam HBEAMANT in the antenna's axes; range 1.079 ft a count on low scale, 5
// times that on high (SCALADJ doubles it twice and adds it back: DASAMPL); velocity along the antenna's x, y and z (z = x cross
// y) at -0.644, 1.212 and 0.8668 ft/s a count, offset by 12288 (LVELBIAS)
static constexpr std::array<lm::gimbal_angles_t, 2> antenna_angles = { {
    { 0.0163371759 * lm::two_pi, 0.0665287037 * lm::two_pi, 0.0 },   // outer (x), inner (y), middle (z)
    { 0.0161680555 * lm::two_pi, 0.0001361111 * lm::two_pi, 0.0 },
} };
static constexpr lm::vec3_t range_beam{ -0.9374036082, 0.0, -0.3482448542 };
static constexpr double range_count = 1.079 * 0.3048;                     // m, low scale
static constexpr double low_scale_below = 2481.7 * 0.3048;                // m, HISCALIM
static constexpr std::array<double, 3> velocity_count = { -0.644 * 0.3048, 1.212 * 0.3048, 0.8668 * 0.3048 }; // m/s
static constexpr int velocity_bias = 12288;
// where it locks on: a guess, about where Apollo 11's did (39000 ft), with the beam no more than 70 degrees off the
// vertical
static constexpr double lock_range = 12000.0;                             // m
static const double lock_angle = std::cos(70.0 * lm::degrees);

void lm::c_lunar_module::run_radar(agc::c_computer& computer, double dt)
{
    landing_radar_t& radar = m_landing_radar;

    // the antenna: swinging to position 2 while commanded, and staying there
    static constexpr double antenna_swing = 8.0;
    if (radar.antenna_position == 1 && (computer.channel(012) & lr_position_2_command))
    {
        radar.antenna_travel += dt;
        if (radar.antenna_travel >= antenna_swing)
            radar.antenna_position = 2;
    }

    // the beams: the range beam's slant range to the surface sphere, the velocity over the turning surface in the
    // antenna's axes
    const lm::mat3_t antenna_to_body = lm::gimbal_matrix(antenna_angles[static_cast<std::size_t>(radar.antenna_position - 1)]);
    const lm::mat3_t body_to_reference = m_attitude.transposed();
    const lm::vec3_t beam = body_to_reference * (antenna_to_body * range_beam);
    const double along = lm::dot(m_position, beam);
    const double discriminant = along * along - (lm::dot(m_position, m_position) - surface_radius * surface_radius);
    const double range = discriminant >= 0.0 ? -along - std::sqrt(discriminant) : -1.0;
    const lm::vec3_t down = m_position / -lm::length(m_position);
    const bool locked = range > 0.0 && range < lock_range && lm::dot(beam, down) > lock_angle;
    radar.range_good = radar.velocity_good = locked;
    radar.slant_range = locked ? range : 0.0;
    radar.low_scale = locked && range < low_scale_below;
    const lm::vec3_t over_surface = m_velocity - lm::cross(moon_rotation, m_position);
    radar.velocity = antenna_to_body.transposed() * (m_attitude * over_surface);

    computer.set_input(033, lr_position_1 | lr_position_2 | lr_range_good | lr_velocity_good | lr_range_low_scale,
        (radar.antenna_position == 1 ? lr_position_2 : lr_position_1) | (radar.range_good ? 0 : lr_range_good) |
        (radar.velocity_good ? 0 : lr_velocity_good) | (radar.low_scale ? 0 : lr_range_low_scale));

    // its readings: one wanted starts (over again, if one was under way), the activity bit dropped abandons it, and
    // it's answered once it's taken its time. the rendezvous radar's (select bit 3 clear) aren't, there's none
    static constexpr double reading_time = 0.080;
    if (radar.answered != computer.radar_requests())
    {
        radar.answered = computer.radar_requests();
        radar.reading = (computer.radar_select() & 4) ? 0.0 : -1.0;
    }
    else if (!(computer.channel(013) & radar_activity))
        radar.reading = -1.0;
    if (radar.reading >= 0.0)
    {
        radar.reading += dt;
        if (radar.reading >= reading_time)
        {
            radar.reading = -1.0;
            computer.radar_reading(radar_value(computer.radar_select() & 3));
        }
    }
}

// what goes into RNRAD, an unsigned 15 bits: for select 0-2 the velocity along the antenna's x, y, z, for 3 the range
std::uint16_t lm::c_lunar_module::radar_value(int select) const
{
    const landing_radar_t& radar = m_landing_radar;
    double counts = 0.0;
    if (select == 3)
        counts = radar.slant_range / (radar.low_scale ? range_count : 5.0 * range_count);
    else
    {
        const std::array<double, 3> along{ radar.velocity.x, radar.velocity.y, radar.velocity.z };
        counts = along[static_cast<std::size_t>(select)] / velocity_count[static_cast<std::size_t>(select)] + velocity_bias;
    }
    return static_cast<std::uint16_t>(std::clamp(std::lround(counts), 0l, 077777l));
}

// a counter's value from -1 to 1 of the hand controller's travel, 42 counts at full deflection, in ones' complement
static int hand_controller_counts(double deflection)
{
    const int counts = static_cast<int>(std::lround(std::clamp(deflection, -1.0, 1.0) * 42.0));
    return counts < 0 ? 077777 & ~(-counts) : counts;
}

void lm::c_lunar_module::set_switches(agc::c_computer& computer, double dt)
{
    computer.set_input(030, lgc_has_control | engine_armed_signal | auto_throttle_signal,
        (guidance_pgns ? 0 : lgc_has_control) | (engine_armed ? 0 : engine_armed_signal) | (auto_throttle ? 0 : auto_throttle_signal));

    const bool deflected = std::abs(hand_controller.pitch) > 0.02 || std::abs(hand_controller.yaw) > 0.02 || std::abs(hand_controller.roll) > 0.02;
    std::uint16_t channel31 = attitude_hold_mode | auto_mode | out_of_detent;
    if (mode == lm::mode_control::attitude_hold)
        channel31 &= static_cast<std::uint16_t>(~attitude_hold_mode);
    else if (mode == lm::mode_control::automatic)
        channel31 &= static_cast<std::uint16_t>(~auto_mode);
    if (deflected)
        channel31 &= static_cast<std::uint16_t>(~out_of_detent);
    computer.set_input(031, attitude_hold_mode | auto_mode | out_of_detent, channel31);

    run_radar(computer, dt);

    // read into the counters while the autopilot has them enabled
    const std::uint16_t channel13 = computer.channel(013);
    if ((channel13 & rhc_counter_enable) && (channel13 & rhc_read))
    {
        computer.set_counter(agc::counter_q_rhc, hand_controller_counts(hand_controller.pitch));
        computer.set_counter(agc::counter_p_rhc, hand_controller_counts(hand_controller.yaw));
        computer.set_counter(agc::counter_r_rhc, hand_controller_counts(hand_controller.roll));
    }
}

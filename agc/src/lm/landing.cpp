#include "lm/landing.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <string_view>

#include "agc/computer.hpp"
#include "agc/scaled.hpp"
#include "lm/lunar_module.hpp"
#include "yul/assembler.hpp"

// the moon's orientation: three angles that go on at their rates from TEPHEM's epoch (NEWANGLE), and the angle
// between the moon's mean equator and the ecliptic (COSI, SINI). the constants of CONTROLLED_CONSTANTS.agc, in the SI
// units their comments give
static constexpr double node_0 = 6.19653663041, node_rate = -1.07047011e-8;   // NODIO, NODDOT: rad, rad/s
static constexpr double f_0 = 5.20932947829, f_rate = 2.67240410e-6;          // FSUBO, FDOT
static constexpr double b_0 = 0.40916190299, b_rate = -7.19757301e-14;        // BSUBO, BDOT
static constexpr double inclination = 5521.5 / 3600.0 * lm::degrees;          // 1 degree 32.1 minutes

// how fast the moon turns, as the landing guidance has it (OMEGMOON, 2.66169947 E-8 rad/cs)
static constexpr double moon_rate = 2.66169947e-6;                            // rad/s

// from the ignition algorithm (THE_LUNAR_LANDING.agc): the guidance starts its descent this long before TLAND
static constexpr double guidance_duration = 664.400314;                       // GUIDDURN, s

lm::mat3_t lm::moon_matrix(double seconds)
{
    const double b = b_0 + b_rate * seconds, f = f_0 + f_rate * seconds, node = node_0 + node_rate * seconds;
    const double cos_b = std::cos(b), sin_b = std::sin(b);
    const double cos_i = std::cos(inclination), sin_i = std::sin(inclination);

    const lm::vec3_t a{ std::cos(node), cos_b * std::sin(node), sin_b * std::sin(node) };
    const lm::vec3_t bv{ -std::sin(node), cos_b * std::cos(node), sin_b * std::cos(node) };
    const lm::vec3_t c{ 0.0, -sin_b, cos_b };
    const lm::vec3_t d = bv * cos_i - c * sin_i;
    return { { -(a * std::cos(f) + d * std::sin(f)), a * std::sin(f) - d * std::cos(f), bv * sin_i + c * cos_i } };
}

lm::vec3_t lm::moon_fixed_to_reference(const lm::vec3_t& moon_fixed, double seconds, const lm::vec3_t& libration)
{
    return lm::moon_matrix(seconds).transposed() * (moon_fixed + lm::cross(libration, moon_fixed));
}

// ---------------------------------------------------------------------------------------------------------------

static int erasable_address(const yul::assembly_t& assembly, std::string_view name)
{
    const auto symbol = assembly.symbols.find(std::string(name));
    if (symbol == assembly.symbols.end() || symbol->second.value >= 04000)
        throw std::runtime_error("no erasable symbol " + std::string(name));
    return symbol->second.value;
}

static double read_scaled(const agc::c_computer& computer, int address, int words, int scale)
{
    std::array<std::uint16_t, 3> value{};
    for (int i = 0; i < words; ++i)
        value[static_cast<std::size_t>(i)] = computer.erasable(address + i);
    return agc::from_scaled(std::span(value.data(), static_cast<std::size_t>(words)), scale);
}

static lm::vec3_t read_vector(const agc::c_computer& computer, int address, int scale)
{
    return { read_scaled(computer, address, 2, scale), read_scaled(computer, address + 2, 2, scale), read_scaled(computer, address + 4, 2, scale) };
}

static void write_scaled(agc::c_computer& computer, int address, double value, int scale, int words)
{
    const auto encoded = agc::to_scaled(value, scale, words);
    for (int i = 0; i < words; ++i)
        computer.load_erasable(address + i, encoded[static_cast<std::size_t>(i)]);
}

static void write_vector(agc::c_computer& computer, int address, const lm::vec3_t& value, int scale)
{
    for (int i = 0; i < 3; ++i)
        write_scaled(computer, address + 2 * i, value[i], scale, 2);
}

static void set_flag(agc::c_computer& computer, int address, std::uint16_t bit)
{
    computer.load_erasable(address, static_cast<std::uint16_t>(computer.erasable(address) | bit));
}

// two bodies, the moon a point: runge-kutta, steps of a quarter second, either way in time
static void propagate(lm::vec3_t& position, lm::vec3_t& velocity, double seconds)
{
    const auto gravity = [](const lm::vec3_t& r) { const double d = lm::length(r); return r * (-lm::moon_mu / (d * d * d)); };
    const int steps = static_cast<int>(std::ceil(std::abs(seconds) / 0.25));
    const double h = seconds / steps;
    for (int i = 0; i < steps; ++i)
    {
        const lm::vec3_t k1v = gravity(position), k1r = velocity;
        const lm::vec3_t k2v = gravity(position + k1r * (h / 2)), k2r = velocity + k1v * (h / 2);
        const lm::vec3_t k3v = gravity(position + k2r * (h / 2)), k3r = velocity + k2v * (h / 2);
        const lm::vec3_t k4v = gravity(position + k3r * h), k4r = velocity + k3v * h;
        position += (k1r + k2r * 2.0 + k3r * 2.0 + k4r) * (h / 6);
        velocity += (k1v + k2v * 2.0 + k3v * 2.0 + k4v) * (h / 6);
    }
}

lm::landing_start_t lm::plan_landing(const agc::c_computer& computer, const yul::assembly_t& assembly, double lead)
{
    // the pad load, as the computer has it
    const double tland = read_scaled(computer, erasable_address(assembly, "TLAND"), 2, 28) / 100.0;
    const double tephem = read_scaled(computer, erasable_address(assembly, "TEPHEM"), 3, 42) / 100.0;
    const lm::vec3_t rls = read_vector(computer, erasable_address(assembly, "RLS"), 27);
    const lm::vec3_t libration = read_vector(computer, erasable_address(assembly, "504LM"), 0);
    const double rignx = read_scaled(computer, erasable_address(assembly, "RIGNX"), 2, 24);
    const double rignz = read_scaled(computer, erasable_address(assembly, "RIGNZ"), 2, 24);
    const double vign = read_scaled(computer, erasable_address(assembly, "VIGN"), 2, 10) * 100.0;
    const double zoomtime = read_scaled(computer, erasable_address(assembly, "ZOOMTIME"), 1, 14) / 100.0;

    landing_start_t start;
    start.tland = tland;
    start.site = lm::moon_fixed_to_reference(rls, tland + tephem, libration);

    // the landing site alignment, as P63 takes it for its guidance frame: x up through the site at TLAND, z downrange
    // (the orbit eastward, its pole the moon's north turned square to x), y = z cross x
    const lm::vec3_t up = start.site / lm::length(start.site);
    const lm::vec3_t north = lm::moon_fixed_to_reference({ 0.0, 0.0, 1.0 }, tland + tephem, {});
    start.moon_rotation = north * moon_rate;
    const lm::vec3_t pole = north - up * lm::dot(north, up);
    const lm::vec3_t h = pole / lm::length(pole);
    const lm::vec3_t downrange = lm::cross(h, up);
    start.refsmmat = { { up, lm::cross(downrange, up), downrange } };

    // where the guidance starts, in that frame: RIGN from the site, horizontal and downrange at VIGN
    const lm::vec3_t at{ lm::length(start.site) + rignx, 0.0, rignz };
    const lm::vec3_t along = lm::cross({ 0.0, -1.0, 0.0 }, at / lm::length(at));
    lm::vec3_t position = start.refsmmat.transposed() * at;
    lm::vec3_t velocity = start.refsmmat.transposed() * (along * vign);

    // back to lead seconds before ignition, which is ZOOMTIME before the guidance starts
    const double guidance_start = tland - guidance_duration;
    start.ignition = guidance_start - zoomtime;
    start.time = start.ignition - lead;
    propagate(position, velocity, start.time - guidance_start);
    start.position = position;
    start.velocity = velocity;
    return start;
}

void lm::align_for_landing(lm::c_lunar_module& lunar_module, const lm::landing_start_t& start)
{
    lunar_module.set_attitude(start.refsmmat);
}

void lm::uplink_landing(agc::c_computer& computer, lm::c_lunar_module& lunar_module, const lm::landing_start_t& start, const yul::assembly_t& assembly)
{
    // the clock: TIME2 and TIME1, centiseconds since launch
    const auto centiseconds = static_cast<long long>(std::llround(start.time * 100.0));
    computer.load_erasable(024, static_cast<std::uint16_t>((centiseconds >> 14) & 037777));
    computer.load_erasable(025, static_cast<std::uint16_t>(centiseconds & 037777));

    // the permanent state vectors, just rectified: the conic each is on is where it is (RCV, VCV), nothing off it
    // (DELTA, NUV) and no time on it (TC, XKEP). in the moon's sphere positions are m at 2^27, velocities m/cs at 2^5,
    // times cs at 2^28 (INTEGRATION_INITIALIZATION.agc). the lunar module's where it is, the command module's (which the
    // programs integrate too, and divide by its distance from the moon doing it) a circular orbit 60 nmi up, overhead
    const auto write_state = [&](std::string_view first, const lm::vec3_t& position, const lm::vec3_t& velocity) {
        const int state = erasable_address(assembly, first);
        for (int i = 0; i < 42; ++i)
            computer.load_erasable(state + i, 0);
        write_vector(computer, state, position, 27);                   // RRECT
        write_vector(computer, state + 6, velocity / 100.0, 5);        // VRECT
        write_scaled(computer, state + 12, start.time * 100.0, 28, 2); // TET
        write_vector(computer, state + 26, position, 27);              // RCV
        write_vector(computer, state + 32, velocity / 100.0, 5);       // VCV
    };
    write_state("RRECTLEM", start.position, start.velocity);
    const double csm_radius = lm::length(start.site) + 60.0 * 1852.0;
    const lm::vec3_t up = start.position / lm::length(start.position);
    const lm::vec3_t along = lm::cross(lm::cross(start.position, start.velocity), up);
    write_state("RRECTCSM", up * csm_radius, along / lm::length(along) * std::sqrt(lm::moon_mu / csm_radius));

    // REFSMMAT, its elements at 2^1
    const int refsmmat = erasable_address(assembly, "REFSMMAT");
    for (int row = 0; row < 3; ++row)
        write_vector(computer, refsmmat + 6 * row, start.refsmmat.rows[static_cast<std::size_t>(row)], 1);

    // REFSMMAT good, and both state vectors (and the integration) in the moon's sphere
    set_flag(computer, erasable_address(assembly, "FLAGWRD3"), 010000);  // REFSMFLG, bit 13
    set_flag(computer, erasable_address(assembly, "FLAGWRD8"), 006000);  // CMOONFLG bit 12, LMOONFLG bit 11
    set_flag(computer, erasable_address(assembly, "FLAGWRD0"), 004000);  // MOONFLAG, bit 12

    lunar_module.set_orbit(start.position, start.velocity);
    lunar_module.surface_radius = lm::length(start.site);
    lunar_module.moon_rotation = start.moon_rotation;
}

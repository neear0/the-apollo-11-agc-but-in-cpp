#include <cmath>
#include <cstdio>
#include <filesystem>
#include <format>
#include <fstream>
#include <memory>
#include <optional>
#include <print>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "agc/computer.hpp"
#include "lm/lunar_module.hpp"
#include "lm/pad_load.hpp"
#include "yul/assembler.hpp"

// a rope transcribed from a printed listing, Virtual AGC's .binsource format: BANK=nn lines, then the bank's words in
// octal, @ for a word nothing was assembled into, ; to the end of a line is a comment
struct reference_rope_t
{
    agc::rope_t words{};
    std::array<std::array<bool, agc::bank_size>, agc::fixed_bank_count> present{};
};

static std::optional<reference_rope_t> load_binsource(const std::filesystem::path& path)
{
    std::ifstream stream(path);
    if (!stream)
        return std::nullopt;

    reference_rope_t rope;
    std::size_t bank = 0, offset = 0;
    std::string line;
    while (std::getline(stream, line))
    {
        if (const std::size_t comment = line.find(';'); comment != std::string::npos)
            line.resize(comment);

        if (line.starts_with("BANK="))
        {
            bank = std::stoul(line.substr(5), nullptr, 8);
            offset = 0;
            continue;
        }

        std::istringstream words(line);
        std::string word;
        while (words >> word && bank < agc::fixed_bank_count && offset < agc::bank_size)
        {
            if (word != "@")
            {
                rope.words[bank][offset] = static_cast<std::uint16_t>(std::stoul(word, nullptr, 8));
                rope.present[bank][offset] = true;
            }
            ++offset;
        }
    }
    return rope;
}

static int assemble(const std::filesystem::path& main_file, const std::optional<std::filesystem::path>& compare_with)
{
    const auto assembly = yul::assemble(main_file);
    if (!assembly)
    {
        for (const std::string& error : assembly.error())
            std::println("{}", error);
        std::println("{} errors", assembly.error().size());
        return 1;
    }

    int words = 0;
    for (const auto& bank : assembly->used)
        for (const bool used : bank)
            words += used;
    std::println("assembled {} words, {} symbols", words, assembly->symbols.size());

    if (!compare_with)
        return 0;

    const std::optional<reference_rope_t> reference = load_binsource(*compare_with);
    if (!reference)
    {
        std::println("can't read {}", compare_with->string());
        return 1;
    }

    int mismatches = 0, shown = 0;
    for (std::size_t bank = 0; bank < agc::fixed_bank_count; ++bank)
    {
        int bank_mismatches = 0;
        for (std::size_t offset = 0; offset < agc::bank_size; ++offset)
        {
            const bool ours = assembly->used[bank][offset];
            const bool theirs = reference->present[bank][offset];
            const std::uint16_t our_word = ours ? assembly->rope[bank][offset] : 0;
            const std::uint16_t their_word = theirs ? reference->words[bank][offset] : 0;
            if (ours == theirs && our_word == their_word)
                continue;

            ++bank_mismatches;
            if (shown++ < 40)
            {
                const int origin = ours ? assembly->origin[bank][offset] : -1;
                std::println("  {:02o},{:04o}: ours {}  reference {}  {}", bank, offset + 02000,
                    ours ? std::format("{:05o}", our_word) : std::string("  @  "), theirs ? std::format("{:05o}", their_word) : std::string("  @  "),
                    origin >= 0 && ours ? assembly->sources[static_cast<std::size_t>(origin)] : std::string("(checksum)"));
            }
        }
        if (bank_mismatches)
            std::println("bank {:02o}: {} words differ", bank, bank_mismatches);
        mismatches += bank_mismatches;
    }

    if (mismatches == 0)
        std::println("matches the reference rope word for word");
    else
        std::println("{} words differ", mismatches);
    return mismatches == 0 ? 0 : 2;
}

static std::string digit_text(std::span<const std::int8_t> digits)
{
    std::string text;
    for (const std::int8_t digit : digits)
        text += digit == agc::blank_digit ? ' ' : static_cast<char>('0' + digit);
    return text;
}

static void print_dsky(const agc::dsky_state_t& state, double time)
{
    std::println("--- t = {:.2f} s", time);
    std::println("  PROG [{}]  VERB [{}]  NOUN [{}]{}{}", digit_text(state.prog), digit_text(state.verb), digit_text(state.noun),
        state.comp_acty ? "  COMP ACTY" : "", state.verb_noun_flash ? "  (flashing)" : "");
    for (std::size_t i = 0; i < state.registers.size(); ++i)
    {
        const agc::display_register_t& reg = state.registers[i];
        const char sign = reg.sign == agc::register_sign::plus ? '+' : reg.sign == agc::register_sign::minus ? '-' : ' ';
        std::println("  R{}   [{}{}]", i + 1, sign, digit_text(reg.digits));
    }

    const agc::lights_t& l = state.lights;
    const std::pair<bool, const char*> lights[] = {
        { l.uplink_acty, "UPLINK ACTY" }, { l.no_att, "NO ATT" }, { l.stby, "STBY" }, { l.key_rel, "KEY REL" }, { l.opr_err, "OPR ERR" },
        { l.prio_disp, "PRIO DISP" }, { l.no_dap, "NO DAP" }, { l.temp, "TEMP" }, { l.gimbal_lock, "GIMBAL LOCK" }, { l.prog, "PROG" },
        { l.restart, "RESTART" }, { l.tracker, "TRACKER" }, { l.alt, "ALT" }, { l.vel, "VEL" },
    };
    std::string lit;
    for (const auto& [on, name] : lights)
        if (on)
            lit += std::format(" [{}]", name);
    std::println("  lights:{}", lit.empty() ? " none" : lit);
}

// a cdu counter in degrees, two's complement
static double cdu_degrees(std::uint16_t counter)
{
    const int value = counter & 077777;
    return (value >= 040000 ? value - 0100000 : value) * 360.0 / 32768.0;
}

// erasable words to show with the state, by symbol: name and linear address
static std::vector<std::pair<std::string, int>> watched;

static void print_computer(const agc::c_computer& computer, const lm::c_lunar_module& lm)
{
    print_dsky(computer.dsky().state(), computer.time());
    if (!watched.empty())
    {
        std::string words;
        for (const auto& [name, address] : watched)
            words += std::format(" {} {:05o}", name, computer.erasable(address));
        std::println("  erasable:{}", words);
    }
    std::println("  restarts: {}  channel 77: {:05o}{}", computer.restarts(), computer.channel(077), computer.standby() ? "  (standby)" : "");

    const lm::gimbal_angles_t& gimbals = lm.imu().gimbals();
    const lm::vec3_t& rates = lm.rates();
    std::println("  gimbals  outer {:8.3f}  inner {:8.3f}  middle {:8.3f}{}{}", gimbals.outer / lm::degrees, gimbals.inner / lm::degrees,
        gimbals.middle / lm::degrees, lm.imu().coarse_align() ? "  (coarse align)" : "", lm.imu().turn_on_request() ? "  (turn-on request)" : "");
    std::println("  trim     pitch {:8.3f}  roll  {:8.3f}", lm.engine_trim().pitch / lm::degrees, lm.engine_trim().roll / lm::degrees);
    const lm::descent_engine_t& engine = lm.descent_engine();
    std::println("  orbit    altitude {:9.1f} m  speed {:7.1f} m/s  mass {:7.1f} kg   engine{}{}  thrust {:7.0f} n  throttle {} pulses",
        lm.altitude(), lm::length(lm.velocity()), lm.mass(), lm.engine_armed ? " armed" : "", engine.on ? " on" : " off",
        engine.thrust, engine.throttle_pulses);
    std::println("  cdus     x     {:8.3f}  y     {:8.3f}  z      {:8.3f}", cdu_degrees(computer.erasable(agc::counter_cdux)),
        cdu_degrees(computer.erasable(agc::counter_cduy)), cdu_degrees(computer.erasable(agc::counter_cduz)));
    std::println("  rates    p     {:8.3f}  q     {:8.3f}  r      {:8.3f} deg/s   jets {:05o}  channels 12 {:05o} 14 {:05o} 30 {:05o} 31 {:05o}",
        rates.x / lm::degrees, rates.y / lm::degrees, rates.z / lm::degrees, lm.jets(),
        computer.channel(012), computer.channel(014), computer.channel(030), computer.channel(031));
}

static std::optional<agc::dsky_key> key_for(char c)
{
    if (c >= '0' && c <= '9')
        return static_cast<agc::dsky_key>(c - '0');
    switch (c)
    {
    case 'V': return agc::dsky_key::verb;
    case 'N': return agc::dsky_key::noun;
    case 'E': return agc::dsky_key::enter;
    case 'P': return agc::dsky_key::proceed;
    case 'C': return agc::dsky_key::clear;
    case 'R': return agc::dsky_key::reset;
    case 'K': return agc::dsky_key::key_release;
    case '+': return agc::dsky_key::plus;
    case '-': return agc::dsky_key::minus;
    default:  return std::nullopt;
    }
}

// the program booted in the lunar module, with the pad load
struct flight_t
{
    explicit flight_t(const yul::assembly_t& assembly)
        : computer(assembly.rope)
    {
        for (const std::string& name : lm::load_pad(computer, assembly))
            std::println("no pad load symbol {}", name);
    }

    agc::c_computer computer;
    lm::c_lunar_module lm;
};

// types the keys into the dsky (V35E, N for noun, E enter, P pro, R rset, C clr, K key rel). a space waits a second,
// W ten seconds, M a minute, S holds pro for 3 seconds (standby and back), ? shows the state there. the crew's side
// in lowercase: i turns the imu off or on, e arms the descent engine or disarms it, o h a set the mode control to
// off, attitude hold or auto, and the hand controller is held over for a second by p q (pitch up, down), y z (yaw
// right, left), r l (roll right, left)
static void fly(flight_t& flight, std::string_view keys)
{
    agc::c_computer& computer = flight.computer;
    lm::c_lunar_module& lm = flight.lm;

    const auto stick = [&](double lm::hand_controller_t::* axis, double deflection) {
        lm.hand_controller.*axis = deflection;
        lm.run(computer, 1.0);
        lm.hand_controller.*axis = 0.0;
    };

    for (const char c : keys)
    {
        switch (c)
        {
        case ' ': lm.run(computer, 1.0); continue;
        case 'W': lm.run(computer, 10.0); continue;
        case 'M': lm.run(computer, 60.0); continue;
        case '?': print_computer(computer, lm); continue;
        case 'i': lm.imu().operate = !lm.imu().operate; continue;
        case 'e': lm.engine_armed = !lm.engine_armed; continue;
        case 'o': lm.mode = lm::mode_control::off; continue;
        case 'h': lm.mode = lm::mode_control::attitude_hold; continue;
        case 'a': lm.mode = lm::mode_control::automatic; continue;
        case 'p': stick(&lm::hand_controller_t::pitch, 1.0); continue;
        case 'q': stick(&lm::hand_controller_t::pitch, -1.0); continue;
        case 'y': stick(&lm::hand_controller_t::yaw, 1.0); continue;
        case 'z': stick(&lm::hand_controller_t::yaw, -1.0); continue;
        case 'r': stick(&lm::hand_controller_t::roll, 1.0); continue;
        case 'l': stick(&lm::hand_controller_t::roll, -1.0); continue;
        case 'S':
            computer.dsky().hold_proceed(true);
            lm.run(computer, 3.0);
            computer.dsky().hold_proceed(false);
            lm.run(computer, 0.3);
            continue;
        default:
            break;
        }

        if (const std::optional<agc::dsky_key> key = key_for(c))
        {
            computer.dsky().press_key(*key);
            lm.run(computer, 0.3);
        }
    }
}

// boots the program in the lunar module, waits, flies the keys and shows the dsky and the spacecraft at the end
static int run(const std::filesystem::path& main_file, std::string_view keys, double settle, std::string_view watch)
{
    const auto assembly = yul::assemble(main_file);
    if (!assembly)
    {
        for (const std::string& error : assembly.error())
            std::println("{}", error);
        return 1;
    }

    // symbols to watch, erasable ones only: comma separated, or semicolon separated if any has a comma (L,PVT-CG)
    const char separator = watch.contains(';') ? ';' : ',';
    for (std::size_t start = 0; start < watch.size(); )
    {
        std::size_t end = watch.find(separator, start);
        if (end == std::string_view::npos)
            end = watch.size();
        const std::string name(watch.substr(start, end - start));
        if (const auto symbol = assembly->symbols.find(name); symbol != assembly->symbols.end() && symbol->second.value < 04000)
            watched.emplace_back(name, symbol->second.value);
        else
            std::println("no erasable symbol {}", name);
        start = end + 1;
    }

    const auto flight = std::make_unique<flight_t>(*assembly);
    flight->lm.run(flight->computer, settle);
    print_computer(flight->computer, flight->lm);

    fly(*flight, keys);
    if (!keys.empty())
    {
        flight->lm.run(flight->computer, settle);
        print_computer(flight->computer, flight->lm);
    }
    return 0;
}

// a two-digit dsky field as a number, -1 blank
static int field_value(std::span<const std::int8_t> digits)
{
    int value = 0;
    for (const std::int8_t digit : digits)
    {
        if (digit == agc::blank_digit)
            return -1;
        value = value * 10 + digit;
    }
    return value;
}

// what the readme says works, checked: both ropes against the 1969 ones, and Luminary flying the lunar module. each
// flight starts from power-on and waits out the imu's turn-on (2 minutes) first. the exit code is the failures
static int test(const std::filesystem::path& root)
{
    int failures = 0;
    const auto check = [&](bool ok, std::string_view what) {
        std::println("  {}  {}", ok ? "ok  " : "FAIL", what);
        failures += !ok;
    };

    for (const std::string_view name : { "Luminary099", "Comanche055" })
    {
        std::println("{}", name);
        const std::filesystem::path main_file = root / name / "MAIN.agc";
        const std::filesystem::path rope = root / "tests" / "ropes" / std::format("{}.binsource", name);
        check(assemble(main_file, rope) == 0, "assembles to the 1969 rope");
    }

    const auto assembly = yul::assemble(root / "Luminary099" / "MAIN.agc");
    if (!assembly)
    {
        check(false, "Luminary099 assembles");
        return failures;
    }
    const auto failreg = assembly->symbols.find("FAILREG");
    if (failreg == assembly->symbols.end())
    {
        check(false, "FAILREG found");
        return failures;
    }

    // a fresh start from zeroed erasable is alarm 1107 in the first of the three alarm registers: anything in the
    // other two is an alarm since
    const auto no_new_alarms = [&](const flight_t& flight) {
        return flight.computer.erasable(failreg->second.value + 1) == 0 && flight.computer.erasable(failreg->second.value + 2) == 0;
    };
    const auto near = [](double radians, double degrees, double tolerance) {
        return std::abs(radians / lm::degrees - degrees) <= tolerance;
    };
    const auto shows = [](const flight_t& flight, int verb, int noun) {
        const agc::dsky_state_t& state = flight.computer.dsky().state();
        return field_value(state.verb) == verb && field_value(state.noun) == noun;
    };
    // the autopilot holds attitude to its 5 degree deadband, about each of the vehicle's axes: two of them off by that
    // much put a gimbal 7.1 degrees out. after V49 it overshoots to the deadband's edge and drifts back
    static constexpr double deadband = 7.5;
    const auto at_gimbals = [&](const flight_t& flight, double outer, double inner, double middle) {
        const lm::gimbal_angles_t& gimbals = flight.lm.imu().gimbals();
        return near(gimbals.outer, outer, deadband) && near(gimbals.inner, inner, deadband) && near(gimbals.middle, middle, deadband);
    };
    static constexpr std::string_view v49_to_30_20_0 = "V37E00E V49E V25E +03000E +02000E +00000E P P MM";

    std::println("Luminary099 in the lunar module");
    {
        const auto flight = std::make_unique<flight_t>(*assembly);
        fly(*flight, "MM");
        check(!flight->computer.dsky().state().lights.no_att, "imu turned on: NO ATT out after 90 s");
        check(at_gimbals(*flight, 0.0, 0.0, 0.0), "holding attitude after turn-on");
        check(flight->computer.restarts() == 0 && no_new_alarms(*flight), "no restarts or alarms");
    }
    {
        const auto flight = std::make_unique<flight_t>(*assembly);
        fly(*flight, std::format("MM{}", v49_to_30_20_0));
        check(at_gimbals(*flight, 30.0, 20.0, 0.0), "V49 maneuvers to gimbals 30 20 0");
        check(flight->computer.restarts() == 0 && no_new_alarms(*flight), "no restarts or alarms");
    }
    {
        const auto flight = std::make_unique<flight_t>(*assembly);
        fly(*flight, "MMV48E P P P M W W W W");
        const lm::engine_trim_t& trim = flight->lm.engine_trim();
        check(shows(*flight, 50, 48), "V48 trims the descent engine to V50N48");
        check(near(trim.pitch, 0.0, 0.05) && near(trim.roll, 0.0, 0.05), "trim gimbal back at centre");
        check(no_new_alarms(*flight), "no alarms (1204 before PITTIME/ROLLTIME were pad loaded)");
        fly(*flight, std::format("P {}", v49_to_30_20_0));
        check(at_gimbals(*flight, 30.0, 20.0, 0.0) && !flight->computer.dsky().state().lights.opr_err, "V49 after V48 still maneuvers");
    }
    {
        const auto flight = std::make_unique<flight_t>(*assembly);
        fly(*flight, "MMhpp WW");
        check(flight->lm.imu().gimbals().inner / lm::degrees > 20.0, "attitude hold: the hand controller pitches it");
        check(std::abs(flight->lm.rates().y / lm::degrees) < 0.6, "and it stops when let go");
    }
    {
        const auto flight = std::make_unique<flight_t>(*assembly);
        fly(*flight, "MMV37E06E S");
        check(flight->computer.standby() && flight->computer.dsky().state().lights.stby, "P06 and held pro: standby");
        fly(*flight, "W S MM");
        check(!flight->computer.standby() && shows(*flight, 37, -1), "held pro again: back, asking for a program");
    }

    std::println("{}", failures == 0 ? "all passed" : std::format("{} failed", failures));
    return failures;
}

static void usage()
{
    std::println("usage:");
    std::println("  agc_check assemble <MAIN.agc> [--compare <rope.binsource>]");
    std::println("  agc_check run <MAIN.agc> [keys] [seconds to settle, default 3] [erasable symbols to show, comma or semicolon separated]");
    std::println("  agc_check test [repo root, default .]");
}

int main(int argc, char** argv)
{
    const std::vector<std::string_view> args(argv + 1, argv + argc);
    if (args.size() >= 2 && args[0] == "assemble")
    {
        std::optional<std::filesystem::path> compare_with;
        if (args.size() >= 4 && args[2] == "--compare")
            compare_with = args[3];
        return assemble(args[1], compare_with);
    }

    if (args.size() >= 2 && args[0] == "run")
    {
        const std::string_view keys = args.size() >= 3 ? args[2] : std::string_view();
        const double settle = args.size() >= 4 ? std::stod(std::string(args[3])) : 3.0;
        return run(args[1], keys, settle, args.size() >= 5 ? args[4] : std::string_view());
    }

    if (args.size() >= 1 && args[0] == "test")
        return test(args.size() >= 2 ? std::filesystem::path(args[1]) : std::filesystem::path("."));

    usage();
    return 1;
}

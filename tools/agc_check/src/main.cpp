#include <algorithm>
#include <cctype>
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
#include "agc/scaled.hpp"
#include "lm/lunar_module.hpp"
#include "lm/landing.hpp"
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
    std::println("  radar    channels 13 {:05o} 33 {:05o}  requests {}  antenna position {}", computer.channel(013), computer.channel(033),
        computer.radar_requests(), lm.landing_radar().antenna_position);
    const lm::landing_radar_t& radar = lm.landing_radar();
    std::println("  lr      {}{}  slant range {:8.1f} m  velocity x {:8.2f} y {:8.2f} z {:8.2f} m/s   above the surface {:8.1f} m",
        radar.range_good ? "locked" : "no lock", radar.low_scale ? " (low scale)" : "", radar.slant_range,
        radar.velocity.x, radar.velocity.y, radar.velocity.z, lm::length(lm.position()) - lm.surface_radius);
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

// the descent orbit, lead seconds before powered descent: aligned before the imu's turn-on, the uplink after it
static lm::landing_start_t start_landing(flight_t& flight, const yul::assembly_t& assembly, double lead)
{
    static constexpr double imu_turn_on = 120.0;
    const lm::landing_start_t start = lm::plan_landing(flight.computer, assembly, lead);
    lm::align_for_landing(flight.lm, start);
    flight.lm.run(flight.computer, imu_turn_on);
    lm::uplink_landing(flight.computer, flight.lm, start, assembly);
    return start;
}

// seconds since launch as hh:mm:ss.ss
static std::string get_text(double seconds)
{
    const int hours = static_cast<int>(seconds / 3600.0);
    const int minutes = static_cast<int>((seconds - hours * 3600.0) / 60.0);
    return std::format("{:03}:{:02}:{:05.2f}", hours, minutes, seconds - hours * 3600.0 - minutes * 60.0);
}

// types the keys into the dsky (V35E, N for noun, E enter, P pro, R rset, C clr, K key rel). a space waits a second,
// W ten seconds, M a minute, S holds pro for 3 seconds (standby and back), ? shows the state there. the crew's side
// in lowercase: i turns the imu off or on, e arms the descent engine or disarms it, o h a set the mode control to
// off, attitude hold or auto, and the hand controller is held over for a second by p q (pitch up, down), y z (yaw
// right, left), r l (roll right, left). {vvnn} waits, up to 20 minutes, for the dsky to show verb vv noun nn
static void fly(flight_t& flight, std::string_view keys)
{
    agc::c_computer& computer = flight.computer;
    lm::c_lunar_module& lm = flight.lm;

    const auto stick = [&](double lm::hand_controller_t::* axis, double deflection) {
        lm.hand_controller.*axis = deflection;
        lm.run(computer, 1.0);
        lm.hand_controller.*axis = 0.0;
    };
    const auto shows = [&](std::string_view verb_noun) {
        const agc::dsky_state_t& state = computer.dsky().state();
        const std::array<std::int8_t, 4> digits{ state.verb[0], state.verb[1], state.noun[0], state.noun[1] };
        for (std::size_t i = 0; i < 4; ++i)
            if (digits[i] != verb_noun[i] - '0')
                return false;
        return true;
    };

    for (std::size_t at = 0; at < keys.size(); ++at)
    {
        const char c = keys[at];
        if (c == '{' && at + 5 < keys.size() && keys[at + 5] == '}')
        {
            const std::string_view verb_noun = keys.substr(at + 1, 4);
            const double give_up = computer.time() + 1200.0;
            while (!shows(verb_noun) && computer.time() < give_up)
                lm.run(computer, 0.1);
            at += 5;
            continue;
        }

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

// boots the program in the lunar module, waits, flies the keys and shows the dsky and the spacecraft at the end. landing,
// it starts in the descent orbit 8 minutes before powered descent
static int run(const std::filesystem::path& main_file, std::string_view keys, double settle, std::string_view watch, bool landing)
{
    const auto assembly = yul::assemble(main_file);
    if (!assembly)
    {
        for (const std::string& error : assembly.error())
            std::println("{}", error);
        return 1;
    }

    // symbols to watch, erasable ones only, NAME+n for a word after one: comma separated, or semicolon separated if
    // any has a comma (L,PVT-CG)
    const char separator = watch.contains(';') ? ';' : ',';
    for (std::size_t start = 0; start < watch.size(); )
    {
        std::size_t end = watch.find(separator, start);
        if (end == std::string_view::npos)
            end = watch.size();
        const std::string name(watch.substr(start, end - start));
        const std::size_t plus = name.rfind('+');
        const bool offset_given = plus != std::string::npos && plus > 0 && plus + 1 < name.size() && std::isdigit(static_cast<unsigned char>(name[plus + 1]));
        const std::string base = offset_given ? name.substr(0, plus) : name;
        const int offset = offset_given ? std::stoi(name.substr(plus + 1)) : 0;
        if (const auto symbol = assembly->symbols.find(base); symbol != assembly->symbols.end() && symbol->second.value + offset < 04000)
            watched.emplace_back(name, symbol->second.value + offset);
        else
            std::println("no erasable symbol {}", name);
        start = end + 1;
    }

    const auto flight = std::make_unique<flight_t>(*assembly);
    if (landing)
    {
        const lm::landing_start_t start = start_landing(*flight, *assembly, 480.0);
        std::println("landing: uplink at {}, ignition planned {}, TLAND {}", get_text(start.time), get_text(start.ignition), get_text(start.tland));
        std::println("  site  {:12.1f} {:12.1f} {:12.1f} m   lm {:12.1f} {:12.1f} {:12.1f} m  {:8.2f} {:8.2f} {:8.2f} m/s",
            start.site.x, start.site.y, start.site.z, start.position.x, start.position.y, start.position.z, start.velocity.x, start.velocity.y, start.velocity.z);
    }
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

    std::println("the pad load");
    {
        const std::vector<std::string> problems = lm::check_pad(*assembly);
        for (const std::string& problem : problems)
            std::println("        {}", problem);
        check(problems.empty(), "every word at its symbol and the value the table gives");
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
        // to the 6 degree stop, then back at 0.2 deg/s for the pad load's times: 23.29 s pitch, 29.26 s roll
        check(near(trim.pitch, -6.0 + 0.2 * 23.29, 0.05) && near(trim.roll, -6.0 + 0.2 * 29.26, 0.05), "trim gimbal where PITTIME and ROLLTIME put it");
        check(no_new_alarms(*flight), "no alarms (1204 before PITTIME/ROLLTIME were pad loaded)");
        fly(*flight, std::format("P {}", v49_to_30_20_0));
        check(at_gimbals(*flight, 30.0, 20.0, 0.0) && !flight->computer.dsky().state().lights.opr_err, "V49 after V48 still maneuvers");
    }
    {
        const auto flight = std::make_unique<flight_t>(*assembly);
        fly(*flight, "MMhpp WW");
        check(flight->lm.imu().gimbals().inner / lm::degrees > 10.0, "attitude hold: the hand controller pitches it");
        check(std::abs(flight->lm.rates().y / lm::degrees) < 0.6, "and it stops when let go");
    }
    {
        const auto flight = std::make_unique<flight_t>(*assembly);
        fly(*flight, "MMV37E06E S");
        check(flight->computer.standby() && flight->computer.dsky().state().lights.stby, "P06 and held pro: standby");
        fly(*flight, "W S MM");
        check(!flight->computer.standby() && shows(*flight, 37, -1), "held pro again: back, asking for a program");
    }

    std::println("Luminary099 landing, from 8 minutes before powered descent");
    {
        const auto flight = std::make_unique<flight_t>(*assembly);
        const lm::landing_start_t start = start_landing(*flight, *assembly, 480.0);
        const auto erasable = [&](std::string_view name, int words, int scale) {
            std::array<std::uint16_t, 3> value{};
            const int address = assembly->symbols.at(std::string(name)).value;
            for (int i = 0; i < words; ++i)
                value[static_cast<std::size_t>(i)] = flight->computer.erasable(address + i);
            return agc::from_scaled(std::span(value.data(), static_cast<std::size_t>(words)), scale);
        };
        const double uplinked_at = flight->computer.time();
        const auto mission_time = [&] { return start.time + flight->computer.time() - uplinked_at; };
        const auto above_site = [&] { return lm::length(flight->lm.position()) - lm::length(start.site); };

        // P63: the ignition algorithm, R60's maneuver, the landing radar's antenna checked, then ignition at TIG - 5
        fly(*flight, "V37E63E{0661}");
        const double tig = erasable("TIG", 2, 28) / 100.0;
        check(std::abs(tig - start.ignition) < 2.0, std::format("P63's ignition algorithm puts TIG where the orbit was planned ({:+.1f} s)", tig - start.ignition));
        fly(*flight, "P{5025}E{5018}P{0618}{5018}eE {9962}P");
        flight->lm.run(flight->computer, tig + 5.0 - mission_time());
        const lm::descent_engine_t& engine = flight->lm.descent_engine();
        check(engine.on && std::abs(engine.thrust - 4670.0) < 100.0, "the descent engine lights at TIG, at 10%");
        flight->lm.run(flight->computer, 30.0);
        check(std::abs(engine.thrust - 43454.7) < 100.0, "at TIG + 26 s it throttles up to the stop (FLATOUT)");

        // the braking phase: the landing radar locks on, the crew permits its updates (V57E), to high gate where P64
        // takes over
        const int modreg = assembly->symbols.at("MODREG").value;
        double worst_trim = 0.0;
        const auto fly_while = [&](const auto& condition, double limit) {
            while (condition() && mission_time() < limit)
            {
                flight->lm.run(flight->computer, 0.5);
                worst_trim = std::max({ worst_trim, std::abs(flight->lm.engine_trim().pitch), std::abs(flight->lm.engine_trim().roll) });
            }
        };
        const lm::landing_radar_t& radar = flight->lm.landing_radar();
        fly_while([&] { return !radar.range_good; }, tig + 600.0);
        check(radar.range_good && radar.velocity_good, std::format("the landing radar locks on at {:.0f} m above the surface", above_site()));
        fly(*flight, "W");
        const agc::lights_t& lights = flight->computer.dsky().state().lights;
        check(!lights.alt && !lights.vel, "ALT and VEL lights out: the computer has its data");
        const double delta_h_before = std::abs(erasable("DELTAH", 2, 24));
        fly(*flight, "V57E MM");
        const double delta_h_after = std::abs(erasable("DELTAH", 2, 24));
        check(delta_h_after < 0.25 * delta_h_before, std::format("V57E: its updates bring delta h from {:.0f} to {:.0f} m", delta_h_before, delta_h_after));

        fly_while([&] { return flight->computer.erasable(modreg) == 63; }, tig + 800.0);
        check(flight->computer.erasable(modreg) == 64, std::format("braking phase flown to P64 in {:.0f} s", mission_time() - tig));
        check(above_site() > 1500.0 && above_site() < 3500.0 && lm::length(flight->lm.velocity()) < 250.0,
            std::format("high gate at {:.0f} m above the site, {:.0f} m/s", above_site(), lm::length(flight->lm.velocity())));
        check(std::abs(erasable("HCALC", 2, 24) - above_site()) < 30.0,
            std::format("the computer's altitude {:.0f} m, the truth {:.0f} m", erasable("HCALC", 2, 24), above_site()));

        // the approach, P64, to where P65 takes over and comes straight down
        fly_while([&] { return flight->computer.erasable(modreg) != 65 && above_site() > 20.0; }, tig + 1000.0);
        check(radar.antenna_position == 2, "the antenna swung to position 2 for the approach");
        check(flight->computer.erasable(modreg) == 65, std::format("P65 at {:.0f} m above the surface, {:.0f} s after ignition", above_site(), mission_time() - tig));
        fly_while([&] { return above_site() > 25.0; }, tig + 1000.0);
        const lm::vec3_t over_surface = flight->lm.velocity() - lm::cross(flight->lm.moon_rotation, flight->lm.position());
        const lm::vec3_t up = flight->lm.position() / lm::length(flight->lm.position());
        const double descent_rate = lm::dot(over_surface, up);
        const double drift = lm::length(over_surface - up * descent_rate);
        check(std::abs(erasable("HCALC", 2, 24) - above_site()) < 3.0,
            std::format("at {:.1f} m the computer's altitude is {:.1f} m", above_site(), erasable("HCALC", 2, 24)));
        check(descent_rate < 0.0 && descent_rate > -1.5 && drift < 0.5, std::format("coming down at {:.2f} m/s, drifting {:.2f} m/s", -descent_rate, drift));
        check(worst_trim / lm::degrees < 1.5, std::format("the trim gimbal holds the thrust through the centre of mass (at most {:.2f} deg)", worst_trim / lm::degrees));
        check(flight->computer.restarts() == 0 && no_new_alarms(*flight), std::format("no restarts or alarms on the way ({} restarts, alarms {:05o} {:05o})",
            flight->computer.restarts(), flight->computer.erasable(failreg->second.value + 1), flight->computer.erasable(failreg->second.value + 2)));
    }

    std::println("{}", failures == 0 ? "all passed" : std::format("{} failed", failures));
    return failures;
}

// every symbol and where it points (a linear pseudo-address, octal: erasable 0-3777, fixed from 4000), by name
static int symbols(const std::filesystem::path& main_file)
{
    const auto assembly = yul::assemble(main_file);
    if (!assembly)
    {
        for (const std::string& error : assembly.error())
            std::println("{}", error);
        return 1;
    }

    std::vector<std::pair<std::string, int>> sorted;
    for (const auto& [name, address] : assembly->symbols)
        sorted.emplace_back(name, address.value);
    std::ranges::sort(sorted);
    for (const auto& [name, value] : sorted)
        std::println("{} {:o}", name, value);
    return 0;
}

static void usage()
{
    std::println("usage:");
    std::println("  agc_check assemble <MAIN.agc> [--compare <rope.binsource>]");
    std::println("  agc_check symbols <MAIN.agc>");
    std::println("  agc_check run <MAIN.agc> [keys] [seconds to settle, default 3] [erasable symbols to show, comma or semicolon separated]");
    std::println("  agc_check land <MAIN.agc> [keys] [settle] [symbols]   as run, from the descent orbit 8 minutes before powered descent");
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

    if (args.size() >= 2 && (args[0] == "run" || args[0] == "land"))
    {
        const std::string_view keys = args.size() >= 3 ? args[2] : std::string_view();
        const double settle = args.size() >= 4 ? std::stod(std::string(args[3])) : 3.0;
        return run(args[1], keys, settle, args.size() >= 5 ? args[4] : std::string_view(), args[0] == "land");
    }

    if (args.size() >= 2 && args[0] == "symbols")
        return symbols(args[1]);

    if (args.size() >= 1 && args[0] == "test")
        return test(args.size() >= 2 ? std::filesystem::path(args[1]) : std::filesystem::path("."));

    usage();
    return 1;
}

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <print>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "agc/computer.hpp"
#include "lm/lunar_module.hpp"
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

// boots the program in the lunar module, waits, types the keys (V35E, N for noun, E enter, P pro, R rset, C clr, K
// key rel) and shows the dsky and the spacecraft at the end. a space waits a second, W ten seconds, M a minute, S
// holds pro for 3 seconds (standby and back), ? shows the state there. the crew's side in lowercase: i turns the imu
// off or on, o h a set the mode control to off, attitude hold or auto, and the hand controller is held over for a
// second by p q (pitch up, down), y z (yaw right, left), r l (roll right, left)
static int run(const std::filesystem::path& main_file, std::string_view keys, double settle, std::string_view watch)
{
    const auto assembly = yul::assemble(main_file);
    if (!assembly)
    {
        for (const std::string& error : assembly.error())
            std::println("{}", error);
        return 1;
    }

    // symbols to watch, comma separated, erasable ones only
    for (std::size_t start = 0; start < watch.size(); )
    {
        std::size_t end = watch.find(',', start);
        if (end == std::string_view::npos)
            end = watch.size();
        const std::string name(watch.substr(start, end - start));
        if (const auto symbol = assembly->symbols.find(name); symbol != assembly->symbols.end() && symbol->second.value < 04000)
            watched.emplace_back(name, symbol->second.value);
        else
            std::println("no erasable symbol {}", name);
        start = end + 1;
    }

    agc::c_computer computer(assembly->rope);
    lm::c_lunar_module lm;
    lm.run(computer, settle);
    print_computer(computer, lm);

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

    if (!keys.empty())
    {
        lm.run(computer, settle);
        print_computer(computer, lm);
    }
    return 0;
}

static void usage()
{
    std::println("usage:");
    std::println("  agc_check assemble <MAIN.agc> [--compare <rope.binsource>]");
    std::println("  agc_check run <MAIN.agc> [keys] [seconds to settle, default 3] [erasable symbols to show, comma separated]");
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

    usage();
    return 1;
}

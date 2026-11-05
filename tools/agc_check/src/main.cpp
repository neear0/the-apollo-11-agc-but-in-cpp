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

// boots the program, waits, types the keys (V35E, N for noun, E enter, P pro, R rset, C clr, K key rel, a space waits
// a second) and shows the dsky after each step
static int run(const std::filesystem::path& main_file, std::string_view keys, double settle)
{
    const auto assembly = yul::assemble(main_file);
    if (!assembly)
    {
        for (const std::string& error : assembly.error())
            std::println("{}", error);
        return 1;
    }

    agc::c_computer computer(assembly->rope);
    computer.run(settle);
    print_dsky(computer.dsky().state(), computer.time());

    for (const char c : keys)
    {
        if (c == ' ')
            computer.run(1.0);
        else if (const std::optional<agc::dsky_key> key = key_for(c))
        {
            computer.dsky().press_key(*key);
            computer.run(0.3);
        }
    }

    if (!keys.empty())
    {
        computer.run(settle);
        print_dsky(computer.dsky().state(), computer.time());
    }
    return 0;
}

static void usage()
{
    std::println("usage:");
    std::println("  agc_check assemble <MAIN.agc> [--compare <rope.binsource>]");
    std::println("  agc_check run <MAIN.agc> [keys] [seconds to settle, default 3]");
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
        return run(args[1], keys, settle);
    }

    usage();
    return 1;
}

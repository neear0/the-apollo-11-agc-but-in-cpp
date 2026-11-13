#include "lm/pad_load.hpp"

#include <cstdint>
#include <string_view>

#include "agc/computer.hpp"
#include "yul/assembler.hpp"

struct pad_word_t
{
    std::string_view symbol;
    std::uint16_t value;
};

static constexpr pad_word_t pad_words[] = {
    // R03 (V48) trims the descent engine: the gimbal driven to its stop for a minute, then back for these times, in
    // centiseconds. zero, the waitlist call for them is alarm 1204 and the abort ends V48. the flown values aren't in
    // the source (they came from Grumman); at the gimbal's 0.2 deg/s, 3000 brings it from the 6 degree stop to centre
    { "PITTIME", 3000 },
    { "ROLLTIME", 3000 },
};

std::vector<std::string> lm::load_pad(agc::c_computer& computer, const yul::assembly_t& assembly)
{
    std::vector<std::string> missing;
    for (const pad_word_t& word : pad_words)
    {
        const auto symbol = assembly.symbols.find(std::string(word.symbol));
        if (symbol == assembly.symbols.end() || symbol->second.value >= 04000)
        {
            missing.emplace_back(word.symbol);
            continue;
        }
        computer.load_erasable(symbol->second.value, word.value);
    }
    return missing;
}

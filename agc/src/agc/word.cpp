#include "agc/word.hpp"

agc::word_t agc::from_int(int value)
{
    if (value < 0)
        return agc::negate(agc::word_t{ static_cast<std::uint16_t>(-value & agc::word_mask) });

    return agc::word_t{ static_cast<std::uint16_t>(value & agc::word_mask) };
}

int agc::to_int(agc::word_t word)
{
    if (agc::is_negative(word))
        return -static_cast<int>(agc::negate(word).value);

    return word.value;
}

agc::word_t agc::negate(agc::word_t word)
{
    return agc::word_t{ static_cast<std::uint16_t>(~word.value & agc::word_mask) };
}

agc::word_t agc::add(agc::word_t lhs, agc::word_t rhs)
{
    std::uint32_t sum = static_cast<std::uint32_t>(lhs.value) + rhs.value;

    // end-around carry
    if (sum > agc::word_mask)
        sum = (sum & agc::word_mask) + 1;

    return agc::word_t{ static_cast<std::uint16_t>(sum & agc::word_mask) };
}

bool agc::is_negative(agc::word_t word)
{
    return (word.value & agc::sign_bit) != 0;
}

bool agc::is_zero(agc::word_t word)
{
    // ones' complement has both +0 and -0
    return word.value == 0 || word.value == agc::word_mask;
}

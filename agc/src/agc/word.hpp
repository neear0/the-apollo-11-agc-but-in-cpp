#pragma once

#include <cstdint>

namespace agc
{
    // a 15-bit ones' complement agc memory word (parity bit not modelled)
    struct word_t
    {
        std::uint16_t value{};

        bool operator==(const word_t&) const = default;
    };

    inline constexpr std::uint16_t word_mask = 0x7fff;
    inline constexpr std::uint16_t sign_bit  = 0x4000;

    word_t from_int(int value);
    int to_int(word_t word);

    word_t negate(word_t word);
    word_t add(word_t lhs, word_t rhs);

    bool is_negative(word_t word);
    bool is_zero(word_t word);
}

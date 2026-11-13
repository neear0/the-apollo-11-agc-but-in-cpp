#pragma once

#include <array>
#include <cstdint>
#include <span>

// numbers as the programs keep them: a fraction of 2^scale, a word for each 14 bits of it, most significant first.
// a word is ones' complement, 14 bits and a sign; the programs leave the words of one number with signs that may
// differ (each is a part of the sum), and write them with the number's sign throughout
namespace agc
{
    // past the largest fraction it saturates, at 37777 37777...
    std::array<std::uint16_t, 3> to_scaled(double value, int scale, int words);
    double from_scaled(std::span<const std::uint16_t> words, int scale);
}

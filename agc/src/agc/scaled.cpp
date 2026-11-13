#include "agc/scaled.hpp"

#include <algorithm>
#include <cmath>

std::array<std::uint16_t, 3> agc::to_scaled(double value, int scale, int words)
{
    const double fraction = value / std::ldexp(1.0, scale);
    const long long full = 1ll << (14 * words);
    const long long magnitude = std::min(std::llround(std::abs(fraction) * static_cast<double>(full)), full - 1);
    const bool negative = std::signbit(fraction) && magnitude != 0;

    std::array<std::uint16_t, 3> result{};
    for (int i = 0; i < words; ++i)
    {
        const auto word = static_cast<std::uint16_t>((magnitude >> (14 * (words - 1 - i))) & 037777);
        result[static_cast<std::size_t>(i)] = negative ? static_cast<std::uint16_t>(~word & 077777) : word;
    }
    return result;
}

double agc::from_scaled(std::span<const std::uint16_t> words, int scale)
{
    double fraction = 0.0;
    double unit = 1.0;
    for (const std::uint16_t word : words)
    {
        unit /= 16384.0;
        const int value = (word & 040000) ? -static_cast<int>(~word & 037777) : (word & 037777);
        fraction += value * unit;
    }
    return std::ldexp(fraction, scale);
}

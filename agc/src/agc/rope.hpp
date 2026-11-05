#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// the core rope: the agc's fixed memory, 36 banks of 1024 15-bit words (banks 00-43 octal)
namespace agc
{
    inline constexpr std::size_t fixed_bank_count = 044;
    inline constexpr std::size_t bank_size        = 02000;

    using rope_bank_t = std::array<std::uint16_t, bank_size>;
    using rope_t      = std::array<rope_bank_t, fixed_bank_count>;
}

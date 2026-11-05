#pragma once

#include <array>
#include <cstdint>

#include "agc/dsky.hpp"
#include "agc/rope.hpp"

// the block ii agc: 2048 words of erasable memory, the core rope, the central registers, the counters and interrupts,
// and the i/o channels the dsky is wired to. instructions behave as the Virtual AGC project's yaAGC has them, which
// runs the flight software as the real computer did
namespace agc
{
    // a memory cycle time (mct) is 12 periods of the 1.024 mhz clock, about 11.7 microseconds
    inline constexpr double mcts_per_second = 1024000.0 / 12.0;

    class c_computer
    {
    public:
        explicit c_computer(const rope_t& rope);

        void power_on();          // erasable memory cleared and a GOJAM: the computer starts at 4000, as when switched on
        void run(double seconds); // as much as the computer does in that much time
        void step();              // one instruction, or one interrupt taken

        std::uint64_t mct() const; // memory cycles since power-on
        double time() const;       // seconds since power-on

        c_dsky& dsky();
        const c_dsky& dsky() const;

        // for debugging
        std::uint16_t erasable(int address) const; // a linear erasable address, 0-3777
        std::uint16_t channel(int number) const;
        int z() const;

    private:
        // a word of memory: a linear erasable address (0-3777) or fixed one (bank * 2000 + offset)
        struct location_t
        {
            bool fixed = false;
            int index = 0;
        };

        int execute();                         // the next instruction or interrupt, how many mcts it took
        void finish(int current_eb, int current_fb, int current_bb, bool keep_extra_code);
        void advance(int mcts);                // time passes: the scaler and the counters it drives
        int scaler_tick();                     // 3200 times a second, how many mcts the counters took
        void service_inputs();                 // keys, the pro button, downrupt timing

        location_t locate(int address12) const;
        int read(location_t location) const;
        void assign(location_t location, int value);
        void assign_erasable(int bank, int offset, int value);
        void counter_overflowed(int address10, int sum); // INCR or AUG of a timer, as if the counter had overflowed

        int get(int reg) const;                // a central register, 16 bits for A, L and Q
        void set(int reg, int value);

        int read_channel(int number) const;
        void write_channel(int number, int value);
        void cpu_write_channel(int number, int value);

        void divide(int divisor);              // DV with a divisor no larger than the dividend, as the hardware does it

        rope_t m_rope;
        std::array<std::array<std::uint16_t, 0400>, 8> m_erasable{};
        std::array<std::uint16_t, 01000> m_channels{};
        std::array<bool, 11> m_interrupt_requests{}; // 1-10: T6RUPT, T5RUPT, T3RUPT, T4RUPT, KEYRUPT1, KEYRUPT2, UPRUPT, DOWNRUPT, RADARUPT, HANDRUPT

        int m_next_z = 0;
        int m_index_value = 0;
        bool m_extra_code = false;             // the instruction after EXTEND
        bool m_substitute_instruction = false; // RESUME: the next instruction is BRUPT rather than memory
        bool m_in_isr = false;
        bool m_allow_interrupt = true;

        std::uint64_t m_mct = 0;
        int m_scaler_phase = 0;                // the scaler ticks 3 times every 80 mcts

        int m_downlink = 0;                    // channels 34 and 35 written: downrupt follows 20 ms later
        bool m_downrupt_pending = false;
        std::uint64_t m_downrupt_at = 0;

        bool m_pro_held = false;
        std::uint64_t m_pro_release_at = 0;

        c_dsky m_dsky;
    };
}

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

    // the counters the spacecraft counts into, by address
    inline constexpr int counter_cdux = 032, counter_cduy = 033, counter_cduz = 034; // imu gimbal angles: outer, inner, middle
    inline constexpr int counter_pipax = 037, counter_pipay = 040, counter_pipaz = 041; // accelerometers, 1 cm/s a count
    inline constexpr int counter_q_rhc = 042, counter_p_rhc = 043, counter_r_rhc = 044; // the hand controller, pitch, yaw, roll

    // what a counter is told: up or down by one, in ones' complement (pinc, minc) or two's complement (pcdu, mcdu)
    enum class counter_pulse : std::uint8_t { pinc, minc, pcdu, mcdu };

    // the restart monitor's alarms in channel 77, each one a GOJAM
    inline constexpr std::uint16_t alarm_tc_trap = 0004, alarm_rupt_lock = 0010, alarm_night_watchman = 0020;

    class c_computer
    {
    public:
        explicit c_computer(const rope_t& rope);

        void power_on();                  // erasable memory cleared and a GOJAM: the computer starts at 4000, as when switched on
        void run(double seconds);         // as much as the computer does in that much time
        void run_until(double seconds);   // up to that long after power-on
        void step();                      // one instruction, or one interrupt taken, or a moment of standby

        std::uint64_t mct() const; // memory cycles since power-on
        double time() const;       // seconds since power-on

        c_dsky& dsky();
        const c_dsky& dsky() const;

        bool standby() const; // switched to standby: PRO held with standby enabled (P06), and held again to come back
        int restarts() const; // GOJAMs since power-on from the restart monitor's alarms

        // the spacecraft's side
        void count(int counter, counter_pulse pulse);              // a pulse into a counter, which takes a memory cycle
        void set_counter(int counter, int value);                  // loaded at once, as the hand controller's read is
        void set_input(int channel, std::uint16_t mask, std::uint16_t bits); // bits of an input channel, 30-33 are active low
        std::array<int, 3> take_gyro_pulses();                     // gyro torquing since the last call, signed, x y z
        std::array<int, 3> take_cdu_drive_pulses();                // imu coarse align drive since the last call, signed, x y z
        int take_thrust_pulses();                                  // descent engine throttle commands since the last call, signed

        // the ground's side: a word of erasable memory loaded through the test connector, as the pad loads were
        void load_erasable(int address, std::uint16_t value);      // a linear erasable address, 0-3777

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
        int alarm_checks(int scaler);          // the restart monitor and the standby circuit, which alarms went off
        int output_counters(int scaler);       // the gyro and cdu drive counters counted out, how many mcts that took
        void gojam(bool restart_light);        // the hardware restart: z to 4000, interrupts and output channels cleared
        void service_inputs();                 // keys, the pro button, downrupt timing

        location_t locate(int address12);      // and the night watchman sees NEWJOB looked at
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

        bool m_pro_tapped = false;             // a key press holds pro down long enough for the software to see it
        std::uint64_t m_pro_release_at = 0;

        // the restart monitor: each watch starts on one scaler phase and goes off on a later one unless what it
        // watches for happened in between
        bool m_night_watchman = false;         // NEWJOB not looked at, 0.64 s
        bool m_night_watchman_tripped = false; // its channel 77 bit stays up for 1.28 s
        bool m_rupt_lock = false;              // stuck in an interrupt, 140 ms
        bool m_no_rupt = false;                // no interrupt, 140 ms
        bool m_tc_trap = false;                // nothing but TC and TCF, 5 ms
        bool m_no_tc = false;                  // no TC at all, 5 ms
        bool m_took_bzf = false;               // the last instruction branched on BZF or BZMF, which looks like a TC
        bool m_took_bzmf = false;
        int m_restarts = 0;

        bool m_standby = false;
        bool m_standby_pressed = false;        // pro held at the last 1.28 s check
        bool m_standby_still_pressed = false;  // and not let go since going to standby

        std::array<int, 3> m_gyro_pulses{};
        std::array<int, 3> m_cdu_drive_pulses{};
        int m_thrust_pulses = 0;

        c_dsky m_dsky;
    };
}

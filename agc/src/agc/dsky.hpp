#pragma once

#include <array>
#include <cstdint>
#include <optional>

// the dsky (display and keyboard): what it shows and the keys pressed on it. no ui here: the computer drives the
// display through output channels 10 and 11 and reads the keys from input channel 15, a view draws the state and
// presses the keys
namespace agc
{
    inline constexpr std::int8_t blank_digit = -1;

    enum class register_sign : std::uint8_t { blank, plus, minus };

    enum class dsky_key : std::uint8_t
    {
        zero, one, two, three, four, five, six, seven, eight, nine,
        verb, noun, plus, minus, clear, proceed, key_release, enter, reset
    };

    // r1, r2 and r3: a sign and five digits
    struct display_register_t
    {
        register_sign sign = agc::register_sign::blank;
        std::array<std::int8_t, 5> digits{ blank_digit, blank_digit, blank_digit, blank_digit, blank_digit };
    };

    // the caution and status lights left of the display, as on the lunar module's dsky
    struct lights_t
    {
        bool uplink_acty = false;
        bool no_att = false;
        bool stby = false;
        bool key_rel = false;
        bool opr_err = false;
        bool prio_disp = false;
        bool no_dap = false;
        bool temp = false;
        bool gimbal_lock = false;
        bool prog = false;
        bool restart = false;
        bool tracker = false;
        bool alt = false;
        bool vel = false;
    };

    // everything the dsky shows, a digit is 0-9 or blank_digit
    struct dsky_state_t
    {
        bool display_off = false;     // the computer is in standby: the electroluminescent display is dark
        bool comp_acty = false;
        bool verb_noun_flash = false; // the verb and noun digits blink, the computer wants a response
        std::array<std::int8_t, 2> prog{ blank_digit, blank_digit };
        std::array<std::int8_t, 2> verb{ blank_digit, blank_digit };
        std::array<std::int8_t, 2> noun{ blank_digit, blank_digit };
        std::array<display_register_t, 3> registers{};
        lights_t lights{};
    };

    // the 5-bit code a key puts on input channel 15. none for pro, which is bit 14 of channel 32 instead
    std::optional<std::uint8_t> keycode(dsky_key key);
    const char* key_name(dsky_key key);

    class c_dsky
    {
    public:
        const dsky_state_t& state() const;
        dsky_state_t& state();

        // from the keypad. the computer takes them one at a time, a key pressed before it took the last one replaces it.
        // pro is a button rather than a key: a press holds it down for a moment, hold_proceed() for as long as wanted
        // (going to standby and back takes holding it for a few seconds)
        void press_key(dsky_key key);
        std::optional<dsky_key> pending_key() const;
        std::optional<dsky_key> take_key();
        void hold_proceed(bool held);
        bool proceed_held() const;

        // from the computer
        void write_channel10(std::uint16_t value); // a relay word: two digits (and a sign bit) or the lights in word 12
        void write_channel11(std::uint16_t value); // comp acty, uplink acty, temp, key rel, verb/noun flash, opr err
        void set_restart_light(bool on);
        void set_standby(bool on);                 // the stby light on and the display off
        void clear();                              // everything off, as at power-on

    private:
        dsky_state_t m_state{};
        std::optional<dsky_key> m_pending_key;
        bool m_proceed_held = false;
    };
}

#include "agc/dsky.hpp"

#include <utility>

std::optional<std::uint8_t> agc::keycode(agc::dsky_key key)
{
    switch (key)
    {
    case agc::dsky_key::zero:        return 020;
    case agc::dsky_key::verb:        return 021;
    case agc::dsky_key::reset:       return 022;
    case agc::dsky_key::key_release: return 031;
    case agc::dsky_key::plus:        return 032;
    case agc::dsky_key::minus:       return 033;
    case agc::dsky_key::enter:       return 034;
    case agc::dsky_key::clear:       return 036;
    case agc::dsky_key::noun:        return 037;
    case agc::dsky_key::proceed:     return std::nullopt;
    default:                         return static_cast<std::uint8_t>(std::to_underlying(key)); // 1-9 are their own value
    }
}

const char* agc::key_name(agc::dsky_key key)
{
    switch (key)
    {
    case agc::dsky_key::verb:        return "VERB";
    case agc::dsky_key::noun:        return "NOUN";
    case agc::dsky_key::plus:        return "+";
    case agc::dsky_key::minus:       return "-";
    case agc::dsky_key::clear:       return "CLR";
    case agc::dsky_key::proceed:     return "PRO";
    case agc::dsky_key::key_release: return "KEY REL";
    case agc::dsky_key::enter:       return "ENTR";
    case agc::dsky_key::reset:       return "RSET";
    default:
    {
        static constexpr const char* digits[] = { "0", "1", "2", "3", "4", "5", "6", "7", "8", "9" };
        return digits[std::to_underlying(key)];
    }
    }
}

const agc::dsky_state_t& agc::c_dsky::state() const
{
    return m_state;
}

agc::dsky_state_t& agc::c_dsky::state()
{
    return m_state;
}

void agc::c_dsky::press_key(agc::dsky_key key)
{
    m_pending_key = key;
}

std::optional<agc::dsky_key> agc::c_dsky::pending_key() const
{
    return m_pending_key;
}

std::optional<agc::dsky_key> agc::c_dsky::take_key()
{
    return std::exchange(m_pending_key, std::nullopt);
}

void agc::c_dsky::hold_proceed(bool held)
{
    m_proceed_held = held;
}

bool agc::c_dsky::proceed_held() const
{
    return m_proceed_held;
}

// the 5-bit relay code of a digit, blank for any other
static std::int8_t relay_digit(int code)
{
    switch (code)
    {
    case 025: return 0;
    case 003: return 1;
    case 031: return 2;
    case 033: return 3;
    case 017: return 4;
    case 036: return 5;
    case 034: return 6;
    case 023: return 7;
    case 035: return 8;
    case 037: return 9;
    default:  return agc::blank_digit;
    }
}

// a relay word is AAAA B CCCCC DDDDD: which pair of digits (AAAA), a sign bit (B), the left (C) and right (D) digits
void agc::c_dsky::write_channel10(std::uint16_t value)
{
    const int word = (value >> 11) & 017;
    const bool b = (value >> 10) & 1;
    const std::int8_t c = relay_digit((value >> 5) & 037);
    const std::int8_t d = relay_digit(value & 037);

    agc::display_register_t& r1 = m_state.registers[0];
    agc::display_register_t& r2 = m_state.registers[1];
    agc::display_register_t& r3 = m_state.registers[2];

    // the sign of a register is two relays, plus and minus, in two different words
    const auto set_sign = [](agc::display_register_t& reg, bool plus_relay, bool on) {
        const bool plus = plus_relay ? on : reg.sign == agc::register_sign::plus;
        const bool minus = plus_relay ? reg.sign == agc::register_sign::minus : on;
        reg.sign = plus ? agc::register_sign::plus : minus ? agc::register_sign::minus : agc::register_sign::blank;
    };

    switch (word)
    {
    case 013: m_state.prog = { c, d }; break;
    case 012: m_state.verb = { c, d }; break;
    case 011: m_state.noun = { c, d }; break;
    case 010: r1.digits[0] = d; break;
    case 007: set_sign(r1, true, b);  r1.digits[1] = c; r1.digits[2] = d; break;
    case 006: set_sign(r1, false, b); r1.digits[3] = c; r1.digits[4] = d; break;
    case 005: set_sign(r2, true, b);  r2.digits[0] = c; r2.digits[1] = d; break;
    case 004: set_sign(r2, false, b); r2.digits[2] = c; r2.digits[3] = d; break;
    case 003: r2.digits[4] = c; r3.digits[0] = d; break;
    case 002: set_sign(r3, true, b);  r3.digits[1] = c; r3.digits[2] = d; break;
    case 001: set_sign(r3, false, b); r3.digits[3] = c; r3.digits[4] = d; break;
    case 014:
        m_state.lights.prio_disp   = value & 0001;
        m_state.lights.no_dap      = value & 0002;
        m_state.lights.vel         = value & 0004;
        m_state.lights.no_att      = value & 0010;
        m_state.lights.alt         = value & 0020;
        m_state.lights.gimbal_lock = value & 0040;
        m_state.lights.tracker     = value & 0200;
        m_state.lights.prog        = value & 0400;
        break;
    default:
        break;
    }
}

void agc::c_dsky::write_channel11(std::uint16_t value)
{
    m_state.comp_acty           = value & 0002;
    m_state.lights.uplink_acty  = value & 0004;
    m_state.lights.temp         = value & 0010;
    m_state.lights.key_rel      = value & 0020;
    m_state.verb_noun_flash     = value & 0040;
    m_state.lights.opr_err      = value & 0100;
}

void agc::c_dsky::set_restart_light(bool on)
{
    m_state.lights.restart = on;
}

void agc::c_dsky::set_standby(bool on)
{
    m_state.lights.stby = on;
    m_state.display_off = on;
}

void agc::c_dsky::clear()
{
    m_state = {};
}

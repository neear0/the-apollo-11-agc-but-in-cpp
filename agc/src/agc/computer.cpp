#include "agc/computer.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

// central registers and the counters, by address
static constexpr int reg_a = 00, reg_l = 01, reg_q = 02, reg_eb = 03, reg_fb = 04, reg_z = 05, reg_bb = 06, reg_zero = 07;
static constexpr int reg_zrupt = 015, reg_brupt = 017;
static constexpr int reg_cyr = 020, reg_sr = 021, reg_cyl = 022, reg_edop = 023;
static constexpr int reg_time2 = 024, reg_time1 = 025, reg_time3 = 026, reg_time4 = 027, reg_time5 = 030, reg_time6 = 031;
static constexpr int reg16 = 3; // A, L and Q hold 16 bits, the overflow with the sign

// the i/o channels the computer itself uses
static constexpr int channel_hiscalar = 03, channel_loscalar = 04, channel_superbank = 07;
static constexpr int channel_dsky = 010, channel_lights = 011, channel_13 = 013, channel_keys = 015, channel_32 = 032, channel_33 = 033;

static constexpr int plus_one = 01, minus_one = 077776, minus_zero = 077777;

// how many more mcts than one an instruction takes, by its top five bits
static constexpr int instruction_timing[32] = {
    0, 0, 0, 0,  1, 0, 0, 0,  2, 1, 1, 1,  1, 1, 1, 1,
    1, 1, 1, 1,  1, 2, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1,
};
static constexpr int extracode_timing[32] = {
    1, 1, 1, 1,  5, 0, 0, 0,  1, 1, 1, 1,  2, 2, 2, 2,
    2, 2, 2, 2,  1, 1, 1, 1,  1, 0, 0, 0,  2, 2, 2, 2,
};

// -----------------------------------------------------------------------------------------------------------------
// ones' complement arithmetic. a 16-bit value has the sign in bit 16 and an overflow when bits 16 and 15 differ

static int add_sp16(int lhs, int rhs)
{
    int sum = lhs + rhs;
    if (sum & 0200000)
        sum = (sum + plus_one) & 0177777;
    return sum;
}

static int overflow_corrected(int value)
{
    value &= 0177777;
    return (value & 037777) | ((value >> 1) & 040000);
}

static int sign_extend(int word)
{
    word &= 0177777;
    return (word & 077777) | ((word << 1) & 0100000);
}

// +1, -1 or +0: the carry a 16-bit value overflowed with
static int value_overflowed(int value)
{
    switch (value & 0140000)
    {
    case 0040000: return plus_one;
    case 0100000: return minus_one;
    default:      return 0;
    }
}

// |x| - 1, or +0 for |x| <= 1, as CCS leaves it in A
static int dabs(int input)
{
    input &= 077777;
    if (input & 040000)
        input = 037777 & ~input;
    return input > 1 ? input - 1 : 0;
}

static int odabs(int input)
{
    input &= 0177777;
    if (input & 0100000)
        input = 0177777 & ~input;
    return input > 1 ? input - 1 : 0;
}

static int negate_sp(int value)
{
    return 077777 & ~value;
}

static int abs_sp(int value)
{
    value &= 077777;
    return (value & 040000) ? 077777 & ~value : value;
}

static int agc_to_int(int value)
{
    return (value & 040000) ? -(037777 & ~value) : (037777 & value);
}

static int int_to_agc(int value)
{
    return value < 0 ? 077777 & ~(-value) : 077777 & value;
}

static int agc_to_int2(int value)
{
    return (value & 02000000000) ? -(01777777777 & ~value) : (01777777777 & value);
}

static int int_to_agc2(int value)
{
    return value < 0 ? 03777777777 & ~(01777777777 & -value) : 01777777777 & value;
}

// a double word (most and least significant halves, maybe of different signs) as one 28-bit ones' complement value
static int sp_to_decent(int msb, int lsb)
{
    msb &= 077777;
    lsb &= 077777;
    if (msb == 0 || msb == minus_zero)
    {
        int value = sign_extend(lsb);
        if (value & 0100000)
            value |= ~0177777;
        return 07777777777 & value;
    }

    if ((040000 & lsb) != (040000 & msb))
    {
        if (lsb == 0 || lsb == minus_zero)
            lsb = (040000 & msb) ? minus_zero : 0;
        else
        {
            const bool complement = (040000 & msb) != 0;
            if (complement)
            {
                msb = 077777 & ~msb;
                lsb = 077777 & ~lsb;
            }
            --msb;
            lsb = (lsb + 040000 + plus_one) & 077777;
            if (complement)
            {
                msb = 077777 & ~msb;
                lsb = 077777 & ~lsb;
            }
        }
    }

    int value = (03777740000 & (msb << 14)) | (037777 & lsb);
    if (02000000000 & value)
        value |= 04000000000;
    return value;
}

static void decent_to_sp(int decent, int& msb, int& lsb)
{
    lsb = 037777 & decent;
    if (decent & 04000000000)
        lsb |= 040000;
    msb = overflow_corrected(0177777 & (decent >> 14));
}

// counters: increment with the overflow they report, and the T6 count toward zero
static bool counter_pinc(std::uint16_t& counter)
{
    int value = counter;
    if (value == 037777)
    {
        counter = 0;
        return true;
    }

    value = (value + 1) & 077777;
    if (value == 0)
        ++value; // -0 to +1
    counter = static_cast<std::uint16_t>(value);
    return false;
}

static bool counter_minc(std::uint16_t& counter)
{
    int value = counter;
    if (value == 040000)
    {
        counter = minus_zero;
        return true;
    }

    value = (value - 1) & 077777;
    if (value == minus_zero)
        --value; // +0 to -1
    counter = static_cast<std::uint16_t>(value);
    return false;
}

static bool counter_dinc(std::uint16_t& counter)
{
    const int value = counter;
    if (value == 0 || value == minus_zero)
        return true;

    counter = static_cast<std::uint16_t>(add_sp16(sign_extend(value), sign_extend((value & 040000) ? plus_one : minus_one)) & 077777);
    return false;
}

// -----------------------------------------------------------------------------------------------------------------

agc::c_computer::c_computer(const agc::rope_t& rope)
    : m_rope(rope)
{
    power_on();
}

void agc::c_computer::power_on()
{
    for (auto& bank : m_erasable)
        bank.fill(0);
    m_channels.fill(0);

    // the input discretes are active low: all ones is nothing asserted (but the stable member's temperature, in limits)
    m_channels[030] = 037777;
    m_channels[031] = 077777;
    m_channels[032] = 077777;
    m_channels[033] = 077777;

    m_erasable[0][reg_z] = 04000;
    m_interrupt_requests.fill(false);
    m_next_z = 04000;
    m_index_value = 0;
    m_extra_code = false;
    m_substitute_instruction = false;
    m_in_isr = false;
    m_allow_interrupt = true;

    m_mct = 0;
    m_scaler_phase = 0;
    m_downlink = 0;
    m_downrupt_pending = false;
    m_pro_tapped = false;

    m_night_watchman = m_night_watchman_tripped = false;
    m_rupt_lock = m_no_rupt = false;
    m_tc_trap = m_no_tc = false;
    m_took_bzf = m_took_bzmf = false;
    m_restarts = 0;
    m_standby = m_standby_pressed = m_standby_still_pressed = false;
    m_gyro_pulses = {};
    m_cdu_drive_pulses = {};

    m_dsky.clear();
    m_dsky.set_restart_light(true); // a GOJAM lights it, RSET puts it out
}

void agc::c_computer::run(double seconds)
{
    run_until(time() + seconds);
}

void agc::c_computer::run_until(double seconds)
{
    const std::uint64_t until = static_cast<std::uint64_t>(std::llround(std::max(seconds, 0.0) * agc::mcts_per_second));
    while (m_mct < until)
        step();
}

void agc::c_computer::step()
{
    service_inputs();

    // in standby only the scaler runs, which is what brings the computer back
    if (m_standby)
        advance(1);
    else
        advance(execute());
}

std::uint64_t agc::c_computer::mct() const
{
    return m_mct;
}

double agc::c_computer::time() const
{
    return static_cast<double>(m_mct) / agc::mcts_per_second;
}

agc::c_dsky& agc::c_computer::dsky()
{
    return m_dsky;
}

const agc::c_dsky& agc::c_computer::dsky() const
{
    return m_dsky;
}

std::uint16_t agc::c_computer::erasable(int address) const
{
    return m_erasable[static_cast<std::size_t>((address >> 8) & 7)][static_cast<std::size_t>(address & 0377)];
}

void agc::c_computer::load_erasable(int address, std::uint16_t value)
{
    m_erasable[static_cast<std::size_t>((address >> 8) & 7)][static_cast<std::size_t>(address & 0377)] = value & 077777;
}

std::uint16_t agc::c_computer::channel(int number) const
{
    return m_channels[static_cast<std::size_t>(number & 0777)];
}

int agc::c_computer::z() const
{
    return m_erasable[0][reg_z];
}

bool agc::c_computer::standby() const
{
    return m_standby;
}

int agc::c_computer::restarts() const
{
    return m_restarts;
}

void agc::c_computer::count(int counter, agc::counter_pulse pulse)
{
    std::uint16_t& value = m_erasable[0][static_cast<std::size_t>(counter & 0377)];
    switch (pulse)
    {
    case agc::counter_pulse::pinc: counter_pinc(value); break;
    case agc::counter_pulse::minc: counter_minc(value); break;
    case agc::counter_pulse::pcdu: value = static_cast<std::uint16_t>((value + 1) & 077777); break;
    case agc::counter_pulse::mcdu: value = static_cast<std::uint16_t>((value - 1) & 077777); break;
    }
    advance(1);
}

void agc::c_computer::set_counter(int counter, int value)
{
    m_erasable[0][static_cast<std::size_t>(counter & 0377)] = static_cast<std::uint16_t>(value & 077777);
}

void agc::c_computer::set_input(int channel, std::uint16_t mask, std::uint16_t bits)
{
    std::uint16_t& value = m_channels[static_cast<std::size_t>(channel & 0777)];
    value = static_cast<std::uint16_t>((value & ~mask) | (bits & mask));
}

std::array<int, 3> agc::c_computer::take_gyro_pulses()
{
    return std::exchange(m_gyro_pulses, {});
}

std::array<int, 3> agc::c_computer::take_cdu_drive_pulses()
{
    return std::exchange(m_cdu_drive_pulses, {});
}

int agc::c_computer::take_thrust_pulses()
{
    return std::exchange(m_thrust_pulses, 0);
}

void agc::c_computer::service_inputs()
{
    // a key on the keypad: its code into channel 15 and KEYRUPT1. pro is a button held down, channel 32 bit 14 low
    if (const std::optional<agc::dsky_key> key = m_dsky.take_key())
    {
        if (const std::optional<std::uint8_t> code = agc::keycode(*key))
        {
            write_channel(channel_keys, *code);
            m_interrupt_requests[5] = true;
        }
        else
        {
            m_pro_tapped = true;
            m_pro_release_at = m_mct + static_cast<std::uint64_t>(0.25 * agc::mcts_per_second); // the software looks every 120 ms
        }
    }

    if (m_pro_tapped && m_mct >= m_pro_release_at)
        m_pro_tapped = false;

    const bool pro = m_pro_tapped || m_dsky.proceed_held();
    if (pro)
        m_channels[channel_32] &= static_cast<std::uint16_t>(~020000);
    else
    {
        m_channels[channel_32] |= 020000;
        m_standby_pressed = false; // the standby circuit only counts pro held the whole time
        m_standby_still_pressed = false;
    }

    if (m_downrupt_pending && m_mct >= m_downrupt_at)
    {
        m_interrupt_requests[8] = true;
        m_downrupt_pending = false;
    }
}

void agc::c_computer::advance(int mcts)
{
    m_mct += static_cast<std::uint64_t>(mcts);
    m_scaler_phase += mcts * 3;
    while (m_scaler_phase >= 80)
    {
        m_scaler_phase -= 80;
        const int counter_mcts = scaler_tick();
        m_mct += static_cast<std::uint64_t>(counter_mcts);
        m_scaler_phase += counter_mcts * 3;
    }
}

int agc::c_computer::scaler_tick()
{
    int scaler = m_channels[channel_loscalar] + 1;
    if (scaler == 040000)
    {
        scaler = 0;
        m_channels[channel_hiscalar] = static_cast<std::uint16_t>((m_channels[channel_hiscalar] + 1) & 037777);
    }
    m_channels[channel_loscalar] = static_cast<std::uint16_t>(scaler);

    const bool was_standby = m_standby;
    const int alarms = alarm_checks(scaler);
    if (m_standby || was_standby)
    {
        // going to standby is a GOJAM too, without the light. coming back the computer starts at 4000 as it left off
        if (m_standby && !was_standby)
            gojam(false);
        return 0;
    }

    // TIME1-TIME5 count every 10 ms, at different phases of the scaler. TIME6 counts down at 1600 hz while enabled
    int mcts = 0;
    std::array<std::uint16_t, 0400>& registers = m_erasable[0];
    switch (scaler & 037)
    {
    case 020:
        mcts += 2;
        if (counter_pinc(registers[reg_time1]))
        {
            ++mcts;
            counter_pinc(registers[reg_time2]);
        }
        if (counter_pinc(registers[reg_time3]))
            m_interrupt_requests[3] = true;
        break;
    case 000:
        ++mcts;
        if (counter_pinc(registers[reg_time5]))
            m_interrupt_requests[2] = true;
        break;
    case 010:
        ++mcts;
        if (counter_pinc(registers[reg_time4]))
            m_interrupt_requests[4] = true;
        break;
    default:
        break;
    }

    if ((m_channels[channel_13] & 040000) && (scaler & 1))
    {
        ++mcts;
        if (counter_dinc(registers[reg_time6]))
        {
            m_interrupt_requests[1] = true;
            cpu_write_channel(channel_13, m_channels[channel_13] & 037777);
        }
    }

    mcts += output_counters(scaler);

    if (alarms)
    {
        m_channels[077] |= static_cast<std::uint16_t>(alarms);
        gojam(true);
        mcts += 2; // GOJAM and the TC 4000 after it
    }
    return mcts;
}

// the night watchman and the standby circuit look every 1.28 s, half a period apart. the rupt lock looks every 160
// ms, the tc trap every 10 ms
int agc::c_computer::alarm_checks(int scaler)
{
    int alarms = 0;
    if ((scaler & 07777) == 04000)
    {
        if (!m_standby)
            m_night_watchman = true;

        if (m_standby_pressed && ((m_channels[channel_13] & 002000) || m_standby))
        {
            if (!m_standby)
            {
                m_standby = true;
                m_standby_still_pressed = true; // pro has to be let go before holding it again brings the computer back
                m_dsky.set_standby(true);
            }
            else if (!m_standby_still_pressed)
            {
                m_standby = false;
                m_dsky.set_standby(false);
            }
        }
    }
    else if ((scaler & 07777) == 0)
    {
        if (!(m_channels[channel_32] & 020000))
            m_standby_pressed = true;

        if (!m_standby && m_night_watchman)
        {
            alarms |= agc::alarm_night_watchman;
            m_night_watchman_tripped = true;
        }
        else
            m_night_watchman_tripped = false;
    }

    if (m_standby)
        return 0;

    if ((scaler & 0777) == 0400)
        m_rupt_lock = m_no_rupt = true;
    else if ((m_rupt_lock || m_no_rupt) && (scaler & 0777) == 0300)
        alarms |= agc::alarm_rupt_lock;

    if ((scaler & 037) == 020)
        m_tc_trap = m_no_tc = true;
    else if ((m_tc_trap || m_no_tc) && (scaler & 037) == 0)
        alarms |= agc::alarm_tc_trap;

    return alarms;
}

// the counters the computer counts out of: GYROCMD to the gyro channel 14 selects at 3200 pulses a second,
// CDUXCMD-CDUZCMD to the imu's coarse align at 800 a second, and THRUST to the descent engine's throttle at 3200 a
// second (32 a centisecond, FRATE in THROTTLE_CONTROL_ROUTINES), while channel 14 says so. each pulse takes a cycle
int agc::c_computer::output_counters(int scaler)
{
    static constexpr int gyrocmd = 047, cduxcmd = 050, thrust = 055;
    static constexpr int channel_14 = 014;
    int mcts = 0;

    std::array<std::uint16_t, 0400>& registers = m_erasable[0];
    const int channel14 = m_channels[channel_14];
    if (channel14 & 000010)
    {
        std::uint16_t& counter = registers[thrust];
        const bool negative = (counter & 040000) != 0;
        if (!counter_dinc(counter))
            m_thrust_pulses += negative ? -1 : 1;
        ++mcts;
    }
    if (channel14 & 001000)
    {
        std::uint16_t& counter = registers[gyrocmd];
        const bool negative_count = (counter & 040000) != 0;
        if (!counter_dinc(counter))
        {
            // all positive in practice: the sign of the torque is bit 9, the gyro bits 7 and 8 (x 1, y 2, z 3)
            const int axis = (channel14 >> 6) & 3;
            const int sign = ((channel14 & 000400) != 0) != negative_count ? -1 : 1;
            if (axis != 0)
                m_gyro_pulses[static_cast<std::size_t>(axis - 1)] += sign;
        }
        ++mcts;
    }

    if ((scaler & 3) == 0)
    {
        for (int axis = 0; axis < 3; ++axis)
        {
            if (!(channel14 & (040000 >> axis)))
                continue;

            std::uint16_t& counter = registers[static_cast<std::size_t>(cduxcmd + axis)];
            const bool negative = (counter & 040000) != 0;
            if (!counter_dinc(counter))
                m_cdu_drive_pulses[static_cast<std::size_t>(axis)] += negative ? -1 : 1;
            ++mcts;
        }
    }
    return mcts;
}

void agc::c_computer::gojam(bool restart_light)
{
    set(reg_q, get(reg_z));
    m_next_z = 04000;
    set(reg_z, 04000);
    m_in_isr = false;
    m_allow_interrupt = true;
    m_interrupt_requests.fill(false);

    for (const int number : { 005, 006, 010, 011, 012, 013, 014 })
        cpu_write_channel(number, 0);
    m_channels[channel_33] |= 002000; // uplink too fast
    cpu_write_channel(034, 0);
    cpu_write_channel(035, 0);
    m_downrupt_pending = false;
    m_downlink = 0;

    m_index_value = 0;
    m_extra_code = false;
    m_substitute_instruction = false;
    m_took_bzf = m_took_bzmf = false;

    if (restart_light)
    {
        m_dsky.set_restart_light(true);
        ++m_restarts;
    }
}

// -----------------------------------------------------------------------------------------------------------------
// memory

agc::c_computer::location_t agc::c_computer::locate(int address12)
{
    static constexpr int newjob = 067; // the executive looks at it in its idle loop, the night watchman watches for that

    address12 &= 07777;
    if (address12 == newjob)
        m_night_watchman = false;
    if (address12 < 01400)
        return { false, address12 };
    if (address12 < 02000)
        return { false, ((m_erasable[0][reg_eb] >> 8) & 7) * 0400 + (address12 & 0377) };

    int bank = 0;
    if (address12 < 04000)
    {
        bank = (m_erasable[0][reg_fb] >> 10) & 037;
        if ((bank & 030) == 030 && (m_channels[channel_superbank] & 0100))
            bank += 010;
    }
    else
        bank = address12 < 06000 ? 2 : 3;

    return { true, bank * 02000 + (address12 & 01777) };
}

int agc::c_computer::read(location_t location) const
{
    if (location.index < 0)
        return 0;
    if (!location.fixed)
        return m_erasable[static_cast<std::size_t>(location.index / 0400)][static_cast<std::size_t>(location.index % 0400)];

    const std::size_t bank = static_cast<std::size_t>(location.index / 02000);
    return bank < agc::fixed_bank_count ? m_rope[bank][static_cast<std::size_t>(location.index % 02000)] : 0;
}

void agc::c_computer::assign(location_t location, int value)
{
    if (!location.fixed && location.index >= 0 && location.index < 04000)
        assign_erasable(location.index / 0400, location.index % 0400, value);
}

// a write into erasable memory. Z jumps, the editing registers shift or rotate what's written into them
void agc::c_computer::assign_erasable(int bank, int offset, int value)
{
    if (bank == 0)
    {
        switch (offset)
        {
        case reg_z:
            m_next_z = value;
            break;
        case reg_cyr:
            value &= 077777;
            value = (value & 1) ? (value >> 1) | 040000 : value >> 1;
            break;
        case reg_sr:
            value &= 077777;
            value = (value & 040000) ? (value >> 1) | 040000 : value >> 1;
            break;
        case reg_cyl:
            value &= 077777;
            value = (value & 040000) ? (value << 1) + 1 : value << 1;
            break;
        case reg_edop:
            value = ((value & 077777) >> 7) & 0177;
            break;
        case reg_zero:
            value = 0;
            break;
        default:
            break;
        }
    }

    const int mask = (bank == 0 && offset < reg16) ? 0177777 : 077777;
    m_erasable[static_cast<std::size_t>(bank)][static_cast<std::size_t>(offset)] = static_cast<std::uint16_t>(value & mask);
}

void agc::c_computer::counter_overflowed(int address10, int sum)
{
    if (value_overflowed(sum) == 0)
        return;

    if (address10 == reg_time1)
        counter_pinc(m_erasable[0][reg_time2]);
    else if (address10 == reg_time5)
        m_interrupt_requests[2] = true;
    else if (address10 == reg_time3)
        m_interrupt_requests[3] = true;
    else if (address10 == reg_time4)
        m_interrupt_requests[4] = true;
}

int agc::c_computer::get(int reg) const
{
    return m_erasable[0][static_cast<std::size_t>(reg)];
}

void agc::c_computer::set(int reg, int value)
{
    m_erasable[0][static_cast<std::size_t>(reg)] = static_cast<std::uint16_t>(value & 0177777);
}

// -----------------------------------------------------------------------------------------------------------------
// i/o channels

int agc::c_computer::read_channel(int number) const
{
    number &= 0777;
    if (number == reg_l || number == reg_q)
        return get(number);
    return m_channels[static_cast<std::size_t>(number)];
}

void agc::c_computer::write_channel(int number, int value)
{
    value &= 077777;
    number &= 0777;
    if (number == reg_l || number == reg_q)
        set(number, value);

    if ((number == channel_keys || number == 016) && value == 022)
        m_dsky.set_restart_light(false); // RSET
    else if (number == channel_33)
        value = (m_channels[channel_33] & 076000) | (value & 001777);

    m_channels[static_cast<std::size_t>(number)] = static_cast<std::uint16_t>(value);
}

void agc::c_computer::cpu_write_channel(int number, int value)
{
    if (number == channel_13)
        value &= 043777;
    if (number == channel_33)
    {
        // writing it resets the alarm flip-flops in its top bits
        m_channels[channel_33] |= 076000;
        value = m_channels[channel_33];
    }
    else if (number == 077)
        value = m_night_watchman_tripped ? agc::alarm_night_watchman : 0; // writing resets the restart monitor's alarm bits
    else if (number == channel_lights && (value & 01000))
        m_dsky.set_restart_light(false);

    write_channel(number, value);

    if (number == channel_dsky)
        m_dsky.write_channel10(static_cast<std::uint16_t>(value & 077777));
    else if (number == channel_lights)
        m_dsky.write_channel11(static_cast<std::uint16_t>(value & 077777));
    else if (number == 034 || number == 035)
    {
        m_downlink |= number == 034 ? 1 : 2;
        if (m_downlink == 3)
        {
            m_downrupt_pending = true;
            m_downrupt_at = m_mct + static_cast<std::uint64_t>(agc::mcts_per_second / 50);
            m_downlink = 0;
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------
// instructions

int agc::c_computer::execute()
{
    const int current_eb = get(reg_eb);
    const int current_fb = get(reg_fb);
    const int current_bb = get(reg_bb);
    int accumulator = get(reg_a);
    const bool overflow = value_overflowed(accumulator) != 0;
    const int program_counter = get(reg_z) & 07777;

    // the instruction, with INDEX's value added to it
    int instruction = 0;
    if (m_substitute_instruction)
        instruction = get(reg_brupt);
    else
        instruction = overflow_corrected(add_sp16(sign_extend(m_index_value), sign_extend(read(locate(program_counter)))));
    instruction &= 077777;

    int extended_opcode = instruction >> 9;
    if (m_extra_code)
        extended_opcode |= 0100;
    const int address12 = instruction & 07777;
    const int address10 = instruction & 01777;
    const int address9 = instruction & 0777;

    // an interrupt goes in before the instruction: not inside one, after INHINT, EXTEND or with A overflowed
    if ((!m_in_isr && m_allow_interrupt && !m_extra_code && !overflow && instruction != 3 && instruction != 4 && instruction != 6)
        || extended_opcode == 0107)
    {
        bool requested = false;
        for (std::size_t i = 1; i < m_interrupt_requests.size(); ++i)
        {
            if (m_interrupt_requests[i])
            {
                m_interrupt_requests[i] = false;
                m_next_z = 04000 + 4 * static_cast<int>(i);
                requested = true;
                break;
            }
        }
        if (!requested && extended_opcode == 0107) // EDRUPT
        {
            m_next_z = 0;
            requested = true;
        }

        if (requested)
        {
            set(reg_zrupt, program_counter + 1);
            set(reg_brupt, instruction);
            m_extra_code = false;
            m_index_value = 0;
            m_substitute_instruction = false;
            m_in_isr = true;
            finish(current_eb, current_fb, current_bb, false);
            return 2;
        }
    }

    const int mcts = 1 + (m_extra_code ? extracode_timing : instruction_timing)[instruction >> 10];
    m_index_value = 0;
    m_substitute_instruction = false;
    m_next_z = 1 + get(reg_z);
    set(reg_z, m_next_z);
    bool keep_extra_code = false;

    // for the tc trap: a TC or TCF, or something the hardware takes for one (a branch taken by BZF or BZMF and what
    // follows it, CS)
    bool executed_tc = false;
    bool tc_transient = m_took_bzf && !(extended_opcode == 000 && address12 == 6);
    bool took_bzf = false, took_bzmf = false;

    const auto resume = [&] {
        m_next_z = get(reg_zrupt) - 1;
        m_in_isr = false;
        m_substitute_instruction = true;
    };

    switch (extended_opcode)
    {
    case 000: case 001: case 002: case 003: case 004: case 005: case 006: case 007: // TC
        if (address12 == 3 || address12 == 4)
        {
            m_allow_interrupt = address12 == 3; // RELINT, INHINT
            tc_transient = tc_transient || m_took_bzf || m_took_bzmf;
        }
        else if (address12 == 6)
        {
            m_extra_code = true; // EXTEND
            keep_extra_code = true;
        }
        else
        {
            if (address12 != reg_q)
                set(reg_q, m_next_z);
            m_next_z = address12;
            executed_tc = true;
        }
        break;

    case 010: case 011: // CCS
    {
        int operand = 0, value_k = 0;
        if (address10 < reg16)
        {
            value_k = get(address10);
            operand = overflow_corrected(value_k);
            set(reg_a, odabs(value_k));
        }
        else
        {
            const location_t where = locate(address10);
            operand = read(where) & 077777;
            set(reg_a, dabs(operand));
            assign(where, operand);
        }

        if (address10 < reg16 && value_overflowed(value_k) == plus_one)
            m_next_z += 0;
        else if (address10 < reg16 && value_overflowed(value_k) == minus_one)
            m_next_z += 2;
        else if (operand == 0)
            m_next_z += 1;
        else if (operand == minus_zero)
            m_next_z += 3;
        else if (operand & 040000)
            m_next_z += 2;
        break;
    }

    case 012: case 013: case 014: case 015: case 016: case 017: // TCF
        m_next_z = address12;
        executed_tc = true;
        break;

    case 020: case 021: // DAS, the operand is K + 1
    {
        if (address10 == reg_l) // DDOUBL
        {
            int lsw = add_sp16(get(reg_l), get(reg_l));
            int msw = add_sp16(accumulator, accumulator);
            if ((0140000 & lsw) == 0040000)
                msw = add_sp16(msw, plus_one);
            else if ((0140000 & lsw) == 0100000)
                msw = add_sp16(msw, sign_extend(minus_one));
            lsw = overflow_corrected(lsw);
            set(reg_a, msw);
            set(reg_l, sign_extend(lsw));
            break;
        }

        const location_t where = locate(address10);
        const location_t where_high = { where.fixed, where.index - 1 };
        int lsw = address10 < reg16 ? add_sp16(get(reg_l), get(address10)) : add_sp16(get(reg_l), sign_extend(read(where)));
        int msw = address10 < reg16 + 1 ? add_sp16(accumulator, get(address10 - 1)) : add_sp16(accumulator, sign_extend(read(where_high)));
        if ((0140000 & lsw) == 0040000)
            msw = add_sp16(msw, plus_one);
        else if ((0140000 & lsw) == 0100000)
            msw = add_sp16(msw, sign_extend(minus_one));
        lsw = overflow_corrected(lsw);

        if ((0140000 & msw) == 0100000)
            set(reg_a, sign_extend(minus_one));
        else if ((0140000 & msw) == 0040000)
            set(reg_a, plus_one);
        else
            set(reg_a, 0);
        set(reg_l, 0);

        if (address10 < reg16)
            set(address10, sign_extend(lsw));
        else
            assign(where, lsw);
        if (address10 < reg16 + 1)
            set(address10 - 1, msw);
        else
            assign(where_high, overflow_corrected(msw));
        break;
    }

    case 022: case 023: // LXCH
        if (address10 == reg_l)
            break;
        if (address10 == reg_zero) // ZL
            set(reg_l, 0);
        else if (address10 < reg16)
        {
            const int operand = get(reg_l);
            set(reg_l, get(address10));
            set(address10, operand);
        }
        else
        {
            const location_t where = locate(address10);
            const int operand = read(where);
            assign(where, overflow_corrected(get(reg_l)));
            set(reg_l, sign_extend(operand));
        }
        break;

    case 024: case 025: // INCR
        if (address10 < reg16)
            set(address10, add_sp16(plus_one, get(address10)));
        else
        {
            const location_t where = locate(address10);
            const int sum = add_sp16(plus_one, sign_extend(read(where)));
            assign(where, overflow_corrected(sum));
            counter_overflowed(address10, sum);
        }
        break;

    case 026: case 027: // ADS
    {
        const location_t where = locate(address10);
        if (address10 == reg_a)
            accumulator = add_sp16(accumulator, accumulator);
        else if (address10 < reg16)
            accumulator = add_sp16(accumulator, get(address10));
        else
            accumulator = add_sp16(accumulator, sign_extend(read(where)));
        set(reg_a, accumulator);

        if (address10 == reg_a)
            ;
        else if (address10 < reg16)
            set(address10, accumulator);
        else
            assign(where, overflow_corrected(accumulator));
        break;
    }

    case 030: case 031: case 032: case 033: case 034: case 035: case 036: case 037: // CA
    {
        if (address12 == reg_a) // NOOP
            break;
        if (address12 < reg16)
        {
            set(reg_a, get(address12));
            break;
        }
        const location_t where = locate(address12);
        const int word = read(where);
        set(reg_a, sign_extend(word));
        assign(where, word);
        break;
    }

    case 040: case 041: case 042: case 043: case 044: case 045: case 046: case 047: // CS
    {
        tc_transient = true;
        if (address12 == reg_a) // COM
        {
            set(reg_a, ~accumulator);
            break;
        }
        if (address12 < reg16)
        {
            set(reg_a, ~get(address12));
            break;
        }
        const location_t where = locate(address12);
        const int word = read(where);
        set(reg_a, sign_extend(negate_sp(word)));
        assign(where, word);
        break;
    }

    case 050: case 051: // INDEX
        if (address10 == 017)
            resume(); // RESUME
        else if (address10 < reg16)
            m_index_value = overflow_corrected(get(address10));
        else
            m_index_value = read(locate(address10));
        break;

    case 052: case 053: // DXCH, the operand is K + 1
    {
        if (address10 == reg_l)
        {
            set(reg_l, sign_extend(overflow_corrected(get(reg_l))));
            break;
        }

        const location_t where = locate(address10);
        const location_t where_high = { where.fixed, where.index - 1 };
        if (address10 < reg16)
        {
            const int operand = get(address10);
            set(address10, get(reg_l));
            set(reg_l, operand);
        }
        else
        {
            const int operand = sign_extend(read(where));
            assign(where, overflow_corrected(get(reg_l)));
            set(reg_l, operand);
        }
        set(reg_l, sign_extend(overflow_corrected(get(reg_l))));

        if (address10 < reg16 + 1)
        {
            const int operand = get(address10 - 1);
            set(address10 - 1, get(reg_a));
            set(reg_a, operand);
        }
        else
        {
            const int operand = sign_extend(read(where_high));
            assign(where_high, overflow_corrected(get(reg_a)));
            set(reg_a, operand);
        }
        break;
    }

    case 054: case 055: // TS
        if (address10 == reg_a) // OVSK
        {
            if (overflow)
                m_next_z += plus_one;
        }
        else if (address10 == reg_z) // TCAA
        {
            m_next_z = 077777 & accumulator;
            if (overflow)
                set(reg_a, sign_extend(value_overflowed(accumulator)));
        }
        else
        {
            if (address10 < reg16)
                set(address10, accumulator);
            else
                assign(locate(address10), overflow_corrected(accumulator));
            if (overflow)
            {
                // an overflow stores the corrected value, leaves the carry in A and skips
                set(reg_a, sign_extend(value_overflowed(accumulator)));
                m_next_z += plus_one;
            }
        }
        break;

    case 056: case 057: // XCH
        if (address10 == reg_a)
            break;
        if (address10 < reg16)
        {
            set(reg_a, get(address10));
            set(address10, accumulator);
        }
        else
        {
            const location_t where = locate(address10);
            set(reg_a, sign_extend(read(where)));
            assign(where, overflow_corrected(accumulator));
        }
        break;

    case 060: case 061: case 062: case 063: case 064: case 065: case 066: case 067: // AD
        if (address12 == reg_a) // DOUBLE
            accumulator = add_sp16(accumulator, accumulator);
        else if (address12 < reg16)
            accumulator = add_sp16(accumulator, get(address12));
        else
        {
            const location_t where = locate(address12);
            const int word = read(where);
            accumulator = add_sp16(accumulator, sign_extend(word));
            assign(where, word);
        }
        set(reg_a, accumulator);
        break;

    case 070: case 071: case 072: case 073: case 074: case 075: case 076: case 077: // MASK
        if (address12 < reg16)
            set(reg_a, accumulator & get(address12));
        else
            set(reg_a, sign_extend(overflow_corrected(accumulator) & read(locate(address12))));
        break;

    case 0100: // READ
        if (address9 == reg_l || address9 == reg_q)
            set(reg_a, get(address9));
        else
            set(reg_a, sign_extend(read_channel(address9)));
        break;

    case 0101: // WRITE
        if (address9 == reg_l || address9 == reg_q)
            set(address9, accumulator);
        else
            cpu_write_channel(address9, overflow_corrected(accumulator));
        break;

    case 0102: // RAND
        if (address9 == reg_l || address9 == reg_q)
            set(reg_a, accumulator & get(address9));
        else
            set(reg_a, sign_extend(overflow_corrected(accumulator) & read_channel(address9)));
        break;

    case 0103: // WAND
        if (address9 == reg_l || address9 == reg_q)
        {
            set(address9, accumulator & get(address9));
            set(reg_a, get(address9));
        }
        else
        {
            const int operand = overflow_corrected(accumulator) & read_channel(address9);
            cpu_write_channel(address9, operand);
            set(reg_a, sign_extend(operand));
        }
        break;

    case 0104: // ROR
        if (address9 == reg_l || address9 == reg_q)
            set(reg_a, accumulator | get(address9));
        else
            set(reg_a, sign_extend(overflow_corrected(accumulator) | read_channel(address9)));
        break;

    case 0105: // WOR
        if (address9 == reg_l || address9 == reg_q)
        {
            set(address9, accumulator | get(address9));
            set(reg_a, get(address9));
        }
        else
        {
            const int operand = overflow_corrected(accumulator) | read_channel(address9);
            cpu_write_channel(address9, operand);
            set(reg_a, sign_extend(operand));
        }
        break;

    case 0106: // RXOR
        if (address9 == reg_l || address9 == reg_q)
            set(reg_a, accumulator ^ get(address9));
        else
            set(reg_a, sign_extend(overflow_corrected(accumulator) ^ read_channel(address9)));
        break;

    case 0107: // EDRUPT, taken above
        break;

    case 0110: case 0111: // DV
    {
        // the dividend is A,L made one sign, the divisor K
        int high = overflow_corrected(accumulator);
        int low = get(reg_l);
        int dividend = sp_to_decent(high, low);
        decent_to_sp(dividend, high, low);
        const int abs_a = abs_sp(high);
        const int abs_l = abs_sp(low);

        int divisor = 0;
        if (address10 == reg_a)
        {
            divisor = get(reg_a);
            if ((get(reg_a) & 0100000) == 0)
                divisor = 0177777 & ~divisor;
        }
        else if (address10 == reg_l)
        {
            divisor = get(reg_l);
            if ((abs_a == 0 && (0100000 & get(reg_l))) || (abs_a != 0 && (0100000 & get(reg_a))))
                divisor = 0177777 & ~divisor;
            divisor = sign_extend(overflow_corrected(add_sp16(divisor & 0177777, 040000)));
        }
        else if (address10 == reg_z)
        {
            divisor = get(reg_z);
            if ((abs_a == 0 && (0100000 & get(reg_l))) || (abs_a != 0 && (0100000 & get(reg_a))))
                divisor |= 0100000;
        }
        else if (address10 < reg16)
            divisor = get(address10);
        else
            divisor = sign_extend(read(locate(address10)));
        divisor &= 0177777;

        const int abs_k = abs_sp(overflow_corrected(divisor));
        if (abs_a > abs_k || (abs_a == abs_k && abs_l != 0) || value_overflowed(divisor) != 0)
            divide(divisor);
        else if (abs_a == 0 && abs_l == 0)
        {
            int quotient = 0;
            if ((040000 & get(reg_l)) == (040000 & overflow_corrected(divisor)))
                quotient = abs_k == 0 ? 037777 : 0;
            else
                quotient = abs_k == 0 ? (077777 & ~037777) : minus_zero;
            set(reg_a, sign_extend(quotient));
        }
        else if (abs_a == abs_k && abs_l == 0)
        {
            const int quotient = high == overflow_corrected(divisor) ? 037777 : (077777 & ~037777);
            set(reg_l, sign_extend(high));
            set(reg_a, sign_extend(quotient));
        }
        else
        {
            const int numerator = agc_to_int2(dividend);
            const int denominator = agc_to_int(overflow_corrected(divisor));
            const int quotient = numerator / denominator;
            const int remainder = numerator % denominator;
            set(reg_a, sign_extend(int_to_agc(quotient)));
            if (remainder == 0)
                set(reg_l, numerator >= 0 ? 0 : sign_extend(minus_zero));
            else
                set(reg_l, sign_extend(int_to_agc(remainder)));
        }
        break;
    }

    case 0112: case 0113: case 0114: case 0115: case 0116: case 0117: // BZF
        if (accumulator == 0 || accumulator == 0177777)
        {
            m_next_z = address12;
            took_bzf = true;
        }
        break;

    case 0120: case 0121: // MSU: modular (two's complement) subtraction, for the cdu counters
    {
        const location_t where = locate(address10);
        unsigned lhs = 0, rhs = 0;
        if (address10 < reg16)
        {
            lhs = 0177777u & static_cast<unsigned>(accumulator);
            rhs = 0177777u & ~static_cast<unsigned>(get(address10));
        }
        else
        {
            lhs = 077777u & static_cast<unsigned>(overflow_corrected(accumulator));
            rhs = 077777u & ~static_cast<unsigned>(read(where));
        }

        int difference = static_cast<int>(lhs + rhs + 1);
        if (difference & 040000)
        {
            difference |= 0100000;
            --difference;
        }

        if (address10 == reg_q)
            set(reg_a, 0177777 & difference);
        else
            set(reg_a, sign_extend(077777 & difference));
        if (address10 >= reg_cyr && address10 <= reg_edop)
            assign(where, read(where));
        break;
    }

    case 0122: case 0123: // QXCH
        if (address10 == reg_q)
            break;
        if (address10 == reg_zero) // ZQ
            set(reg_q, 0);
        else if (address10 < reg16)
        {
            const int operand = get(reg_q);
            set(reg_q, get(address10));
            set(address10, operand);
        }
        else
        {
            const location_t where = locate(address10);
            const int operand = overflow_corrected(get(reg_q));
            set(reg_q, sign_extend(read(where)));
            assign(where, operand);
        }
        break;

    case 0124: case 0125: // AUG
    case 0126: case 0127: // DIM
    {
        const bool augment = extended_opcode <= 0125;
        const location_t where = locate(address10);
        const int operand = (address10 < reg16 ? get(address10) : sign_extend(read(where))) & 0177777;
        if (!augment && (operand == 0 || operand == sign_extend(minus_zero)))
            break;

        const bool negative = (operand & 0100000) != 0;
        const int increment = (augment != negative) ? plus_one : sign_extend(minus_one);
        const int sum = add_sp16(0177777 & increment, operand);
        if (address10 < reg16)
            set(address10, sum);
        else
        {
            assign(where, overflow_corrected(sum));
            if (augment)
                counter_overflowed(address10, sum);
        }
        break;
    }

    case 0130: case 0131: case 0132: case 0133: case 0134: case 0135: case 0136: case 0137: // DCA, the operand is K + 1
    case 0140: case 0141: case 0142: case 0143: case 0144: case 0145: case 0146: case 0147: // DCS
    {
        const bool complement = extended_opcode >= 0140;
        const auto maybe_not = [complement](int value) { return complement ? ~value : value; };

        if (address12 == reg_l)
        {
            if (complement) // DCOM
            {
                set(reg_a, ~accumulator);
                set(reg_l, ~get(reg_l));
            }
            set(reg_l, sign_extend(overflow_corrected(get(reg_l))));
            break;
        }

        const location_t where = locate(address12);
        const location_t where_high = { where.fixed, where.index - 1 };
        set(reg_l, maybe_not(address12 < reg16 ? get(address12) : sign_extend(read(where))));
        set(reg_l, sign_extend(overflow_corrected(get(reg_l))));
        set(reg_a, maybe_not(address12 < reg16 + 1 ? get(address12 - 1) : sign_extend(read(where_high))));
        if (address12 >= reg_cyr && address12 <= reg_edop)
            assign(where, read(where));
        if (address12 >= reg_cyr + 1 && address12 <= reg_edop + 1)
            assign(where_high, read(where_high));
        break;
    }

    case 0150: case 0151: case 0152: case 0153: case 0154: case 0155: case 0156: case 0157: // INDEX, extended
        if (address12 == 017 << 1)
            resume();
        else
        {
            m_index_value = address12 < reg16 ? overflow_corrected(get(address12)) : read(locate(address12));
            keep_extra_code = true;
        }
        break;

    case 0160: case 0161: // SU
        if (address10 == reg_a)
            accumulator = sign_extend(minus_zero);
        else if (address10 < reg16)
            accumulator = add_sp16(accumulator, 0177777 & ~get(address10));
        else
        {
            const location_t where = locate(address10);
            const int word = read(where);
            accumulator = add_sp16(accumulator, sign_extend(negate_sp(word)));
            assign(where, word);
        }
        set(reg_a, accumulator);
        break;

    case 0162: case 0163: case 0164: case 0165: case 0166: case 0167: // BZMF
        if (accumulator == 0 || (accumulator & 0100000))
        {
            m_next_z = address12;
            took_bzmf = true;
        }
        break;

    case 0170: case 0171: case 0172: case 0173: case 0174: case 0175: case 0176: case 0177: // MP
    {
        const int multiplicand = overflow_corrected(accumulator);
        const int multiplier = address12 < reg16 ? overflow_corrected(get(address12)) : read(locate(address12));
        int high = 0, low = 0;
        if (multiplier == 0 || multiplier == minus_zero)
            high = low = 0;
        else if (multiplicand == 0 || multiplicand == minus_zero)
        {
            // a zero times a number is a zero with the sign of the product
            const bool negative = (multiplicand == 0 && (040000 & multiplier)) || (multiplicand == minus_zero && !(040000 & multiplier));
            high = low = negative ? minus_zero : 0;
        }
        else
        {
            int product = int_to_agc2(agc_to_int(sign_extend(multiplicand)) * agc_to_int(sign_extend(multiplier)));
            if (02000000000 & product)
                product |= 004000000000;
            decent_to_sp(product, high, low);
        }
        set(reg_a, sign_extend(high));
        set(reg_l, sign_extend(low));
        break;
    }

    default:
        break;
    }

    finish(current_eb, current_fb, current_bb, keep_extra_code);

    // what the restart monitor watches for happened
    if (m_in_isr)
        m_no_rupt = false;
    else
        m_rupt_lock = false;
    if (executed_tc || tc_transient)
        m_no_tc = false;
    if (!executed_tc)
        m_tc_trap = false;
    m_took_bzf = took_bzf;
    m_took_bzmf = took_bzmf;

    return mcts;
}

// after every instruction: Z moves on, the bank registers agree with each other, L holds 15 bits
void agc::c_computer::finish(int current_eb, int current_fb, int current_bb, bool keep_extra_code)
{
    set(reg_zero, 0);
    m_channels[channel_superbank] &= 0160;

    set(reg_z, m_next_z);
    if (!m_substitute_instruction)
        set(reg_z, get(reg_z) & 07777);
    if (!keep_extra_code)
        m_extra_code = false;

    if (current_bb != get(reg_bb))
    {
        set(reg_fb, get(reg_bb) & 076000);
        set(reg_eb, (get(reg_bb) & 07) << 8);
    }
    else if (current_eb != get(reg_eb) || current_fb != get(reg_fb))
        set(reg_bb, (get(reg_fb) & 076000) | ((get(reg_eb) & 03400) >> 8));

    set(reg_eb, get(reg_eb) & 03400);
    set(reg_fb, get(reg_fb) & 076000);
    set(reg_bb, get(reg_bb) & 076007);
    set(reg_l, sign_extend(overflow_corrected(get(reg_l))));
}

// DV when the quotient doesn't fit: the hardware's own sequence of 14 subtractions
void agc::c_computer::divide(int divisor)
{
    unsigned a = static_cast<unsigned>(get(reg_a));
    unsigned l = static_cast<unsigned>(get(reg_l));
    unsigned d = static_cast<unsigned>(divisor) & 0177777u;

    unsigned dividend_sign = a & 0100000u;
    if (!dividend_sign)
        a = ~a & 0177777u;
    if (a == 0177777u)
        dividend_sign = l & 0100000u;
    if (dividend_sign)
        l = ~l & 0177777u;

    l = static_cast<unsigned>(add_sp16(static_cast<int>(l), 040000));
    if (value_overflowed(static_cast<int>(l)) != plus_one)
        a = static_cast<unsigned>(add_sp16(static_cast<int>(a), 1));
    unsigned remainder = a;

    const unsigned divisor_sign = d & 0100000u;
    if (divisor_sign)
        d = ~d & 0177777u;

    const unsigned quotient_sign = l & 0100000u;
    unsigned quotient = (quotient_sign | ((l & 037777u) << 1) | (quotient_sign >> 15)) & 0177777u;
    for (int i = 0; i < 14; ++i)
    {
        quotient = (quotient << 1) & 0177777u;
        const unsigned remainder_sign = remainder & 0100000u;
        remainder = remainder_sign | ((remainder & 037777u) << 1);
        if ((quotient & 0100000u) == 0)
            remainder |= remainder_sign >> 15;

        const unsigned sum = static_cast<unsigned>(add_sp16(static_cast<int>(remainder), static_cast<int>(d)));
        if (sum & 0100000u)
        {
            quotient |= 1;
            remainder = sum;
        }
    }

    a = quotient_sign | (quotient & 077777u);
    set(reg_a, static_cast<int>(dividend_sign != divisor_sign ? ~a : a));
    set(reg_l, static_cast<int>(dividend_sign ? remainder : ~remainder));
}

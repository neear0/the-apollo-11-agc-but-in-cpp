#include "yul/assembler.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <format>
#include <memory>
#include <fstream>
#include <utility>

// the operand constraints of a basic instruction
static constexpr int flag_kplus1   = 0x1;     // assembles to K + 1 (the double-word instructions)
static constexpr int flag_extended = 0x2;     // needs EXTEND before it
static constexpr int flag_pc0      = 0x4;     // peripheral (channel) code n
static constexpr int flag_qc0      = 0x8;     // quarter code n
static constexpr int flag_pc1      = 0x10;
static constexpr int flag_pc2      = 0x20;
static constexpr int flag_qc1      = 0x40;
static constexpr int flag_pc3      = 0x80;
static constexpr int flag_pc4      = 0x100;
static constexpr int flag_qc2      = 0x200;
static constexpr int flag_pc5      = 0x400;
static constexpr int flag_pc6      = 0x800;
static constexpr int flag_qc3      = 0x1000;
static constexpr int flag_pc7      = 0x2000;
static constexpr int flag_qcnot0   = 0x4000;  // any quarter code but 0, a fixed address
static constexpr int flag_fixed    = 0x8000;
static constexpr int flag_erasable = 0x10000;
static constexpr int flag_enumber  = 0x20000; // a number in the operand is an absolute address

static constexpr int any_pc = flag_pc0 | flag_pc1 | flag_pc2 | flag_pc3 | flag_pc4 | flag_pc5 | flag_pc6 | flag_pc7;
static constexpr int any_qc = flag_qc0 | flag_qc1 | flag_qc2 | flag_qc3;

using kind = yul::parser_kind;

static constexpr yul::parser_match_t basic(std::string_view name, int opcode, int flags, int xmask = 0)
{
    return { name, yul::op_type::basic, kind::general, opcode, flags, {}, {}, 0, xmask };
}

static constexpr yul::parser_match_t pseudo(std::string_view name, kind parser, bool pinch_hit = false)
{
    return { name, yul::op_type::pseudo, parser, 0, 0, {}, {}, 0, 0, 0, 0, pinch_hit };
}

static constexpr yul::parser_match_t alias(std::string_view name, std::string_view op, std::string_view operand)
{
    return { name, yul::op_type::basic, kind::none, 0, 0, op, operand };
}

static constexpr yul::parser_match_t ignored(std::string_view name)
{
    return { name, yul::op_type::pseudo, kind::none };
}

static constexpr yul::parser_match_t downlink(std::string_view name, kind parser, int opcode, int flags, int adder, int xmask)
{
    return { name, yul::op_type::downlink, parser, opcode, flags, {}, {}, adder, xmask };
}

static constexpr yul::parser_match_t interpreter(std::string_view name, kind parser, int adder = 0)
{
    return { name, yul::op_type::interpreter, parser, 0, 0, {}, {}, adder };
}

static constexpr yul::parser_match_t parsers[] = {
    downlink("-1DNADR", kind::ecadr, 0, 0, 0, 077777),
    { "-2CADR", yul::op_type::pseudo, kind::two_cadr, 0, 0, {}, {}, 0, 077777, 0, 077777 },
    downlink("-2DNADR", kind::ecadr, 0, 0, 004000, 077777),
    downlink("-3DNADR", kind::ecadr, 0, 0, 010000, 077777),
    downlink("-4DNADR", kind::ecadr, 0, 0, 014000, 077777),
    downlink("-5DNADR", kind::ecadr, 0, 0, 020000, 077777),
    downlink("-6DNADR", kind::ecadr, 0, 0, 024000, 077777),
    basic("-CCS", 010000, flag_qc0 | flag_enumber, 077777),
    downlink("-DNCHAN", kind::general, 034000, flag_pc0 | flag_enumber, 0, 077777),
    downlink("-DNPTR", kind::genadr, 0, 0, 030000, 077777),
    { "-GENADR", yul::op_type::pseudo, kind::genadr, 0, 0, {}, {}, 0, 077777 },
    pseudo("=", kind::equals),
    pseudo("=MINUS", kind::eq_minus),
    downlink("1DNADR", kind::ecadr, 0, 0, 0, 0),
    pseudo("2BCADR", kind::two_cadr),
    pseudo("2CADR", kind::two_cadr),
    pseudo("2DEC", kind::two_dec),
    pseudo("2DEC*", kind::two_dec),
    downlink("2DNADR", kind::ecadr, 0, 0, 004000, 0),
    pseudo("2OCT", kind::two_oct),
    downlink("3DNADR", kind::ecadr, 0, 0, 010000, 0),
    downlink("4DNADR", kind::ecadr, 0, 0, 014000, 0),
    downlink("5DNADR", kind::ecadr, 0, 0, 020000, 0),
    downlink("6DNADR", kind::ecadr, 0, 0, 024000, 0),
    basic("AD", 060000, flag_enumber),
    pseudo("ADRES", kind::genadr),
    basic("ADS", 020000, flag_qc3),
    basic("AUG", 020000, flag_extended | flag_qc2),
    pseudo("BANK", kind::bank),
    pseudo("BBCON", kind::bbcon),
    pseudo("BBCON*", kind::bbcon_star),
    pseudo("BLOCK", kind::block),
    ignored("BNKSUM"),
    basic("BZF", 010000, flag_extended | flag_qcnot0),
    basic("BZMF", 060000, flag_extended | flag_qcnot0),
    basic("CA", 030000, flag_enumber),
    pseudo("CADR", kind::cadr, true),
    basic("CAE", 030000, flag_erasable | flag_enumber),
    basic("CAF", 030000, flag_fixed | flag_enumber),
    basic("CCS", 010000, flag_qc0 | flag_enumber),
    alias("COM", "CS", "A"),
    ignored("COUNT"),
    ignored("COUNT*"),
    basic("CS", 040000, flag_enumber),
    basic("DAS", 020000, flag_qc0 | flag_kplus1),
    basic("DCA", 030000, flag_extended | flag_kplus1 | flag_enumber),
    alias("DCOM", "DCS", "A"),
    basic("DCS", 040000, flag_extended | flag_kplus1 | flag_enumber),
    alias("DDOUBL", "DAS", "A"),
    pseudo("DEC", kind::dec, true),
    pseudo("DEC*", kind::dec),
    basic("DIM", 020000, flag_extended | flag_qc3),
    downlink("DNCHAN", kind::general, 034000, flag_pc0 | flag_enumber, 0, 0),
    downlink("DNPTR", kind::genadr, 0, 0, 030000, 0),
    alias("DOUBLE", "AD", "A"),
    alias("DTCB", "DXCH", "Z"),
    alias("DTCF", "DXCH", "$4"),
    basic("DV", 010000, flag_extended | flag_qc0),
    basic("DXCH", 050000, flag_qc1 | flag_kplus1 | flag_enumber),
    pseudo("EBANK=", kind::ebank_equals),
    pseudo("ECADR", kind::ecadr, true),
    { "EDRUPT", yul::op_type::basic, kind::edrupt },
    pseudo("EQUALS", kind::equals),
    pseudo("ERASE", kind::erase),
    alias("EXTEND", "TC", "$6"),
    pseudo("FCADR", kind::cadr, true),
    pseudo("GENADR", kind::genadr),
    basic("INCR", 020000, flag_qc2),
    { "INDEX", yul::op_type::basic, kind::index },
    alias("INHINT", "TC", "$4"),
    basic("LXCH", 020000, flag_qc1 | flag_enumber),
    basic("MASK", 070000, 0),
    ignored("MEMORY"),
    pseudo("MM", kind::dec),
    basic("MP", 070000, flag_extended),
    basic("MSK", 070000, 0),
    basic("MSU", 020000, flag_extended | flag_qc0 | flag_enumber),
    { "NDX", yul::op_type::basic, kind::index },
    pseudo("NV", kind::vn, true),
    pseudo("OCT", kind::oct, true),
    pseudo("OCTAL", kind::oct, true),
    alias("OVSK", "TS", "A"),
    basic("QXCH", 020000, flag_extended | flag_qc1 | flag_enumber),
    basic("RAND", 0, flag_extended | flag_pc2 | flag_enumber),
    basic("READ", 0, flag_extended | flag_pc0 | flag_enumber),
    alias("RELINT", "TC", "$3"),
    pseudo("REMADR", kind::genadr),
    alias("RESUME", "INDEX", "$17"),
    alias("RETURN", "TC", "Q"),
    basic("ROR", 0, flag_extended | flag_pc4 | flag_enumber),
    basic("RXOR", 0, flag_extended | flag_pc6 | flag_enumber),
    pseudo("SBANK=", kind::sbank_equals),
    pseudo("SETLOC", kind::setloc),
    alias("SQUARE", "MP", "A"),
    interpreter("STCALL", kind::st_call),
    interpreter("STODL", kind::st_odl),
    interpreter("STODL*", kind::st_odl, 04000),
    interpreter("STORE", kind::st_ore),
    interpreter("STOVL", kind::st_ovl),
    interpreter("STOVL*", kind::st_ovl, 04000),
    basic("SU", 060000, flag_extended | flag_qc0 | flag_enumber),
    ignored("SUBRO"),
    basic("TC", 0, flag_enumber),
    alias("TCAA", "TS", "Z"),
    { "TCF", yul::op_type::basic, kind::tcf },
    basic("TCR", 0, flag_enumber),
    basic("TS", 050000, flag_qc2 | flag_enumber),
    pseudo("VN", kind::vn, true),
    basic("WAND", 0, flag_extended | flag_pc3 | flag_enumber),
    basic("WOR", 0, flag_extended | flag_pc5 | flag_enumber),
    basic("WRITE", 0, flag_extended | flag_pc1 | flag_enumber),
    basic("XCH", 050000, flag_qc3 | flag_enumber),
    alias("XLQ", "TC", "L"),
    alias("XXALQ", "TC", "A"),
    alias("ZL", "LXCH", "$7"),
    alias("ZQ", "QXCH", "$7"),
};

// interpretive opcodes: the 7-bit code, how many operands follow, and how they're encoded

static constexpr yul::interpreter_match_t interpreters[] = {
    { "ABS", 0130, 0 },             { "ABVAL", 0130, 0 },           { "ACOS", 0050, 0 },            { "ARCCOS", 0050, 0 },
    { "ARCSIN", 0040, 0 },          { "ASIN", 0040, 0 },            { "AXC,1", 0016, 1 },           { "AXC,2", 0012, 1 },
    { "AXT,1", 0006, 1 },           { "AXT,2", 0002, 1 },           { "BDDV", 0111, 1, 0, { 1, 0 } }, { "BDDV*", 0113, 1, 0, { 1, 0 } },
    { "BDSU", 0155, 1, 0, { 1, 0 } }, { "BDSU*", 0157, 1, 0, { 1, 0 } }, { "BHIZ", 0146, 1 },     { "BMN", 0136, 1 },
    { "BOF", 0162, 2, 000341 },     { "BOFCLR", 0162, 2, 000241 },  { "BOFF", 0162, 2, 000341 },    { "BOFINV", 0162, 2, 000141 },
    { "BOFSET", 0162, 2, 000041 },  { "BON", 0162, 2, 000301 },     { "BONCLR", 0162, 2, 000201 },  { "BONINV", 0162, 2, 000101 },
    { "BONSET", 0162, 2, 000001 },  { "BOV", 0176, 1 },             { "BOVB", 0172, 1 },            { "BPL", 0132, 1 },
    { "BVSU", 0131, 1, 0, { 1, 0 } }, { "BVSU*", 0133, 1, 0, { 1, 0 } }, { "BZE", 0122, 1 },      { "CALL", 0152, 1 },
    { "CALRB", 0152, 1 },           { "CCALL", 0065, 2, 0, { 1, 0 } }, { "CCALL*", 0067, 2, 0, { 1, 0 } }, { "CGOTO", 0021, 2, 0, { 1, 0 } },
    { "CGOTO*", 0023, 2, 0, { 1, 0 } }, { "CLEAR", 0162, 1, 000261 }, { "CLR", 0162, 1, 000261 },  { "CLRGO", 0162, 2, 000221 },
    { "COS", 0030, 0 },             { "COSINE", 0030, 0 },          { "DAD", 0161, 1, 0, { 1, 0 } }, { "DAD*", 0163, 1, 0, { 1, 0 } },
    { "DCOMP", 0100, 0 },           { "DDV", 0105, 1, 0, { 1, 0 } }, { "DDV*", 0107, 1, 0, { 1, 0 } }, { "DLOAD", 0031, 1, 0, { 1, 0 } },
    { "DLOAD*", 0033, 1, 0, { 1, 0 } }, { "DMP", 0171, 1, 0, { 1, 0 } }, { "DMP*", 0173, 1, 0, { 1, 0 } }, { "DMPR", 0101, 1, 0, { 1, 0 } },
    { "DMPR*", 0103, 1, 0, { 1, 0 } }, { "DOT", 0135, 1, 0, { 1, 0 } }, { "DOT*", 0137, 1, 0, { 1, 0 } }, { "DSQ", 0060, 0 },
    { "DSU", 0151, 1, 0, { 1, 0 } }, { "DSU*", 0153, 1, 0, { 1, 0 } }, { "EXIT", 0000, 0 },       { "GOTO", 0126, 1 },
    { "INCR,1", 0066, 1 },          { "INCR,2", 0062, 1 },          { "INVERT", 0162, 1, 000161 },  { "INVGO", 0162, 2, 000121 },
    { "ITA", 0156, 1 },             { "ITCQ", 0160, 0 },            { "LXA,1", 0026, 1 },           { "LXA,2", 0022, 1 },
    { "LXC,1", 0036, 1 },           { "LXC,2", 0032, 1 },           { "MXV", 0055, 1, 0, { 1, 0 } }, { "MXV*", 0057, 1, 0, { 1, 0 } },
    { "NORM", 0075, 1, 0, { 1, 0 } }, { "NORM*", 0077, 1, 0, { 1, 0 } }, { "PDDL", 0051, 1, 0, { 1, 0 } }, { "PDDL*", 0053, 1, 0, { 1, 0 } },
    { "PDVL", 0061, 1, 0, { 1, 0 } }, { "PDVL*", 0063, 1, 0, { 1, 0 } }, { "PUSH", 0170, 0 },     { "ROUND", 0070, 0 },
    { "RTB", 0142, 1 },             { "RVQ", 0160, 0 },             { "SET", 0162, 1, 000061 },     { "SETGO", 0162, 2, 000021 },
    { "SETPD", 0175, 1, 0, { 1, 0 } }, { "SIGN", 0011, 1, 0, { 1, 0 } }, { "SIGN*", 0013, 1, 0, { 1, 0 } }, { "SIN", 0020, 0 },
    { "SINE", 0020, 0 },            { "SL", 0115, 1, 020202, { 1, 0 } }, { "SL*", 0117, 1, 020202, { 1, 0 } }, { "SL1", 0024, 0, 0, { 1, 0 } },
    { "SL1R", 0004, 0, 0, { 1, 0 } }, { "SL2", 0064, 0, 0, { 1, 0 } }, { "SL2R", 0044, 0, 0, { 1, 0 } }, { "SL3", 0124, 0, 0, { 1, 0 } },
    { "SL3R", 0104, 0, 0, { 1, 0 } }, { "SL4", 0164, 0, 0, { 1, 0 } }, { "SL4R", 0144, 0, 0, { 1, 0 } }, { "SLOAD", 0041, 1, 0, { 1, 0 } },
    { "SLOAD*", 0043, 1, 0, { 1, 0 } }, { "SLR", 0115, 1, 021202, { 1, 0 } }, { "SLR*", 0117, 1, 021202, { 1, 0 } }, { "SQRT", 0010, 0 },
    { "SR", 0115, 1, 020602, { 1, 0 } }, { "SR*", 0117, 1, 020602, { 1, 0 } }, { "SR1", 0034, 0, 0, { 1, 0 } }, { "SR1R", 0014, 0, 0, { 1, 0 } },
    { "SR2", 0074, 0, 0, { 1, 0 } }, { "SR2R", 0054, 0, 0, { 1, 0 } }, { "SR3", 0134, 0, 0, { 1, 0 } }, { "SR3R", 0114, 0, 0, { 1, 0 } },
    { "SR4", 0174, 0, 0, { 1, 0 } }, { "SR4R", 0154, 0, 0, { 1, 0 } }, { "SRR", 0115, 1, 021602, { 1, 0 } }, { "SRR*", 0117, 1, 021602, { 1, 0 } },
    { "SSP", 0045, 2, 0, { 1, 0 } }, { "SSP*", 0047, 1, 0, { 1, 0 } }, { "STADR", 0150, 0 },      { "STQ", 0156, 1 },
    { "SXA,1", 0046, 1 },           { "SXA,2", 0042, 1 },           { "TAD", 0005, 1, 0, { 1, 0 } }, { "TAD*", 0007, 1, 0, { 1, 0 } },
    { "TIX,1", 0076, 1 },           { "TIX,2", 0072, 1 },           { "TLOAD", 0025, 1, 0, { 1, 0 } }, { "TLOAD*", 0027, 1, 0, { 1, 0 } },
    { "UNIT", 0120, 0 },            { "V/SC", 0035, 1, 0, { 1, 0 } }, { "V/SC*", 0037, 1, 0, { 1, 0 } }, { "VAD", 0121, 1, 0, { 1, 0 } },
    { "VAD*", 0123, 1, 0, { 1, 0 } }, { "VCOMP", 0100, 0 },         { "VDEF", 0110, 0 },            { "VLOAD", 0001, 1, 0, { 1, 0 } },
    { "VLOAD*", 0003, 1, 0, { 1, 0 } }, { "VPROJ", 0145, 1, 0, { 1, 0 } }, { "VPROJ*", 0147, 1, 0, { 1, 0 } }, { "VSL", 0115, 1, 020202, { 1, 0 } },
    { "VSL*", 0117, 1, 020202, { 1, 0 } }, { "VSL1", 0004, 0, 0, { 1, 0 } }, { "VSL2", 0024, 0, 0, { 1, 0 } }, { "VSL3", 0044, 0, 0, { 1, 0 } },
    { "VSL4", 0064, 0, 0, { 1, 0 } }, { "VSL5", 0104, 0, 0, { 1, 0 } }, { "VSL6", 0124, 0, 0, { 1, 0 } }, { "VSL7", 0144, 0, 0, { 1, 0 } },
    { "VSL8", 0164, 0, 0, { 1, 0 } }, { "VSQ", 0140, 0 },           { "VSR", 0115, 1, 020602, { 1, 0 } }, { "VSR*", 0117, 1, 020602, { 1, 0 } },
    { "VSR1", 0014, 0, 0, { 1, 0 } }, { "VSR2", 0034, 0, 0, { 1, 0 } }, { "VSR3", 0054, 0, 0, { 1, 0 } }, { "VSR4", 0074, 0, 0, { 1, 0 } },
    { "VSR5", 0114, 0, 0, { 1, 0 } }, { "VSR6", 0134, 0, 0, { 1, 0 } }, { "VSR7", 0154, 0, 0, { 1, 0 } }, { "VSR8", 0174, 0, 0, { 1, 0 } },
    { "VSU", 0125, 1, 0, { 1, 0 } }, { "VSU*", 0127, 1, 0, { 1, 0 } }, { "VXM", 0071, 1, 0, { 1, 0 } }, { "VXM*", 0073, 1, 0, { 1, 0 } },
    { "VXSC", 0015, 1, 0, { 1, 0 } }, { "VXSC*", 0017, 1, 0, { 1, 0 } }, { "VXV", 0141, 1, 0, { 1, 0 } }, { "VXV*", 0143, 1, 0, { 1, 0 } },
    { "XAD,1", 0106, 1 },           { "XAD,2", 0102, 1 },           { "XCHX,1", 0056, 1 },          { "XCHX,2", 0052, 1 },
    { "XSU,1", 0116, 1 },           { "XSU,2", 0112, 1 },
};

static const yul::parser_match_t* find_parser(std::string_view name)
{
    static const std::unordered_map<std::string_view, const yul::parser_match_t*> table = [] {
        std::unordered_map<std::string_view, const yul::parser_match_t*> map;
        for (const yul::parser_match_t& match : parsers)
            map.emplace(match.name, &match);
        return map;
    }();

    const auto found = table.find(name);
    return found == table.end() ? nullptr : found->second;
}

static const yul::interpreter_match_t* find_interpreter(std::string_view name)
{
    static const std::unordered_map<std::string_view, const yul::interpreter_match_t*> table = [] {
        std::unordered_map<std::string_view, const yul::interpreter_match_t*> map;
        for (const yul::interpreter_match_t& match : interpreters)
            map.emplace(match.name, &match);
        return map;
    }();

    const auto found = table.find(name);
    return found == table.end() ? nullptr : found->second;
}

// +n, -n, +nD or -nD: a label in column 2 that only marks an offset
static bool is_false_label(std::string_view text)
{
    if (text.size() < 2 || (text[0] != '-' && text[0] != '+') || !std::isdigit(static_cast<unsigned char>(text[1])))
        return false;

    std::size_t i = 2;
    while (i < text.size() && std::isdigit(static_cast<unsigned char>(text[i])))
        ++i;

    return i == text.size() || (text[i] == 'D' && i + 1 == text.size());
}

static int add_agc(int lhs, int rhs)
{
    return 077777 & (lhs + rhs);
}

// ones' complement addition with the carry wrapped around, as the bank checksums are made
static int add_checksum(int lhs, int rhs)
{
    const auto native = [](int word) { return (word & 040000) ? -(077777 & ~word) : word; };

    int sum = native(lhs) + native(rhs);
    if (sum > 16383)
        sum = sum - 16384 + 1;
    else if (sum < -16383)
        sum = sum + 16384 - 1;

    return sum >= 0 ? sum : 077777 & ~(-sum);
}

static double scale_factor(std::string_view text)
{
    if (text.empty())
        return 1.0;

    const int exponent = std::atoi(std::string(text.substr(1)).c_str());
    if (text[0] == 'E')
        return std::pow(10.0, exponent);
    if (text[0] == 'B')
        return std::pow(2.0, exponent);

    return 1.0;
}

static bool fuzzed_is_int(double x)
{
    x = std::fabs(std::fmod(x, 1.0));
    return x < 1.0e-10 || x > 1.0 - 1.0e-10;
}

static std::string expand_tabs(std::string_view line)
{
    std::string expanded;
    for (const char c : line)
    {
        if (c == '\t')
        {
            do
                expanded += ' ';
            while (expanded.size() % 8 != 0);
        }
        else
            expanded += c;
    }
    return expanded;
}

// -----------------------------------------------------------------------------------------------------------------
// addresses

yul::address_t yul::invalid_address()
{
    yul::address_t address;
    address.invalid = true;
    return address;
}

yul::address_t yul::pseudo_to_struct(int value)
{
    yul::address_t address;
    if (value < 0 || value > 0117777)
    {
        address.invalid = true;
        return address;
    }

    address.address = true;
    if (value <= 01377)
    {
        address.erasable = true;
        address.unbanked = true;
        address.sreg = value;
    }
    else if (value <= 03777)
    {
        address.erasable = true;
        address.banked = true;
        address.sreg = 01400 + (value & 0377);
        address.eb = value / 0400;
    }
    else if (value <= 07777)
    {
        address.fixed = true;
        address.unbanked = true;
        address.sreg = value;
    }
    else
    {
        address.fixed = true;
        address.banked = true;
        address.sreg = 02000 + (value & 01777);
        if (value < 0110000)
            address.fb = (value - 010000) / 02000;
        else
        {
            address.super = true;
            address.fb = (value - 030000) / 02000;
        }
    }

    address.value = value;
    return address;
}

yul::address_t yul::inc_pc(const yul::address_t& old_pc, int increment)
{
    int bank_increment = 0;
    if (increment >= 0)
    {
        bank_increment = 7 & (increment >> 12);
        increment &= 07777;
    }
    else
    {
        increment = -increment;
        bank_increment = -(7 & (increment >> 12));
        increment = -(07777 & increment);
    }

    yul::address_t pc = old_pc;
    if (pc.invalid)
        return pc;

    if (pc.constant)
    {
        if (pc.value < 0 || pc.value >= 01400)
            return pc;

        // a small constant counts as an unbanked erasable address
        pc.constant = false;
        pc.address = true;
        pc.erasable = true;
        pc.fixed = false;
        pc.banked = false;
        pc.unbanked = true;
        pc.eb = 0;
        pc.fb = 0;
        pc.super = false;
        pc.sreg = pc.value;
    }

    if (pc.overflow)
        return pc;

    const int sreg = pc.sreg + increment;
    pc.sreg = sreg & 07777;
    if (pc.erasable && pc.banked)
        pc.eb = (pc.eb + bank_increment) & 7;
    else if (pc.fixed && pc.banked)
        pc.fb = (pc.fb + bank_increment) & 037;

    if (pc.fixed && bank_increment != 0 && pc.fb == 0)
    {
        pc.constant = false;
        pc.address = true;
        pc.erasable = true;
        pc.fixed = false;
        pc.banked = false;
        pc.unbanked = true;
        pc.eb = 0;
        pc.fb = 0;
        pc.super = false;
    }

    // yaYUL steps a banked address's bank a second time here, kept for the same results
    int min = 0, max = 0;
    if (pc.erasable && pc.unbanked)
        max = 01377;
    else if (pc.erasable && pc.banked)
    {
        min = 01400;
        max = 01777;
        pc.eb = (pc.eb + bank_increment) & 7;
    }
    else if (pc.fixed && pc.banked)
    {
        min = 02000;
        max = 03777;
        pc.fb = (pc.fb + bank_increment) & 037;
    }
    else if (pc.fixed && pc.unbanked)
    {
        min = 04000;
        max = 07777;
    }
    else
    {
        pc.invalid = true;
        return pc;
    }

    if (sreg < min)
    {
        pc.overflow = true;
        pc.sreg = min;
    }
    else if (sreg > max)
    {
        pc.overflow = true;
        pc.sreg = max;
    }

    if (pc.unbanked)
        pc.value = pc.sreg;
    else if (pc.erasable)
        pc.value = (pc.sreg - 01400) + 0400 * pc.eb;
    else
    {
        pc.value = 010000 + (pc.sreg - 02000) + 02000 * pc.fb;
        if (pc.super && pc.fb >= 030)
            pc.value += 010 * 02000;
    }

    return pc;
}

std::optional<int> yul::parse_number(std::string_view text)
{
    std::size_t start = 0;
    if (start < text.size() && (text[start] == '+' || text[start] == '-'))
        ++start;
    if (start == text.size() || text[start] == 'D')
        return std::nullopt;

    const bool negative = text[0] == '-';
    const auto convert = [&](std::string_view digits, int base) {
        int value = 0;
        for (const char c : digits)
            value = value * base + (c - '0');
        return negative ? -value : value;
    };

    std::size_t end = start;
    while (end < text.size() && text[end] >= '0' && text[end] <= '7')
        ++end;
    if (end == text.size())
        return convert(text.substr(start), 8);

    bool has_8_or_9 = false;
    end = start;
    for (; end < text.size(); ++end)
    {
        if (text[end] == '8' || text[end] == '9')
            has_8_or_9 = true;
        else if (text[end] < '0' || text[end] > '9')
            break;
    }

    if ((end < text.size() && text[end] == 'D' && end + 1 == text.size()) || (has_8_or_9 && end == text.size()))
        return convert(text.substr(start, end - start), 10);

    return std::nullopt;
}

// -----------------------------------------------------------------------------------------------------------------
// the assembler

std::expected<yul::assembly_t, std::vector<std::string>> yul::assemble(const std::filesystem::path& main_file)
{
    const auto assembler = std::make_unique<yul::c_assembler>(main_file); // its working state is too big for the stack
    return assembler->run();
}

yul::c_assembler::c_assembler(std::filesystem::path main_file)
    : m_main_file(std::move(main_file))
{
}

std::expected<yul::assembly_t, std::vector<std::string>> yul::c_assembler::run()
{
    if (!load(m_main_file, 0))
        return std::unexpected(m_errors);

    collect_symbols();

    const auto unresolved = [this] {
        int count = 0;
        for (const auto& [name, value] : m_symbols)
            count += value.invalid;
        return count;
    };

    // passes until the symbols settle, then one that writes the rope
    int last_unresolved = unresolved();
    bool written = false;
    for (int i = 0; i < 10 && !written; ++i)
    {
        pass(false);

        const int now_unresolved = unresolved();
        if ((now_unresolved == 0 || now_unresolved >= last_unresolved) && m_symbols_reassigned == 0)
        {
            pass(true);
            written = true;
        }
        last_unresolved = now_unresolved;
    }
    if (!written)
        pass(true);

    if (!m_errors.empty())
        return std::unexpected(m_errors);

    add_bugger_words();
    m_assembly.symbols = m_symbols;
    return std::move(m_assembly);
}

bool yul::c_assembler::load(const std::filesystem::path& file, int depth)
{
    if (depth > 10)
    {
        m_errors.push_back(std::format("{}: includes nested too deep", file.string()));
        return false;
    }

    std::ifstream stream(file);
    if (!stream)
    {
        m_errors.push_back(std::format("{}: can't open", file.string()));
        return false;
    }

    std::string raw;
    int number = 0;
    while (std::getline(stream, raw))
    {
        ++number;
        if (!raw.empty() && raw.back() == '\r')
            raw.pop_back();

        if (!raw.empty() && raw[0] == '$')
        {
            const std::size_t end = raw.find_first_of(" \t", 1);
            if (!load(file.parent_path() / raw.substr(1, end == std::string::npos ? std::string::npos : end - 1), depth + 1))
                return false;
            continue;
        }

        yul::source_line_t line;
        line.file = file.filename().string();
        line.line_number = number;
        line.text = expand_tabs(raw);
        if (const std::size_t comment = line.text.find('#'); comment != std::string::npos)
            line.text.resize(comment);

        for (std::size_t i = 0; i < line.text.size() && line.fields.size() < 6; )
        {
            while (i < line.text.size() && std::isspace(static_cast<unsigned char>(line.text[i])))
                ++i;
            if (i == line.text.size())
                break;

            const std::size_t start = i;
            while (i < line.text.size() && !std::isspace(static_cast<unsigned char>(line.text[i])))
                ++i;

            if (line.fields.empty())
                line.first_column = static_cast<int>(start);
            line.fields.push_back(line.text.substr(start, i - start));
        }

        m_lines.push_back(std::move(line));
    }

    return true;
}

void yul::c_assembler::collect_symbols()
{
    for (const yul::source_line_t& line : m_lines)
        if (!line.fields.empty() && line.first_column == 0)
            m_symbols.try_emplace(line.fields[0], yul::invalid_address());

    // the registers the aliases (EXTEND is TC $6) refer to
    const auto reg = [](int n) {
        yul::address_t address;
        address.address = true;
        address.erasable = true;
        address.unbanked = true;
        address.sreg = n;
        address.value = n;
        return address;
    };
    m_symbols["$3"] = reg(03);
    m_symbols["$4"] = reg(04);
    m_symbols["$5"] = reg(05);
    m_symbols["$6"] = reg(06);
    m_symbols["$7"] = reg(07);
    m_symbols["$17"] = reg(017);
}

yul::address_t* yul::c_assembler::find_symbol(const std::string& name)
{
    if (name.size() > 10)
        return nullptr;

    const auto found = m_symbols.find(name);
    return found == m_symbols.end() ? nullptr : &found->second;
}

void yul::c_assembler::edit_symbol(const std::string& name, const yul::address_t& value)
{
    yul::address_t* symbol = find_symbol(name);
    if (!symbol)
        return;

    if (*symbol != value)
        ++m_symbols_reassigned;
    *symbol = value;
}

bool yul::c_assembler::fetch_symbol_plus_offset(const yul::address_t& pc, const std::string& operand, const std::string& mod1, yul::address_t& value)
{
    if (operand.empty())
    {
        value = pc;
        return true;
    }

    if (const std::optional<int> offset = yul::parse_number(operand))
    {
        value = yul::inc_pc(pc, *offset);
        return true;
    }

    value.invalid = true;
    const yul::address_t* symbol = find_symbol(operand);
    if (!symbol)
        return false;

    value = *symbol;
    if (const std::optional<int> offset = yul::parse_number(mod1))
        value = yul::pseudo_to_struct(symbol->value + *offset);

    return true;
}

void yul::c_assembler::start_bank_counts()
{
    m_used_in_bank.fill(0);
}

void yul::c_assembler::update_bank_counts(const yul::address_t& pc)
{
    if (pc.invalid || !pc.address || !pc.fixed || pc.overflow)
        return;

    int count = 0, bank = 0;
    if (pc.banked)
    {
        count = pc.sreg - 02000;
        bank = pc.fb + 010 * pc.super;
    }
    else if (pc.unbanked)
    {
        if (pc.sreg >= 04000 && pc.sreg < 06000)
        {
            count = pc.sreg - 04000;
            bank = 2;
        }
        else if (pc.sreg >= 06000 && pc.sreg < 010000)
        {
            count = pc.sreg - 06000;
            bank = 3;
        }
        else
            return;
    }

    if (count > m_used_in_bank[static_cast<std::size_t>(bank)])
        m_used_in_bank[static_cast<std::size_t>(bank)] = count;
}

void yul::c_assembler::pass(bool write)
{
    m_symbols_reassigned = 0;
    m_prior_used_in_bank = m_used_in_bank;
    start_bank_counts();
    m_stadr_invert = 0;
    if (write)
        m_assembly = {};

    yul::line_output_t out;
    out.pc = yul::invalid_address();
    out.ebank = { false, yul::invalid_address(), yul::invalid_address() };

    static const std::string empty;
    for (const yul::source_line_t& line : m_lines)
    {
        m_line = &line;
        m_opcode_offset = 0;
        m_arg_type = 0;
        bool pinch_hitting = false;

        yul::line_input_t in;
        in.pc = out.pc;
        in.ebank = out.ebank;
        in.sbank = out.sbank;
        in.index_valid = out.index_valid;
        in.extend = out.extend;

        out = yul::line_output_t{};
        out.pc = yul::invalid_address();
        out.label_value = yul::invalid_address();
        out.words = { 0, 0 };
        out.ebank = in.ebank;
        out.sbank = in.sbank;

        // split the line into label, operator, operand and modifiers. an operand line of interpretive code has no
        // operator, it's recognized by an interpretive opcode still waiting for its operands
        const std::vector<std::string>& fields = line.fields;
        const auto field = [&](std::size_t n) -> const std::string& { return n < fields.size() ? fields[n] : empty; };
        std::size_t i = 0;
        if (!fields.empty())
        {
            bool operand_only = false;
            if (line.first_column == 0)
                in.label = fields[i++];
            else if (is_false_label(fields[0]) && line.first_column < 16) // yaYUL stops at 8, but this repo's sources have them at 8 too
            {
                if (fields.size() == 1)
                    operand_only = true;
                else
                    in.false_label = fields[i++];
            }
            else if (line.first_column < 8)
                ++i;

            if (!operand_only)
            {
                const std::string& candidate = field(i);
                const bool at_operator_column = line.text.size() >= 16 && line.text.compare(16, candidate.size(), candidate) == 0;
                const yul::interpreter_match_t* imatch = at_operator_column ? find_interpreter(candidate) : nullptr;
                const yul::parser_match_t* match = find_parser(candidate);

                if (m_num_interpretive_operands && !imatch && !match)
                    in.op.clear();
                else if (match && m_num_interpretive_operands && i + 1 >= fields.size())
                    in.op.clear(); // an opcode name used as an operand
                else if (match && match->pinch_hit && m_num_interpretive_operands)
                {
                    --m_num_interpretive_operands;
                    pinch_hitting = true;
                    if (i < fields.size())
                        in.op = fields[i++];
                }
                else
                {
                    m_num_interpretive_operands = 0;
                    if (i < fields.size())
                        in.op = fields[i++];
                }
            }

            in.operand = field(i++);
            in.mod1 = field(i++);
            in.mod2 = field(i++);
            in.extra = field(i);
        }

        if (in.operand.empty() && in.op == "TC")
            in.operand = "-0";

        if (in.op.empty() && !m_num_interpretive_operands)
        {
            out.pc = in.pc;
            out.extend = in.extend;
            out.ebank = in.ebank;
            out.sbank = in.sbank;
            out.index_valid = in.index_valid;
        }
        else
        {
            if (in.op == "NOOP" && !m_num_interpretive_operands)
            {
                in.op = in.pc.erasable ? "CA" : "TCF";
                in.operand = in.pc.erasable ? "A" : "+1";
                in.alias = "NOOP";
            }

            for (;;)
            {
                if (const yul::interpreter_match_t* imatch = find_interpreter(in.op))
                {
                    // one or two interpretive opcodes, packed into a word with their operands to follow
                    if (in.op == "STADR" || in.operand == "STADR")
                        m_stadr_invert = 2;

                    const yul::interpreter_match_t* imatch2 = nullptr;
                    if (!in.operand.empty())
                    {
                        imatch2 = find_interpreter(in.operand);
                        if (!imatch2)
                        {
                            out.error = std::format("Unrecognized interpretive opcode \"{}\".", in.operand);
                            out.fatal = true;
                        }
                    }

                    m_num_interpretive_operands = 0;
                    out.num_words = 1;
                    for (const yul::interpreter_match_t* opcode : { imatch, imatch2 })
                    {
                        if (!opcode || !opcode->num_operands)
                            continue;

                        m_switch_increment[static_cast<std::size_t>(m_num_interpretive_operands)] = opcode->arg_types[0];
                        m_nnnn_fields[static_cast<std::size_t>(m_num_interpretive_operands++)] = opcode->nnnn0000;
                        if (opcode->num_operands > 1)
                        {
                            m_switch_increment[static_cast<std::size_t>(m_num_interpretive_operands)] = opcode->arg_types[1];
                            m_nnnn_fields[static_cast<std::size_t>(m_num_interpretive_operands++)] = 0;
                        }
                    }
                    m_raw_num_interpretive_operands = m_num_interpretive_operands;

                    out.words[0] = 0177 & (imatch->code + 1);
                    if (imatch2)
                        out.words[0] |= 037600 & ((imatch2->code + 1) << 7);
                    out.words[0] = 077777 & ~out.words[0];

                    out.pc = yul::inc_pc(in.pc, out.num_words);
                    out.ebank = in.ebank;
                    out.sbank = in.sbank;
                    break;
                }

                if (m_num_interpretive_operands && !pinch_hitting)
                {
                    if (in.op.empty() && in.operand.empty())
                    {
                        out.words[0] = 0;
                        out.pc = in.pc;
                    }
                    else if (in.op.empty())
                    {
                        out.num_words = 1;
                        parse_interpretive_operand(in, out);
                        --m_num_interpretive_operands;
                        out.pc = yul::inc_pc(in.pc, out.num_words);
                    }
                    else
                    {
                        out.error = "Missing interpretive operands.";
                        out.fatal = true;
                        out.num_words = 1;
                        out.words[0] = 0;
                        m_num_interpretive_operands = 0;
                        out.pc = yul::inc_pc(in.pc, out.num_words);
                    }
                    out.ebank = in.ebank;
                    out.sbank = in.sbank;
                    break;
                }

                const yul::parser_match_t* match = find_parser(in.op);
                if (!match)
                {
                    if (const std::optional<int> numeric_op = yul::parse_number(in.op))
                    {
                        // a number in place of the operator: the opcode bits themselves
                        int value = 0;
                        if (const std::optional<int> number = yul::parse_number(in.operand))
                            value = *number;
                        else
                        {
                            yul::address_t k;
                            if (!fetch_symbol_plus_offset(in.pc, in.operand, in.mod1, k))
                            {
                                out.error = std::format("Symbol \"{}\" undefined or offset bad", in.operand);
                                out.fatal = true;
                            }
                            value = k.sreg;
                        }

                        const int n = *numeric_op;
                        int flags = 0;
                        if ((!in.extend && n == 2 && (value & 06000) == 0) || (!in.extend && n == 5 && (value & 06000) == 02000)
                            || (in.extend && (n == 4 || n == 5)))
                            flags |= flag_kplus1;

                        parse_general(in, out, n << 12, flags);
                        out.words[0] = add_agc(out.words[0], m_opcode_offset);
                        out.ebank.oneshot_pending = false;
                        out.sbank.oneshot_pending = false;
                    }
                    else
                    {
                        out.ebank = in.ebank;
                        out.sbank = in.sbank;
                        out.error = std::format("Unrecognized opcode/pseudo-op \"{}\".", in.op);
                        out.fatal = true;
                        out.pc = yul::inc_pc(in.pc, 1);
                    }
                    break;
                }

                if (match->kind == yul::parser_kind::none && match->alias_op.empty())
                {
                    out.pc = in.pc; // ignored
                    break;
                }

                if (match->kind == yul::parser_kind::none)
                {
                    in.alias = in.op;
                    in.op = match->alias_op;
                    in.operand = match->alias_operand;
                    in.mod1.clear();
                    in.mod2.clear();
                    continue;
                }

                parse(*match, in, out);

                const int before = out.words[0];
                out.words[0] = add_agc(out.words[0], m_opcode_offset);
                if ((out.words[0] & 040000) && !(before & 040000))
                    --out.words[0];
                out.words[0] = (out.words[0] + match->adder) ^ match->xmask;
                out.words[1] = (out.words[1] + match->adder2) ^ match->xmask2;

                const bool one_shot = match->kind == yul::parser_kind::bbcon || match->kind == yul::parser_kind::two_cadr;
                if (in.ebank.oneshot_pending && one_shot)
                    out.ebank.current = in.ebank.last;
                if (in.sbank.oneshot_pending && one_shot)
                    out.sbank.current = in.sbank.last;
                if (match->kind != yul::parser_kind::ebank_equals)
                    out.ebank.oneshot_pending = false;
                if (match->kind != yul::parser_kind::sbank_equals)
                    out.sbank.oneshot_pending = false;
                break;
            }
        }

        if (m_stadr_invert && out.num_words > 0)
        {
            if (m_stadr_invert == 1)
                out.words[0] = 077777 & ~out.words[0];
            --m_stadr_invert;
        }

        update_bank_counts(out.pc);
        if (out.pc.fb >= 030 && out.pc.fb <= 037 && out.num_words > 0)
            out.sbank.current = out.pc.super ? 4 : 3;

        if (!in.label.empty() && !out.equals && in.op != "MEMORY")
            edit_symbol(in.label, in.pc);

        if (!write)
            continue;

        if (out.fatal)
            m_errors.push_back(std::format("{}:{}: {}", line.file, line.line_number, out.error));

        if (out.num_words > 0 && out.words[0] != yul::illegal_word && !in.pc.invalid && in.pc.address && in.pc.fixed)
        {
            int bank = in.pc.sreg / 02000;
            if (in.pc.banked)
                bank = in.pc.fb + ((in.pc.fb >= 020 && in.pc.super) ? 010 : 0);

            const int origin = static_cast<int>(m_assembly.sources.size());
            m_assembly.sources.push_back(std::format("{}:{}", line.file, line.line_number));

            for (int word = 0; word < out.num_words; ++word)
            {
                const std::size_t offset = static_cast<std::size_t>((in.pc.sreg + word) & 01777);
                m_assembly.rope[static_cast<std::size_t>(bank)][offset] = static_cast<std::uint16_t>(out.words[static_cast<std::size_t>(word)] & 077777);
                m_assembly.used[static_cast<std::size_t>(bank)][offset] = true;
                m_assembly.origin[static_cast<std::size_t>(bank)][offset] = origin;
            }
        }
    }
}

// each bank that has code ends in two TC-to-itself words and a checksum word that makes the bank add up to its own number
void yul::c_assembler::add_bugger_words()
{
    for (std::size_t bank = 0; bank < agc::fixed_bank_count; ++bank)
    {
        int used = m_used_in_bank[bank];
        if (used <= 0)
            continue;

        const int base = bank == 2 ? 04000 : bank == 3 ? 06000 : 02000;
        agc::rope_bank_t& words = m_assembly.rope[bank];

        for (const int limit : { 01776, 01777 })
        {
            if (used < limit)
            {
                words[static_cast<std::size_t>(used)] = static_cast<std::uint16_t>(used + base);
                m_assembly.used[bank][static_cast<std::size_t>(used)] = true;
                m_assembly.origin[bank][static_cast<std::size_t>(used)] = -1;
                ++used;
            }
        }

        if (used < 02000)
        {
            int sum = 0;
            for (int i = 0; i < used; ++i)
                sum = add_checksum(sum, words[static_cast<std::size_t>(i)]);

            const int bank_number = static_cast<int>(bank);
            const int bugger = (sum & 040000) == 0 ? add_checksum(bank_number, 077777 & ~sum)
                                                   : add_checksum(077777 & ~bank_number, 077777 & ~sum);
            words[static_cast<std::size_t>(used)] = static_cast<std::uint16_t>(bugger);
            m_assembly.used[bank][static_cast<std::size_t>(used)] = true;
            m_assembly.origin[bank][static_cast<std::size_t>(used)] = -1;
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------
// the operators

int yul::c_assembler::parse_comma(yul::line_input_t& in)
{
    const auto strip = [](std::string& text) {
        if (text.size() < 3)
            return 0;
        const std::string_view tail = std::string_view(text).substr(text.size() - 2);
        const int type = tail == ",1" ? 1 : tail == ",2" ? 2 : 0;
        if (type)
            text.resize(text.size() - 2);
        return type;
    };

    if (!in.mod1.empty())
        return strip(in.mod1);
    if (!in.operand.empty())
        return strip(in.operand);
    return 0;
}

void yul::c_assembler::parse(const yul::parser_match_t& match, yul::line_input_t& in, yul::line_output_t& out)
{
    switch (match.kind)
    {
    case yul::parser_kind::general:
        parse_general(in, out, match.opcode, match.flags);
        break;
    case yul::parser_kind::tcf:
        if (in.operand.empty())
            in.operand = "+0";
        parse_general(in, out, 010000, flag_qcnot0 | flag_enumber);
        break;
    case yul::parser_kind::edrupt:
        parse_general(in, out, 0, flag_extended | flag_pc7);
        break;
    case yul::parser_kind::index:
        parse_index(in, out);
        break;
    case yul::parser_kind::st_call:
    case yul::parser_kind::st_odl:
    case yul::parser_kind::st_ovl:
    {
        static constexpr std::array<int, 3> opcodes = { 034000, 014000, 024000 };
        const std::size_t which = match.kind == yul::parser_kind::st_call ? 0 : match.kind == yul::parser_kind::st_odl ? 1 : 2;
        m_arg_type = parse_comma(in);
        m_switch_increment[0] = which == 0 ? 0 : 1; // the load that follows STODL and STOVL is address + 1
        m_nnnn_fields[0] = 0;
        m_raw_num_interpretive_operands = m_num_interpretive_operands = 1;
        parse_st(in, out, opcodes[which], flag_erasable | flag_enumber | flag_kplus1);
        break;
    }
    case yul::parser_kind::st_ore:
        m_arg_type = parse_comma(in);
        m_num_interpretive_operands = 0;
        parse_st(in, out, 0, flag_erasable | flag_enumber | flag_kplus1);
        break;
    case yul::parser_kind::equals:       parse_equals(in, out); break;
    case yul::parser_kind::eq_minus:     parse_eq_minus(in, out); break;
    case yul::parser_kind::erase:        parse_erase(in, out); break;
    case yul::parser_kind::setloc:       parse_setloc(in, out); break;
    case yul::parser_kind::bank:         parse_bank(in, out); break;
    case yul::parser_kind::block:        parse_block(in, out); break;
    case yul::parser_kind::ebank_equals: parse_ebank_equals(in, out); break;
    case yul::parser_kind::sbank_equals: parse_sbank_equals(in, out); break;
    case yul::parser_kind::cadr:         parse_cadr(in, out); break;
    case yul::parser_kind::two_cadr:     parse_2cadr(in, out); break;
    case yul::parser_kind::ecadr:        parse_ecadr(in, out); break;
    case yul::parser_kind::genadr:       parse_genadr(in, out); break;
    case yul::parser_kind::bbcon:        parse_bbcon(in, out, false); break;
    case yul::parser_kind::bbcon_star:   parse_bbcon(in, out, true); break;
    case yul::parser_kind::oct:          parse_oct(in, out); break;
    case yul::parser_kind::two_oct:      parse_2oct(in, out); break;
    case yul::parser_kind::dec:          parse_dec(in, out); break;
    case yul::parser_kind::two_dec:      parse_2dec(in, out); break;
    case yul::parser_kind::vn:           parse_vn(in, out); break;
    case yul::parser_kind::none:         break;
    }
}

void yul::c_assembler::parse_general(yul::line_input_t& in, yul::line_output_t& out, int opcode, int flags)
{
    const std::string args = in.mod1 + in.mod2;

    out.pc = yul::inc_pc(in.pc, 1);
    out.ebank = in.ebank;
    out.sbank = in.sbank;
    out.num_words = 1;

    if (flags & flag_pc1)      opcode |= 01000;
    else if (flags & flag_pc2) opcode |= 02000;
    else if (flags & flag_pc3) opcode |= 03000;
    else if (flags & flag_pc4) opcode |= 04000;
    else if (flags & flag_pc5) opcode |= 05000;
    else if (flags & flag_pc6) opcode |= 06000;
    else if (flags & flag_pc7) opcode |= 07000;
    else if (flags & flag_qc1) opcode |= 02000;
    else if (flags & flag_qc2) opcode |= 04000;
    else if (flags & flag_qc3) opcode |= 06000;
    out.words[0] = opcode;
    if (flags & flag_qcnot0)
        out.words[0] |= 06000;

    if (in.extend && !(flags & flag_extended) && !in.index_valid)
    {
        if (in.extend != 2)
        {
            out.error = "Illegally preceded by EXTEND.";
            out.fatal = true;
        }
        out.extend = 0;
    }
    else if (!in.extend && (flags & flag_extended))
    {
        out.error = "Required EXTEND is missing.";
        out.fatal = true;
        out.extend = 0;
    }

    yul::address_t k = yul::invalid_address();
    bool found = true;
    const std::optional<int> number = yul::parse_number(in.operand);
    if (number && (flags & flag_enumber) && in.operand[0] != '+' && in.operand[0] != '-')
    {
        k = yul::pseudo_to_struct(*number);
        if (const std::optional<int> offset = yul::parse_number(args))
            m_opcode_offset = *offset;
    }
    else if (number)
    {
        k = (in.operand[0] == '+' || in.operand[0] == '-') ? yul::inc_pc(in.pc, *number) : yul::pseudo_to_struct(*number);
        if (!args.empty())
            if (const std::optional<int> offset = yul::parse_number(args))
                m_opcode_offset = *offset;
    }
    else if (!fetch_symbol_plus_offset(in.pc, in.operand, args, k))
    {
        out.error = std::format("Symbol \"{}\" undefined or offset bad", in.operand);
        out.fatal = true;
        found = false;
    }

    if (found)
    {
        if (k.invalid)
        {
            out.error = "Destination address not resolved.";
            out.fatal = true;
        }
        else if (k.overflow)
        {
            out.error = "Destination address out of range.";
            out.fatal = true;
        }
        else if (!k.address && !k.constant)
        {
            out.error = "Destination not an address.";
            out.fatal = true;
        }
        else
        {
            if (k.constant)
                k = yul::pseudo_to_struct(k.value);
            if (flags & flag_kplus1)
                k = yul::inc_pc(k, 1);

            int i = k.sreg;
            if (!in.index_valid)
            {
                if ((flags & flag_fixed) && !k.fixed)
                {
                    i &= ~07000;
                    out.error = "The address is not in fixed memory.";
                    out.fatal = true;
                }
                else if ((flags & flag_erasable) && !k.erasable)
                {
                    i &= ~07600;
                    out.error = "The address is not in erasable memory.";
                    out.fatal = true;
                }

                if (flags & any_pc)
                {
                    if ((i & 07000) != 0 && (i & 07000) != (opcode & 07000))
                    {
                        i &= ~07000;
                        out.error = "Operand out of range.";
                        out.fatal = true;
                    }
                }
                else if (flags & any_qc)
                {
                    if ((i & 06000) != 0 && (i & 06000) != (opcode & 06000))
                    {
                        i &= ~06000;
                        out.error = std::format("Operand ({:o}) out of range.", i);
                        out.fatal = true;
                    }
                }
                else if (flags & flag_qcnot0)
                {
                    if ((k.sreg & 06000) == 0)
                    {
                        i |= 06000;
                        out.error = "Operand out of range.";
                        out.fatal = true;
                    }
                }
            }
            out.words[0] = opcode | i;
        }
    }

    out.extend = 0;
    if (opcode == 0 && !(flags & flag_extended) && !k.invalid && (k.constant || (k.erasable && k.unbanked)) && k.sreg == 06)
        out.extend = in.alias.empty() ? 2 : 1;
    out.index_valid = false;
}

void yul::c_assembler::parse_st(yul::line_input_t& in, yul::line_output_t& out, int opcode, int flags)
{
    opcode += 04000 * m_arg_type;
    out.pc = yul::inc_pc(in.pc, 1);
    out.ebank = in.ebank;
    out.sbank = in.sbank;
    out.num_words = 1;
    out.words[0] = opcode;

    if (in.extend && !in.index_valid)
    {
        out.error = "Illegally preceded by EXTEND.";
        out.fatal = true;
    }

    yul::address_t k = yul::invalid_address();
    bool found = true;
    const std::optional<int> number = yul::parse_number(in.operand);
    if (number && (flags & flag_enumber) && in.operand[0] != '+' && in.operand[0] != '-')
    {
        k = yul::pseudo_to_struct(*number);
        if (const std::optional<int> offset = yul::parse_number(in.mod1))
            m_opcode_offset = *offset;
    }
    else if (number && in.mod1.empty())
        k = yul::inc_pc(in.pc, *number);
    else if (fetch_symbol_plus_offset(in.pc, in.operand, in.mod1, k))
    {
        if (k.constant)
            k = yul::pseudo_to_struct(k.value);
    }
    else
    {
        out.error = std::format("Symbol \"{}\" undefined or offset bad", in.operand);
        out.fatal = true;
        found = false;
    }

    if (found)
    {
        if (k.invalid)
        {
            out.error = "Destination address not resolved.";
            out.fatal = true;
        }
        else if (k.overflow)
        {
            out.error = "Destination address out of range.";
            out.fatal = true;
        }
        else if (!k.address)
        {
            out.error = "Destination not an address.";
            out.fatal = true;
        }
        else
        {
            if (k.constant)
                k = yul::pseudo_to_struct(k.value);
            if (flags & flag_kplus1)
                k = yul::inc_pc(k, 1);

            int i = 0;
            if (!k.erasable)
            {
                out.error = "The Address is not in erasable memory.";
                out.fatal = true;
            }
            else
                i = k.banked ? 0400 * k.eb + (k.sreg - 01400) : k.sreg;
            out.words[0] = opcode | i;
        }
    }

    out.extend = 0;
    out.index_valid = false;
}

void yul::c_assembler::parse_interpretive_operand(yul::line_input_t& in, yul::line_output_t& out)
{
    m_arg_type = parse_comma(in);
    out.pc = yul::inc_pc(in.pc, 1);
    out.ebank = in.ebank;
    out.sbank = in.sbank;
    out.num_words = 1;
    out.words[0] = 0;

    if (in.extend && !in.index_valid)
    {
        out.error = "Illegally preceded by EXTEND.";
        out.fatal = true;
    }
    if (in.index_valid)
    {
        out.error = "Illegally preceded by INDEX.";
        out.fatal = true;
    }

    yul::address_t k;
    bool found = true;
    if (const std::optional<int> number = yul::parse_number(in.operand))
    {
        if (in.operand[0] == '+' || in.operand[0] == '-')
            k = yul::inc_pc(in.pc, *number);
        else
        {
            k.constant = true;
            k.value = *number;
        }
    }
    else
        found = fetch_symbol_plus_offset(in.pc, in.operand, "", k);

    if (!found || k.invalid)
    {
        out.error = std::format("Operand \"{}\" not resolved.", in.operand);
        out.fatal = true;
        return;
    }

    int modifier = 0;
    if (!in.mod1.empty())
    {
        const std::optional<int> value = yul::parse_number(in.mod1 + in.mod2);
        if (!value)
        {
            out.error = std::format("Modifier \"{}\" not resolved.", in.mod1);
            out.fatal = true;
            return;
        }
        modifier = *value;
    }
    m_opcode_offset = modifier;

    const std::size_t operand_index = static_cast<std::size_t>(m_raw_num_interpretive_operands - m_num_interpretive_operands);
    for (;;)
    {
        if (k.constant)
        {
            const int nnnn = m_nnnn_fields[operand_index];
            if ((nnnn & 3) == 1)
            {
                // a switch (flag bit) number: its word and bit within the flagwords, and what to do with it
                out.words[0] = (((k.value / 15) & 077) << 8) | (k.value % 15);
                out.words[0] |= 0360 & nnnn;
            }
            else if ((nnnn & 3) == 2)
                out.words[0] = k.value | (nnnn & ~3); // a shift count
            else
            {
                int i = k.value + m_opcode_offset;
                if (i < 0)
                    --i;
                i &= 077777;
                m_opcode_offset = 0;

                if (i & 040000)
                    out.words[0] = i;
                else
                {
                    k = yul::pseudo_to_struct(i);
                    if (k.invalid || !k.address)
                    {
                        out.error = "Incorrect operand type.";
                        out.fatal = true;
                        break;
                    }
                    continue;
                }
            }
        }
        else if (k.address && k.erasable)
            out.words[0] = k.banked ? 0400 * k.eb + (k.sreg - 01400) : k.sreg;
        else if (k.address && k.fixed)
            out.words[0] = k.banked ? 02000 * k.fb + (k.sreg - 02000) : k.sreg;
        else
        {
            out.error = "Incorrect operand type.";
            out.fatal = true;
        }
        break;
    }

    out.words[0] += m_opcode_offset;
    if (m_switch_increment[operand_index])
    {
        out.words[0] = (out.words[0] + 1) & 037777;
        if (m_arg_type == 2)
            out.words[0] = 077777 & ~out.words[0];
    }
}

void yul::c_assembler::parse_index(yul::line_input_t& in, yul::line_output_t& out)
{
    out.extend = in.extend;
    out.pc = yul::inc_pc(in.pc, 1);
    out.ebank = in.ebank;
    out.sbank = in.sbank;
    out.num_words = 1;
    out.index_valid = true;

    yul::address_t offset;
    bool found = true;
    if (const std::optional<int> number = yul::parse_number(in.operand))
    {
        offset = (in.operand[0] == '+' || in.operand[0] == '-') ? yul::inc_pc(in.pc, *number) : yul::pseudo_to_struct(*number);
        if (!in.mod1.empty())
            if (const std::optional<int> value = yul::parse_number(in.mod1))
                m_opcode_offset = *value;
    }
    else if (!fetch_symbol_plus_offset(in.pc, in.operand, in.mod1, offset))
    {
        out.error = std::format("Symbol \"{}\" undefined or offset bad", in.operand);
        out.fatal = true;
        out.words[0] = 050000;
        found = false;
    }

    if (found)
    {
        if (!offset.address && !offset.constant)
        {
            out.error = "Index is not an address.";
            out.fatal = true;
            offset.sreg = 0;
        }
        if ((in.extend && offset.sreg > 07777) || (!in.extend && offset.sreg > 01777))
        {
            out.error = "Index is out of range.";
            out.fatal = true;
            offset.sreg = 0;
        }
        out.words[0] = 050000 + (offset.constant ? offset.value : offset.sreg);
    }

    if (out.words[0] == 050017)
        out.index_valid = false; // RESUME
}

void yul::c_assembler::parse_equals(yul::line_input_t& in, yul::line_output_t& out)
{
    out.pc = in.pc;
    out.ebank = in.ebank;
    out.sbank = in.sbank;
    out.label_value = yul::invalid_address();
    out.num_words = 0;
    out.equals = true;

    if (in.label.empty())
        return;

    if (in.operand.empty() && in.mod1.empty())
    {
        edit_symbol(in.label, in.pc);
        out.label_value = in.pc;
        return;
    }

    yul::address_t label_value = yul::invalid_address();
    if (const std::optional<int> number = yul::parse_number(in.operand))
    {
        int value = *number;
        if (in.operand[0] == '+' || in.operand[0] == '-')
            label_value = yul::inc_pc(in.pc, value);
        else
        {
            value = std::clamp(value, -16383, 32767);
            label_value = {};
            label_value.constant = true;
            label_value.value = value;

            const yul::address_t segmented = yul::pseudo_to_struct(value);
            if (!segmented.invalid)
            {
                label_value = segmented;
                label_value.constant = !(in.op == "ERASE" || (value >= 02000 && in.op == "EQUALS"));
            }
        }
    }
    else
    {
        bool found = false;
        if (in.mod1 == "+" || in.mod1 == "-")
        {
            if (in.mod2.empty())
            {
                out.error = "Syntax error, invalid offset specified";
                out.fatal = true;
                return;
            }
            found = fetch_symbol_plus_offset(in.pc, in.operand, in.mod1 + in.mod2, label_value);
        }
        else
            found = fetch_symbol_plus_offset(in.pc, in.operand, in.mod1, label_value);

        if (!found)
        {
            out.error = std::format("Symbol \"{}\" undefined or offset bad", in.operand);
            out.fatal = true;
            return;
        }

        if (m_opcode_offset)
        {
            if (label_value.constant)
                label_value.value += m_opcode_offset;
            else
                label_value = yul::inc_pc(label_value, m_opcode_offset);
        }
        out.label_value_valid = true;
    }

    edit_symbol(in.label, label_value);
    out.label_value = label_value;
}

void yul::c_assembler::parse_eq_minus(yul::line_input_t& in, yul::line_output_t& out)
{
    out.pc = in.pc;
    out.ebank = in.ebank;
    out.sbank = in.sbank;
    out.num_words = 0;
    out.equals = true;

    if (in.label.empty() || in.operand.empty())
        return;

    yul::address_t label_value = yul::invalid_address();
    if (!fetch_symbol_plus_offset(in.pc, in.operand, in.mod1, label_value))
    {
        out.error = std::format("Symbol \"{}\" undefined or offset bad", in.operand);
        out.fatal = true;
    }
    else
    {
        label_value.invalid = false;
        label_value.constant = true;
        label_value.address = false;
        label_value.erasable = false;
        label_value.fixed = false;
        label_value.value = label_value.sreg - in.pc.sreg;
    }

    edit_symbol(in.label, label_value);
    out.label_value = label_value;
}

void yul::c_assembler::parse_erase(yul::line_input_t& in, yul::line_output_t& out)
{
    out.pc = yul::inc_pc(in.pc, 1);
    out.ebank = in.ebank;
    out.sbank = in.sbank;

    const std::optional<int> number = yul::parse_number(in.operand);
    if (!number)
    {
        if (!in.operand.empty())
        {
            out.error = "Operand is not a number.";
            out.fatal = true;
        }
        return;
    }

    if (*number < 0)
    {
        out.error = "Negative size.";
        out.fatal = true;
        return;
    }

    if (in.mod1 == "-")
    {
        // a fixed range, ERASE first - last: the label is its start, the location counter stays where it was (the
        // Comanche 055 rope shows it, yaYUL would move it to the range)
        const std::optional<int> last = yul::parse_number(in.mod2);
        if (!last || *last + 1 <= *number)
        {
            out.error = "Bad range.";
            out.fatal = true;
        }
        const yul::address_t pc = in.pc;
        parse_equals(in, out);
        out.pc = pc;
        return;
    }

    out.pc = yul::inc_pc(in.pc, 1 + *number);
}

void yul::c_assembler::parse_setloc(yul::line_input_t& in, yul::line_output_t& out)
{
    out.pc = in.pc;
    out.sbank = in.sbank;
    out.extend = in.extend;

    if (const std::optional<int> number = yul::parse_number(in.operand))
        out.pc = (in.operand[0] == '+' || in.operand[0] == '-') ? yul::inc_pc(out.pc, *number) : yul::pseudo_to_struct(*number);
    else if (const yul::address_t* symbol = find_symbol(in.operand))
        out.pc = *symbol;
    else
    {
        out.pc = yul::invalid_address();
        out.error = std::format("Symbol \"{}\" undefined", in.operand);
        out.fatal = true;
    }

    if (const std::optional<int> offset = yul::parse_number(in.mod1))
        out.pc = yul::inc_pc(out.pc, *offset);

    in.pc = out.pc;
    if (!out.pc.invalid && out.pc.erasable)
    {
        out.ebank.current = out.pc;
        out.ebank.last = out.pc;
        out.ebank.oneshot_pending = false;
    }
}

void yul::c_assembler::parse_bank(yul::line_input_t& in, yul::line_output_t& out)
{
    out.extend = in.extend;

    if (in.operand.empty())
    {
        // back to where the current bank's code ends
        out.pc = in.pc;
        if (out.pc.invalid || !out.pc.address || !out.pc.fixed)
            return;

        if (out.pc.banked)
        {
            const int bank = out.pc.fb + 010 * out.pc.super;
            out.pc.sreg = 02000 + m_used_in_bank[static_cast<std::size_t>(bank)];
            out.pc.value = 010000 + bank * 02000 + m_used_in_bank[static_cast<std::size_t>(bank)];
        }
        else if (out.pc.value >= 04000 && out.pc.value <= 05777)
            out.pc.value = out.pc.sreg = 04000 + m_used_in_bank[2];
        else if (out.pc.value >= 06000 && out.pc.value <= 07777)
            out.pc.value = out.pc.sreg = 06000 + m_used_in_bank[3];
        return;
    }

    const std::optional<int> number = yul::parse_number(in.operand);
    if (number && *number >= 0 && *number <= 043)
    {
        const int bank = *number;
        out.pc = {};
        out.pc.address = true;
        out.pc.sreg = 02000;
        out.pc.fixed = true;
        out.pc.banked = true;
        out.pc.super = bank >= 040;
        out.pc.fb = bank >= 040 ? bank - 010 : bank;
        out.pc.value = (bank == 2 || bank == 3) ? bank * 02000 : 010000 + bank * 02000;
        out.pc.value += m_used_in_bank[static_cast<std::size_t>(bank)];
        out.pc.sreg += m_used_in_bank[static_cast<std::size_t>(bank)];
    }
    else
    {
        out.pc = yul::invalid_address();
        out.error = "Bad bank.";
        out.fatal = true;
    }

    in.pc = out.pc;
}

void yul::c_assembler::parse_block(yul::line_input_t& in, yul::line_output_t& out)
{
    const std::optional<int> number = yul::parse_number(in.operand);
    if (number && (*number == 0 || *number == 2 || *number == 3))
    {
        const int used = m_used_in_bank[static_cast<std::size_t>(*number)] + *number * 02000;
        out.pc = {};
        out.pc.address = true;
        out.pc.sreg = used;
        out.pc.erasable = *number == 0;
        out.pc.fixed = *number != 0;
        out.pc.unbanked = true;
        out.pc.value = used;
        if (*number == 0)
            out.ebank.current = out.pc;
    }
    else
    {
        out.pc = yul::invalid_address();
        out.error = "Bad block.";
        out.fatal = true;
    }

    in.pc = out.pc;
}

void yul::c_assembler::parse_ebank_equals(yul::line_input_t& in, yul::line_output_t& out)
{
    out.extend = in.extend;
    out.index_valid = in.index_valid;
    out.ebank = in.ebank;
    out.sbank = in.sbank;
    out.num_words = 0;
    out.pc = in.pc;

    yul::address_t address;
    if (const std::optional<int> number = yul::parse_number(in.operand))
    {
        if (*number < 010 || *number > 03777)
        {
            out.error = "Bad erasable bank.";
            out.fatal = true;
            return;
        }
        address = yul::pseudo_to_struct(*number);
    }
    else if (fetch_symbol_plus_offset(in.pc, in.operand, "", address))
        address = yul::inc_pc(address, m_opcode_offset);
    else
    {
        out.error = std::format("Symbol \"{}\" undefined", in.operand);
        out.fatal = true;
        return;
    }

    if (address.invalid || !address.erasable || address.sreg < 0 || address.sreg > 01777)
    {
        if (!address.invalid)
        {
            out.error = "Not an erasable address.";
            out.fatal = true;
        }
        return;
    }

    out.ebank.last = out.ebank.current;
    out.ebank.current = address;
    out.ebank.oneshot_pending = true;
    out.label_value = address;
    out.label_value_valid = true;
}

void yul::c_assembler::parse_sbank_equals(yul::line_input_t& in, yul::line_output_t& out)
{
    out.ebank = in.ebank;
    out.sbank = in.sbank;
    out.num_words = 0;
    out.pc = in.pc;

    if (in.operand.empty())
        return;

    yul::address_t address;
    if (const std::optional<int> number = yul::parse_number(in.operand))
        address = yul::pseudo_to_struct(*number);
    else if (fetch_symbol_plus_offset(in.pc, in.operand, "", address))
        address = yul::inc_pc(address, m_opcode_offset);
    else
    {
        out.error = std::format("Symbol \"{}\" undefined", in.operand);
        out.fatal = true;
        return;
    }

    if (address.invalid || !address.fixed || address.sreg < 02000 || address.sreg > 03777)
        return;

    if (address.fb >= 030 && address.fb <= 037)
    {
        out.sbank.last = out.sbank.current;
        out.sbank.current = address.super ? 4 : 3;
    }
    else
    {
        out.sbank.current = 0;
        return;
    }

    out.sbank.oneshot_pending = true;
    out.label_value = address;
    out.label_value_valid = true;
}

void yul::c_assembler::parse_cadr(yul::line_input_t& in, yul::line_output_t& out)
{
    out.pc = yul::inc_pc(in.pc, 1);
    out.ebank = in.ebank;
    out.sbank = in.sbank;
    out.num_words = 1;
    out.words[0] = 0;

    yul::address_t address;
    const std::optional<int> number = yul::parse_number(in.operand);
    if (number && in.mod1.empty())
        address = yul::pseudo_to_struct(*number);
    else if (!fetch_symbol_plus_offset(in.pc, in.operand, in.mod1 + in.mod2, address))
    {
        out.error = std::format("Symbol \"{}\" undefined", in.operand);
        out.fatal = true;
        return;
    }

    if (address.invalid || !address.address || !address.fixed || !address.banked)
    {
        out.error = "Destination is not in banked fixed memory.";
        out.fatal = true;
        return;
    }

    if (address.super && address.fb >= 030)
        address.value -= 010 * 02000;
    if (address.value < 010000 || address.value > 0107777)
    {
        out.error = "Destination address out of range.";
        out.fatal = true;
        return;
    }

    out.words[0] = address.value - 010000;
}

void yul::c_assembler::parse_2cadr(yul::line_input_t& in, yul::line_output_t& out)
{
    out.pc = yul::inc_pc(in.pc, 2);
    out.ebank = in.ebank;
    out.sbank = in.sbank;
    out.num_words = 2;
    out.words = { 0, 0 };

    yul::address_t address;
    if (!fetch_symbol_plus_offset(in.pc, in.operand, in.mod1, address))
    {
        out.error = std::format("Symbol \"{}\" undefined", in.operand);
        out.fatal = true;
        return;
    }
    if (address.invalid || !address.address)
    {
        out.error = "Destination not resolved.";
        out.fatal = true;
        return;
    }

    out.words[0] = address.sreg;
    const yul::address_t& ebank = in.ebank.current;
    out.words[1] = (ebank.sreg >= 0 && ebank.sreg < 01400) ? ebank.sreg / 0400 : ebank.eb;

    if (address.sreg >= 02000 && address.sreg <= 07777)
        out.words[1] |= (address.sreg < 04000 ? address.fb : address.sreg / 02000) << 10;
    else if (address.sreg > 07777)
    {
        out.error = "Destination out of range.";
        out.fatal = true;
        return;
    }

    if (address.fb >= 030 && address.fb <= 037)
        out.words[1] |= (address.super ? 4 : 3) << 4;
    else
        out.words[1] |= static_cast<int>(in.sbank.current) << 4;
}

void yul::c_assembler::parse_ecadr(yul::line_input_t& in, yul::line_output_t& out)
{
    out.pc = yul::inc_pc(in.pc, 1);
    out.ebank = in.ebank;
    out.sbank = in.sbank;
    out.num_words = 1;
    out.words[0] = 0;

    yul::address_t address;
    const std::optional<int> number = yul::parse_number(in.operand);
    if (number && in.mod1.empty())
        address = yul::inc_pc(in.pc, *number);
    else if (!fetch_symbol_plus_offset(in.pc, in.operand, in.mod1, address))
    {
        out.error = std::format("Symbol \"{}\" undefined", in.operand);
        out.fatal = true;
        return;
    }

    if (address.invalid)
    {
        out.error = "Destination not resolved.";
        out.fatal = true;
        return;
    }

    if (address.constant && !address.address)
    {
        if (address.value < 01400)
        {
            address.constant = false;
            address.address = true;
            address.erasable = true;
            address.banked = false;
            address.sreg = address.value;
        }
        else if (address.value < 04000)
        {
            address.constant = false;
            address.address = true;
            address.erasable = true;
            address.banked = true;
            address.eb = address.value / 0400;
            address.sreg = (address.value % 0400) + 01400;
        }
    }

    if (!address.address || !address.erasable || address.value < 0 || address.value > 03777)
    {
        out.error = "Not an erasable address.";
        out.fatal = true;
        return;
    }

    out.words[0] = address.value;
}

void yul::c_assembler::parse_genadr(yul::line_input_t& in, yul::line_output_t& out)
{
    out.pc = yul::inc_pc(in.pc, 1);
    out.ebank = in.ebank;
    out.sbank = in.sbank;
    out.num_words = 1;

    yul::address_t address;
    const std::optional<int> number = yul::parse_number(in.operand);
    if (number && in.mod1.empty())
        address = yul::inc_pc(in.pc, *number);
    else if (!fetch_symbol_plus_offset(in.pc, in.operand, in.mod1, address))
    {
        out.error = std::format("Symbol \"{}\" undefined", in.operand);
        out.fatal = true;
        out.words[0] = 0;
        return;
    }

    if (address.invalid)
    {
        out.error = "Destination not resolved.";
        out.fatal = true;
        return;
    }
    if (address.constant)
        address = yul::pseudo_to_struct(address.value);
    if (!address.address)
    {
        out.error = "Destination not an address.";
        out.fatal = true;
        return;
    }

    out.words[0] = address.sreg;
}

void yul::c_assembler::parse_bbcon(yul::line_input_t& in, yul::line_output_t& out, bool star)
{
    out.pc = yul::inc_pc(in.pc, 1);
    out.ebank = in.ebank;
    out.sbank = in.sbank;
    out.num_words = 1;
    out.words[0] = 0;

    yul::address_t address;
    const std::optional<int> number = yul::parse_number(in.operand);
    if (star)
    {
        // the end of the last bank that has anything in it, as of the last pass
        int bank = 043, highest = 0;
        for (; bank >= 0; --bank)
        {
            highest = m_prior_used_in_bank[static_cast<std::size_t>(bank)];
            if (highest > 0)
                break;
        }
        address.address = true;
        address.banked = true;
        address.fixed = true;
        address.fb = bank >= 040 ? bank - 010 : bank;
        address.super = bank >= 040;
        address.sreg = (04000 + highest) & 07777;
        address.value = 010000 + (address.sreg - 02000) + 02000 * address.fb + ((address.super && address.fb >= 030) ? 020000 : 0);
    }
    else if (number && in.mod1.empty())
        address = yul::inc_pc(in.pc, *number);
    else if (!fetch_symbol_plus_offset(in.pc, in.operand, in.mod1, address))
    {
        out.error = std::format("Symbol \"{}\" undefined", in.operand);
        out.fatal = true;
        return;
    }

    if (address.invalid || !address.address || !address.fixed || address.sreg < 02000 || address.sreg > 07777)
    {
        out.error = "Destination not in fixed memory.";
        out.fatal = true;
        return;
    }

    int value = address.banked ? address.fb : address.sreg / 02000;
    yul::address_t ebank = in.ebank.current;
    if (ebank.sreg >= 0 && ebank.sreg < 01400)
        ebank.eb = ebank.sreg / 0400;
    value = (value << 10) | ebank.eb;

    if (address.fb >= 030 && address.fb <= 037)
        value |= (address.super ? 4 : 3) << 4;
    else
        value |= static_cast<int>(in.sbank.current) << 4;

    out.words[0] = value;
}

void yul::c_assembler::parse_oct(yul::line_input_t& in, yul::line_output_t& out)
{
    out.pc = yul::inc_pc(in.pc, 1);
    out.ebank = in.ebank;
    out.sbank = in.sbank;
    out.words[0] = yul::illegal_word;
    out.num_words = 1;

    const std::string_view text = in.operand;
    const bool minus = !text.empty() && text[0] == '-';
    const std::string_view digits = text.substr(!text.empty() && (text[0] == '+' || text[0] == '-') ? 1 : 0);
    if (digits.empty() || digits.find_first_not_of("01234567") != std::string_view::npos)
    {
        out.error = "Not an octal number.";
        out.fatal = true;
        return;
    }

    int value = 0;
    for (const char c : digits)
        value = value * 8 + (c - '0');
    value &= 077777;
    out.words[0] = minus ? ~value : value;
}

void yul::c_assembler::parse_2oct(yul::line_input_t& in, yul::line_output_t& out)
{
    out.pc = yul::inc_pc(in.pc, 2);
    out.ebank = in.ebank;
    out.sbank = in.sbank;
    out.words = { yul::illegal_word, yul::illegal_word };
    out.num_words = 2;

    const auto octal = [](std::string_view digits) -> std::optional<long long> {
        if (digits.empty() || digits.find_first_not_of("01234567") != std::string_view::npos)
            return std::nullopt;
        long long value = 0;
        for (const char c : digits)
            value = value * 8 + (c - '0');
        return value;
    };

    const std::string_view text = in.operand;
    const bool minus = !text.empty() && text[0] == '-';
    const std::optional<long long> high = octal(text.substr(!text.empty() && (text[0] == '+' || text[0] == '-') ? 1 : 0));
    if (!high)
    {
        out.error = "Not an octal number.";
        out.fatal = true;
        return;
    }

    long long value = *high & 07777777777;
    if (const std::optional<long long> low = octal(in.mod1))
        value = ((*high & 077777) << 15) | (*low & 077777);
    if (minus)
        value = ~value;

    out.words[0] = static_cast<int>(077777 & (value >> 15));
    out.words[1] = static_cast<int>(077777 & value);
}

void yul::c_assembler::parse_dec(yul::line_input_t& in, yul::line_output_t& out)
{
    out.pc = yul::inc_pc(in.pc, 1);
    out.ebank = in.ebank;
    out.sbank = in.sbank;
    out.words[0] = yul::illegal_word;
    out.num_words = 1;

    if (in.operand.empty())
    {
        out.error = "Operand is missing.";
        out.fatal = true;
        return;
    }

    double mantissa = 1.0;
    std::string scale1, scale2;
    if (in.operand[0] == 'E' || in.operand[0] == 'B')
    {
        scale1 = in.operand;
        scale2 = in.mod1;
    }
    else
    {
        mantissa = std::strtod(in.operand.c_str(), nullptr);
        scale1 = in.mod1;
        scale2 = in.mod2;
    }

    double x = mantissa * scale_factor(scale1) * scale_factor(scale2);
    const bool negative = in.operand[0] == '-';
    if (negative)
        x = -x;

    int value = 0;
    if (fuzzed_is_int(x))
        value = static_cast<int>(x);
    else
    {
        if (x >= 1.0)
            return;

        // a fraction, to 14 bits, rounded
        for (int i = 0; i < 14; ++i)
        {
            value <<= 1;
            if (x >= 0.5)
            {
                ++value;
                x -= 0.5;
            }
            x *= 2;
        }
        if (x >= 0.5 && value < 0x3fff)
            ++value;
    }

    if (negative)
        value = 0x7fff & ~value;
    out.words[0] = value;
}

void yul::c_assembler::parse_2dec(yul::line_input_t& in, yul::line_output_t& out)
{
    out.pc = yul::inc_pc(in.pc, 2);
    out.ebank = in.ebank;
    out.sbank = in.sbank;
    out.words = { yul::illegal_word, yul::illegal_word };
    out.num_words = 2;

    if (in.operand.empty())
    {
        out.error = "Operand is missing.";
        out.fatal = true;
        return;
    }

    // the scale factors can be written into the number (5.0E-3B2) or after it, in one field or split in two
    double mantissa = 1.0;
    std::string scale1, scale2;
    std::string operand = in.operand;
    if (operand[0] == 'E' || operand[0] == 'B')
    {
        scale1 = operand;
        scale2 = in.mod1;
    }
    else
    {
        const std::size_t e = operand.find('E');
        const std::size_t b = operand.find('B');
        if (e != std::string::npos || b != std::string::npos)
        {
            if (e != std::string::npos && b != std::string::npos)
            {
                scale1 = operand.substr(e, b > e ? b - e : std::string::npos);
                scale2 = operand.substr(b, e > b ? e - b : std::string::npos);
                operand.resize(std::min(e, b));
            }
            else
            {
                scale1 = operand.substr(std::min(e, b));
                operand.resize(std::min(e, b));
            }
        }
        else if (!in.extra.empty())
        {
            if (in.mod1 == "E" || in.mod1 == "B")
            {
                scale1 = in.mod1 + in.mod2;
                scale2 = in.extra;
            }
            else
            {
                scale1 = in.mod1;
                scale2 = in.mod2 + in.extra;
            }
        }
        else if (in.mod1 == "E" || in.mod1 == "B")
            scale1 = in.mod1 + in.mod2;
        else
        {
            scale1 = in.mod1;
            scale2 = in.mod2;
        }
        mantissa = std::strtod(operand.c_str(), nullptr);
    }

    double x = mantissa * scale_factor(scale1) * scale_factor(scale2);
    const bool negative = operand[0] == '-';
    if (negative)
        x = -x;

    int value = 0;
    if (fuzzed_is_int(x))
        value = static_cast<int>(x);
    else
    {
        if (x >= 1.0)
            return;

        for (int i = 0; i < 28; ++i)
        {
            value <<= 1;
            if (x >= 0.5)
            {
                ++value;
                x -= 0.5;
            }
            x *= 2;
        }
        if (x >= 0.5 && value < 0x0fffffff)
            ++value;
    }

    int low = value & 0x3fff;
    int high = (value >> 14) & 0x3fff;
    if (negative)
    {
        high = ~high & 0x7fff;
        low = ~low & 0x7fff;
    }
    out.words = { high, low };
}

void yul::c_assembler::parse_vn(yul::line_input_t& in, yul::line_output_t& out)
{
    out.pc = yul::inc_pc(in.pc, 1);
    out.ebank = in.ebank;
    out.sbank = in.sbank;
    out.words[0] = yul::illegal_word;
    out.num_words = 1;

    if (in.operand.empty() || in.operand.find_first_not_of("0123456789") != std::string::npos)
    {
        out.error = "Operand is not a decimal number.";
        out.fatal = true;
        return;
    }

    const int value = std::atoi(in.operand.c_str());
    out.words[0] = (value % 100) | ((value / 100) << 7);
}

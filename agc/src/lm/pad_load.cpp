#include "lm/pad_load.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <format>
#include <optional>
#include <string>
#include <string_view>

#include "agc/computer.hpp"
#include "yul/assembler.hpp"

// Apollo 11's lunar module erasable load, from the Grumman LM Data Book (LED-540-54, volume II, table LM5/4.5.1-1,
// "G prelaunch erasable load (LUMINARY 99)", amendments to 7/11/69), as scanned by the Virtual AGC project. a row is
// what the table has: the mnemonic (as the source spells it, with the table's +n for the words after a symbol), the
// address, the octal words, and the value they stand for at its scale factor (a fraction of 2^scale). the value is
// there to check the octal against, so a slip in either shows up; rows that are octal to begin with have none.
//
// left out, on purpose:
// - the imu compensation on the launch tape (PBIASX-Z, PIPASCFX-Z, NBDX-Z, ADIAX-Z, ADSRAX-Z): the biases, scale
//   factor errors and drifts of LM-5's own imu. this imu has none, and the computer compensating for them would
//   make some
// - "mission time computed constants, such as state vectors" aren't in the table: the ground uplinked those in
//   flight, and the scenario that starts the landing loads them
struct pad_row_t
{
    std::string_view mnemonic;
    int address;                          // the table's, octal
    std::array<std::uint16_t, 3> words;   // count of them
    int count;
    std::optional<double> value;          // at 2^scale; none for octal rows
    int scale = 0;
    bool twos_complement = false;         // the table marks these "(2's comp)": angles, one word
};

#define PAD1(mnemonic, address, w0, value, scale) { mnemonic, address, { w0, 0, 0 }, 1, value, scale }
#define PAD2(mnemonic, address, w0, w1, value, scale) { mnemonic, address, { w0, w1, 0 }, 2, value, scale }
#define OCT1(mnemonic, address, w0) { mnemonic, address, { w0, 0, 0 }, 1, std::nullopt }
#define OCT2(mnemonic, address, w0, w1) { mnemonic, address, { w0, w1, 0 }, 2, std::nullopt }
#define ANGLE(mnemonic, address, w0, value) { mnemonic, address, { w0, 0, 0 }, 1, value, -1, true }

static const pad_row_t pad_rows[] = {
    // ---- mission tape
    OCT1("FLAGWRD3", 00077, 002000),
    OCT1("FLAGWRD8", 00104, 000000),
    OCT1("FLGWRD10", 00106, 000000),
    PAD1("DUMPCNT", 00333, 010000, 2.0, 3),                              // 2 dumps
    PAD1("E3J22R2M", 01352, 012160, 92.0479047931e15, 58),               // m^5/cs^2
    PAD1("E32C31RM", 01353, 003363, 13.1289255968e22, 80),               // m^6/cs^2
    PAD2("RADSKAL", 01354, 000000, 000000, 0.0, 7),                      // the table's RADSCALE
    PAD1("SKALSKAL", 01356, 000000, 0.0, 0),
    PAD1("GCOMPSW", 01477, 000000, 0.0, 0),
    OCT1("TETCSM", 01570, 037777),                                       // to inhibit initial P00 integration
    OCT1("TETLEM", 01642, 037777),                                       // to inhibit initial P00 integration
    PAD2("X789", 01700, 000000, 000000, 0.0, 0),                         // radians
    PAD2("X789+2", 01702, 000000, 000000, 0.0, 0),
    PAD2("X789+4", 01704, 000000, 000000, 0.0, 0),
    PAD2("REFSMMAT", 01733, 012704, 006264, 0.68020001, 1),              // rev 2 of the mission tape
    PAD2("RANGEVAR", 01770, 001351, 024734, 0.1111111111e-4, -12),
    PAD2("RATEVAR", 01772, 002354, 004750, 1.877777e-5, -12),
    PAD1("RVARMIN", 01774, 000410, 66.0, 12),                            // m^2
    PAD1("VVARMIN", 01775, 000165, 0.17445e-5, -12),                     // m^2/cs^2
    PAD1("WRENDPOS", 02000, 005750, 3048.0, 14),                         // m, 10000 ft
    PAD1("WRENDVEL", 02001, 000763, 0.03048, 0),                         // m/cs, 10 ft/s
    PAD1("WSHAFT", 02002, 017270, 0.015, -5),                            // radians
    PAD1("WTRUN", 02003, 017270, 0.015, -5),
    PAD1("RMAX", 02004, 000023, 609.6, 19),                              // m, 2000 ft
    PAD1("VMAX", 02005, 000001, 0.006096, 7),                            // m/cs, 2 ft/s
    PAD1("WSURFPOS", 02006, 037200, 16000.0, 14),                        // m
    PAD1("WSURFVEL", 02007, 000754, 0.03, 0),                            // m/cs
    PAD1("SHAFTVAR", 02010, 000103, 1e-6, -12),                          // rad^2
    PAD1("TRUNVAR", 02011, 000103, 1e-6, -12),
    PAD2("AGSK", 02020, 003671, 021200, 32400000.0, 28),                 // cs, 90 hr
    PAD2("TLAND", 02400, 004247, 034030, 36304920.0, 28),                // cs, 100:50:49.20 GET
    // the braking phase's targets
    PAD2("RBRFG", 02402, 000000, 001506, 52.37530800, 24),               // m
    PAD2("RBRFG+2", 02404, 000000, 000000, 0.0, 24),
    PAD2("RBRFG+4", 02406, 077774, 072222, -3254.836061, 24),
    PAD2("VBRFG", 02410, 077772, 072612, -0.3227100480, 10),             // m/cs
    PAD2("VBRFG+2", 02412, 000000, 000000, 0.0, 10),
    PAD2("VBRFG+4", 02414, 077777, 076300, -0.0031699200, 10),
    PAD2("ABRFG", 02416, 000004, 037445, 0.000019022568, -4),            // m/cs^2
    PAD2("ABRFG+2", 02420, 000000, 000000, 0.0, -4),
    PAD2("ABRFG+4", 02422, 077667, 050111, -0.000277502112, -4),
    PAD2("VBRFG*", 02424, 077777, 074261, -0.0570585600, 13),
    PAD2("ABRFG*", 02426, 077113, 060670, -0.001665012672, -4),
    PAD2("JBRFG*", 02430, 077472, 072437, -0.5738399496e-8, -21),        // m/cs^3
    PAD2("GAINBRAK", 02432, 037777, 037777, 1.0, 0),
    PAD1("TCGFBRAK", 02434, 000567, 3000.0, 17),                         // cs
    PAD1("TCGIBRAK", 02435, 025762, 90000.0, 17),
    // the approach phase's
    PAD2("RAPFG", 02436, 000000, 001036, 33.85870800, 24),               // m
    PAD2("RAPFG+2", 02440, 000000, 000000, 0.0, 24),
    PAD2("RAPFG+4", 02442, 077777, 077574, -8.166811200, 24),
    PAD2("VAPFG", 02444, 077777, 070152, -0.0152186640, 10),             // m/cs
    PAD2("VAPFG+2", 02446, 000000, 000000, 0.0, 10),
    PAD2("VAPFG+4", 02450, 000000, 000306, 0.0007559040, 10),
    PAD2("AAPFG", 02452, 077775, 074720, -0.07997952e-4, -4),            // m/cs^2
    PAD2("AAPFG+2", 02454, 000000, 000000, 0.0, -4),
    PAD2("AAPFG+4", 02456, 077773, 075055, -0.1560576e-4, -4),
    PAD2("VAPFG*", 02460, 000000, 000676, 0.0136062720, 13),
    PAD2("AAPFG*", 02462, 077747, 056422, -0.9363456e-4, -4),
    PAD2("JAPFG*", 02464, 000022, 035646, 0.5509930560e-9, -21),         // m/cs^3
    PAD2("GAINAPPR", 02466, 000000, 000000, 0.0, 0),
    PAD1("TCGFAPPR", 02470, 000567, 3000.0, 17),                         // cs
    PAD1("TCGIAPPR", 02471, 004704, 20000.0, 17),
    // the ignition algorithm
    PAD2("VIGN", 02472, 000416, 016071, 16.90256208, 10),                // m/cs
    PAD2("RIGNX", 02474, 077731, 044630, -39782.453328, 24),             // m
    PAD2("RIGNZ", 02476, 077125, 062404, -436655.657, 24),
    PAD2("KIGNX/B4", 02500, 076607, 061356, -0.617631, 4),
    PAD2("KIGNY/B8", 02502, 072634, 051602, -2.4770341207e-6, -16),      // 1/m
    PAD2("KIGNV/B4", 02504, 072775, 057777, -41000.0, 18),               // cs
    // the throttle's region to stay out of, in pulses: 5985 and 6615 lbf
    PAD1("LOWCRIT", 02506, 004114, 2124.4, 14),
    PAD1("HIGHCRIT", 02507, 004454, 2348.0, 14),
    PAD2("V2FG", 02510, 077777, 073242, -0.009144, 10),                  // m/cs, -3 ft/s
    PAD2("V2FG+2", 02512, 000000, 000000, 0.0, 10),
    PAD2("V2FG+4", 02514, 000000, 000000, 0.0, 10),
    PAD2("TAUVERT", 02516, 001750, 000000, 1000.0, 14),                  // cs; the table shows the first word
    PAD2("DELQFIX", 02520, 000000, 001717, 60.96, 24),                   // m, 200 ft
    // the landing radar: antenna positions, weights
    PAD1("LRALPHA", 02522, 001027, 0.0163371759, -1),                    // rev, stow position; the table's LRALPHA1
    PAD1("LRBETA1", 02523, 004204, 0.0665287037, -1),
    PAD1("LRALPHA2", 02524, 001022, 0.0161680555, -1),                   // hover position
    PAD1("LRBETA2", 02525, 000004, 0.0001361111, -1),
    PAD1("LRVMAX", 02526, 001414, 6.096, 7),                             // m/cs, 2000 ft/s
    PAD1("LRVF", 02527, 000116, 0.6096, 7),                              // m/cs, 200 ft/s
    PAD1("LRWVZ", 02530, 011463, 0.3, 0),
    PAD1("LRWVY", 02531, 011463, 0.3, 0),
    PAD1("LRWVX", 02532, 011463, 0.3, 0),
    PAD1("LRWVFZ", 02533, 006315, 0.2, 0),
    PAD1("LRWVFY", 02534, 006315, 0.2, 0),
    PAD1("LRWVFX", 02535, 006315, 0.2, 0),
    PAD1("LRWVFF", 02536, 003146, 0.1, 0),
    PAD1("RODSCALE", 02537, 014370, 0.003048, -7),                       // m/cs, 1 ft/s
    PAD2("TAUROD", 02540, 011300, 000000, 150.0, 9),                     // cs
    PAD2("LAG/TAU", 02542, 015164, 001420, 0.413333, 0),
    PAD2("MINFORCE", 02544, 000001, 027631, 0.4359257183, 12),           // kg m/cs^2, 980 lb
    PAD2("MAXFORCE", 02546, 000013, 006551, 2.802379618, 12),            // 6300 lb
    // abort targeting: descent, then ascent
    PAD2("ABTCOF", 02550, 001311, 021176, 2.4755856e-15, -44),
    PAD2("ABTCOF+2", 02552, 076401, 047023, -0.3486912e-9, -27),
    PAD2("ABTCOF+4", 02554, 000132, 026223, 0.054059328e-4, -10),
    PAD2("ABTCOF+6", 02556, 004231, 005721, 17.19675504, 7),
    PAD2("ABTCOF+8", 02560, 001241, 032430, 2.337816e-15, -44),
    PAD2("ABTCOF+10", 02562, 076470, 045634, -0.3236976e-9, -27),
    PAD2("ABTCOF+12", 02564, 000106, 022276, 0.042065448e-4, -10),
    PAD2("ABTCOF+14", 02566, 004232, 010573, 17.20525896, 7),
    PAD2("VMIN", 02570, 004147, 024472, 16.80972, 7),                    // m/cs
    PAD2("YLIM", 02572, 000016, 032446, 15186.4, 24),                    // m, 8.2 nmi
    PAD2("ABTRDOT", 02574, 000007, 023346, 0.059436, 7),                 // m/cs
    PAD2("COSTHET1", 02576, 000000, 000000, 0.0, -8),
    PAD2("COSTHET2", 02600, 006733, 007535, 0.8660254037, 2),
    // R03's trim: from the stop back to trim, cs
    PAD1("ROLLTIME", 03001, 005556, 2926.0, 14),
    PAD1("PITTIME", 03002, 004431, 2329.0, 14),
    // the autopilot
    PAD1("DKTRAP", 03003, 077001, -0.0038888888, -3),                    // rev/s
    PAD1("DKOMEGAN", 03004, 000012, 10.0, 14),
    PAD1("DKKAOSN", 03005, 000074, 60.0, 14),
    PAD1("LMTRAP", 03006, 077001, -0.0038888888, -3),
    PAD1("LMOMEGAN", 03007, 000000, 0.0, 14),
    PAD1("LMKAOSN", 03010, 000074, 60.0, 14),
    PAD1("DKDB", 03011, 000200, 256.0, 15),                              // 1/rev
    PAD1("IGNAOSQ", 03012, 002614, 0.0216666667, -2),                    // rev/s^2, 7.8 deg/s^2
    PAD1("IGNAOSR", 03013, 000062, 0.0007694444, -2),                    // 0.277 deg/s^2
    PAD1("DOWNTORK", 03113, 000000, 0.0, 5),                             // jet seconds
    PAD1("DOWNTORK+1", 03114, 000000, 0.0, 5),
    PAD1("DOWNTORK+2", 03115, 000000, 0.0, 5),
    PAD1("DOWNTORK+3", 03116, 000000, 0.0, 5),
    PAD1("DOWNTORK+4", 03117, 000000, 0.0, 5),
    PAD1("DOWNTORK+5", 03120, 000000, 0.0, 5),
    PAD2("ATIGINC", 03400, 000001, 003120, 18000.0, 28),                 // cs, 3 min
    PAD2("PTIGINC", 03402, 000001, 003120, 18000.0, 28),
    // the alignment optical telescope's detent positions, rev
    ANGLE("AOTAZ", 03404, 065244, -0.1668666667),                        // -60.072 deg
    ANGLE("AOTAZ+1", 03405, 077762, -0.0004277778),                      // -0.154 deg
    PAD1("AOTAZ+2", 03406, 012476, 0.1659444444, -1),                    // 59.740 deg
    PAD1("AOTAZ+3", 03407, 025247, 0.3332305556, -1),                    // 119.963 deg
    PAD1("AOTAZ+4", 03410, 037771, 0.4997944444, -1),                    // 179.926 deg
    ANGLE("AOTAZ+5", 03411, 052510, -0.3337527778),                      // -120.151 deg
    PAD1("AOTEL", 03412, 007777, 0.1249805556, -1),                      // 44.993 deg
    PAD1("AOTEL+1", 03413, 010006, 0.1251694444, -1),                    // 45.061 deg
    PAD1("AOTEL+2", 03414, 010006, 0.1251750000, -1),                    // 45.063 deg
    PAD1("AOTEL+3", 03415, 010000, 0.1249888889, -1),                    // 44.996 deg
    PAD1("AOTEL+4", 03416, 007772, 0.1248027778, -1),                    // 44.929 deg
    PAD1("AOTEL+5", 03417, 007771, 0.1247972222, -1),                    // 44.927 deg
    PAD1("LRHMAX", 03420, 035610, 15240.0, 14),                          // m, 50000 ft
    PAD1("LRWH", 03421, 013146, 0.35, 0),                                // the data book's scale says -1, its octal and the
                                                                         // 7/1/69 mission tape's scale 0
    PAD1("ZOOMTIME", 03422, 005050, 2600.0, 14),                         // cs, 26 s
    PAD1("TENDBRAK", 03423, 001407, 6200.0, 17),                         // cs
    PAD1("TENDAPPR", 03424, 000226, 1200.0, 17),
    PAD1("DELTTFAP", 03425, 075240, -11000.0, 17),
    PAD1("LEADTIME", 03426, 077743, -220.0, 17),                         // 2.2 s (negative for coding ease)
    PAD1("RPCRTIME", 03427, 001407, 6200.0, 17),
    PAD1("RPCRTQSW", 03430, 057777, -1.0, 1),
    OCT2("TNEWA", 03431, 020000, 000000),                                // about 2 weeks: Lambert doesn't recycle

    // ---- launch tape
    PAD2("MASS", 01244, 007351, 015463, 15269.7, 16),                    // kg, 33664 lb at DOI ignition
    PAD1("LEMMASS", 01331, 007351, 15269.7, 16),
    PAD1("CSMMASS", 01332, 010050, 16542.7, 16),                         // 36470.4 lb
    { "TEPHEM", 01706, { 000000, 020017, 020500 }, 3, 134472000.0, 42 },  // cs, for launch on July 16 1969 at 13:32 GMT
    // the moon's orientation: its libration angles, and the matrix the landing site is carried by
    PAD2("AZO", 01711, 030624, 001636, 0.7746616521, 0),                 // rev
    PAD2("-AYO", 01713, 077777, 053172, -0.0000395943, 0),
    PAD2("AXO", 01715, 000000, 026056, 0.0000421325, 0),
    PAD2("REFSMMAT+2", 01735, 012562, 010723, 0.67020001, 1),            // rev 2 of the launch tape
    PAD2("REFSMMAT+4", 01737, 001112, 025001, 0.07161332, 1),
    OCT2("504LM", 02012, 077775, 046355),
    OCT2("504LM+2", 02014, 077766, 072372),
    OCT2("504LM+4", 02016, 077777, 052552),
    // the landing site, moon fixed
    PAD2("RLS", 02022, 000301, 034760, 1588471.994, 27),                 // m
    PAD2("RLS+2", 02024, 000125, 004627, 697547.4954, 27),
    PAD2("RLS+4", 02026, 000002, 024342, 21616.9998, 27),
    PAD1("HIASCENT", 03000, 002350, 5023.6, 16),                         // kg, 11075.1 lb
};

#undef PAD1
#undef PAD2
#undef OCT1
#undef OCT2
#undef ANGLE

// a mnemonic's symbol and the words after it: "RBRFG+4" is RBRFG and 4
static std::pair<std::string, int> split_mnemonic(std::string_view mnemonic)
{
    const std::size_t plus = mnemonic.rfind('+');
    if (plus == std::string_view::npos || plus == 0)
        return { std::string(mnemonic), 0 };
    return { std::string(mnemonic.substr(0, plus)), std::stoi(std::string(mnemonic.substr(plus + 1))) };
}

// the words a value is stored as: a fraction of 2^scale, 14 bits and a sign a word, in ones' complement (each word of
// a multiple precision value takes the sign), or one word in two's complement. past the largest fraction it saturates
static std::array<std::uint16_t, 3> encode(double value, int scale, int count, bool twos_complement)
{
    const double fraction = value / std::ldexp(1.0, scale);
    const long long full = 1ll << (14 * count);
    const long long magnitude = std::min(std::llround(std::abs(fraction) * static_cast<double>(full)), full - 1);
    const bool negative = std::signbit(fraction) && magnitude != 0;

    std::array<std::uint16_t, 3> words{};
    if (twos_complement)
    {
        words[0] = static_cast<std::uint16_t>((negative ? 0100000 - magnitude : magnitude) & 077777);
        return words;
    }
    for (int i = 0; i < count; ++i)
    {
        const auto word = static_cast<std::uint16_t>((magnitude >> (14 * (count - 1 - i))) & 037777);
        words[static_cast<std::size_t>(i)] = negative ? static_cast<std::uint16_t>(~word & 077777) : word;
    }
    return words;
}

// the words as one signed number, to compare two encodings a unit apart (the table's rounding isn't always ours)
static long long as_number(const std::array<std::uint16_t, 3>& words, int count, bool twos_complement)
{
    if (twos_complement)
        return words[0] & 040000 ? static_cast<long long>(words[0]) - 0100000 : words[0];
    const bool negative = (words[0] & 040000) != 0;
    long long number = 0;
    for (int i = 0; i < count; ++i)
        number = (number << 14) | ((negative ? ~words[static_cast<std::size_t>(i)] : words[static_cast<std::size_t>(i)]) & 037777);
    return negative ? -number : number;
}

std::vector<std::string> lm::check_pad(const yul::assembly_t& assembly)
{
    std::vector<std::string> problems;
    for (const pad_row_t& row : pad_rows)
    {
        const auto [name, offset] = split_mnemonic(row.mnemonic);
        const auto symbol = assembly.symbols.find(name);
        if (symbol == assembly.symbols.end())
            problems.push_back(std::format("{}: no symbol {}", row.mnemonic, name));
        else if (symbol->second.value + offset != row.address)
            problems.push_back(std::format("{}: at {:o} in the table, {:o} in the program", row.mnemonic, row.address, symbol->second.value + offset));

        if (!row.value)
            continue;
        const auto expected = encode(*row.value, row.scale, row.count, row.twos_complement);
        const long long difference = as_number(row.words, row.count, row.twos_complement) - as_number(expected, row.count, row.twos_complement);
        if (std::abs(difference) > 1)
        {
            std::string theirs, ours;
            for (int i = 0; i < row.count; ++i)
            {
                theirs += std::format(" {:05o}", row.words[static_cast<std::size_t>(i)]);
                ours += std::format(" {:05o}", expected[static_cast<std::size_t>(i)]);
            }
            problems.push_back(std::format("{}: the table's{} isn't {} at 2^{} ({})", row.mnemonic, theirs, *row.value, row.scale, ours));
        }
    }
    return problems;
}

std::vector<std::string> lm::load_pad(agc::c_computer& computer, const yul::assembly_t& assembly)
{
    std::vector<std::string> problems;
    for (const pad_row_t& row : pad_rows)
    {
        // where the program has it: the table's address is checked against it in check_pad
        const auto [name, offset] = split_mnemonic(row.mnemonic);
        const auto symbol = assembly.symbols.find(name);
        if (symbol == assembly.symbols.end() || symbol->second.value + offset >= 04000)
        {
            problems.push_back(std::format("no erasable symbol {}", name));
            continue;
        }
        for (int i = 0; i < row.count; ++i)
            computer.load_erasable(symbol->second.value + offset + i, row.words[static_cast<std::size_t>(i)]);
    }
    return problems;
}

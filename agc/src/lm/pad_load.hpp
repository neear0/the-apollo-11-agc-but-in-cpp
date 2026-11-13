#pragma once

#include <string>
#include <vector>

namespace agc
{
    class c_computer;
}

namespace yul
{
    struct assembly_t;
}

// the erasable words the ground loaded into the computer before launch, the ones fresh start leaves alone. power-on
// clears erasable memory, so without them the programs that read them see zeros
namespace lm
{
    // loads them by the program's symbols, after power-on. the symbols it didn't find
    std::vector<std::string> load_pad(agc::c_computer& computer, const yul::assembly_t& assembly);
}

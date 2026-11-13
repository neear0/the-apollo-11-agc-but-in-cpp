#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <format>
#include <memory>
#include <print>

#include "lm/pad_load.hpp"
#include "ui/ui.h"
#include "yul/assembler.hpp"

#include "app.hpp"

static std::string program_status;

// the program's sources: <name>/MAIN.agc in the directory the executable is in or one above it (bin\x64\Release\
// in the repo), the closest one
static std::filesystem::path find_program(std::string_view name)
{
    wchar_t buffer[MAX_PATH] = {};
    GetModuleFileNameW(nullptr, buffer, MAX_PATH);

    for (std::filesystem::path directory = std::filesystem::path(buffer).parent_path(); !directory.empty(); directory = directory.parent_path())
    {
        const std::filesystem::path main_file = directory / name / "MAIN.agc";
        if (std::filesystem::exists(main_file))
            return main_file;
        if (directory == directory.root_path())
            break;
    }
    return {};
}

// the computer with Luminary 099's rope and the pad load, or with an empty rope and why in program_status
static std::unique_ptr<agc::c_computer> load_computer()
{
    const std::filesystem::path main_file = find_program("Luminary099");
    if (main_file.empty())
    {
        program_status = "Luminary099/MAIN.agc not found";
        return std::make_unique<agc::c_computer>(agc::rope_t{});
    }

    const auto assembly = yul::assemble(main_file);
    if (!assembly)
    {
        for (const std::string& error : assembly.error())
            std::println("{}", error);
        program_status = std::format("Luminary 099 didn't assemble: {} errors", assembly.error().size());
        return std::make_unique<agc::c_computer>(agc::rope_t{});
    }

    auto computer = std::make_unique<agc::c_computer>(assembly->rope);
    for (const std::string& name : lm::load_pad(*computer, *assembly))
        std::println("no pad load symbol {}", name);
    program_status = "Luminary 099";
    return computer;
}

agc::c_computer& app::computer()
{
    static const std::unique_ptr<agc::c_computer> computer = load_computer();
    return *computer;
}

lm::c_lunar_module& app::lunar_module()
{
    static lm::c_lunar_module lunar_module;
    return lunar_module;
}

const std::string& app::status()
{
    app::computer(); // assembled first
    return program_status;
}

int main()
{
    // dlls loaded at runtime only come from system32, and the ui scales itself to the monitor (see CelosUI's main.cpp)
    SetDefaultDllDirectories(LOAD_LIBRARY_SEARCH_SYSTEM32);
    SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);

    agc::c_computer& computer = app::computer();
    lm::c_lunar_module& lunar_module = app::lunar_module();

    celosia::initialize::context();
    celosia::variables::title = { "apollo ", "11" };
    celosia::style::general::main_color = ImColor(124, 255, 128); // the display's green
    celosia::core::keyboard_nav = false; // the keyboard is the dsky's keypad instead, see view::keyboard()

    if (!celosia::platform::create("apollo 11", ImVec2(0.f, 0.f), ImVec2(816.f, 620.f)))
        return 1;

    celosia::initialize::fonts(); // at the scale create() picked for the monitor
    celosia::resources::fonts::add("dsky_label", { celosia::resources::font_data::poppins_bold, celosia::resources::font_data::poppins_bold_size, 11.f });
    celosia::resources::fonts::add("dsky_key", { celosia::resources::font_data::poppins_bold, celosia::resources::font_data::poppins_bold_size, 15.f });
    celosia::initialize::images();

    // the computer and the spacecraft keep real time: each frame they run for as long as the last one took (a stall
    // isn't caught up on)
    auto last_frame = std::chrono::steady_clock::now();
    while (celosia::platform::poll())
    {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - last_frame).count();
        last_frame = now;
        lunar_module.run(computer, std::min(elapsed, 0.1));

        celosia::ui::render();
        celosia::platform::bring_to_top();
    }

    celosia::platform::destroy();
    ImGui::DestroyContext();
    return 0;
}

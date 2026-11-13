#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <format>
#include <memory>
#include <optional>
#include <print>

#include "lm/landing.hpp"
#include "lm/pad_load.hpp"
#include "ui/ui.h"
#include "yul/assembler.hpp"

#include "app.hpp"

// Luminary 099 assembled once, and the computer and lunar module it runs in (the landing starts them over)
static std::string program_status;
static std::optional<yul::assembly_t> program;
static std::unique_ptr<agc::c_computer> current_computer;
static std::unique_ptr<lm::c_lunar_module> current_lunar_module;
static std::optional<app::landing_t> current_landing;

enum class restart_t { none, power_on, landing };
static restart_t restart_requested = restart_t::none;

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

// Luminary 099 assembled from its sources, or why not in program_status
static void assemble_program()
{
    const std::filesystem::path main_file = find_program("Luminary099");
    if (main_file.empty())
    {
        program_status = "Luminary099/MAIN.agc not found";
        return;
    }

    auto assembly = yul::assemble(main_file);
    if (!assembly)
    {
        for (const std::string& error : assembly.error())
            std::println("{}", error);
        program_status = std::format("Luminary 099 didn't assemble: {} errors", assembly.error().size());
        return;
    }

    program = std::move(*assembly);
    program_status = "Luminary 099";
}

// a computer with Luminary 099's rope and the pad load, just switched on, in a lunar module just made (an empty rope
// if there's no program)
static void start_computer()
{
    current_landing.reset();
    current_lunar_module = std::make_unique<lm::c_lunar_module>();
    if (!program)
    {
        current_computer = std::make_unique<agc::c_computer>(agc::rope_t{});
        return;
    }

    current_computer = std::make_unique<agc::c_computer>(program->rope);
    for (const std::string& name : lm::load_pad(*current_computer, *program))
        std::println("no pad load symbol {}", name);
}

// the landing, as agc_check flies it: aligned before the imu's turn-on, the turn-on waited out (2 minutes, run here at
// once), then the uplink, 8 minutes before ignition
static void start_landing()
{
    static constexpr double lead = 480.0, imu_turn_on = 120.0;

    start_computer();
    if (!program)
        return;

    app::landing_t landing;
    landing.start = lm::plan_landing(*current_computer, *program, lead);
    lm::align_for_landing(*current_lunar_module, landing.start);
    current_lunar_module->run(*current_computer, imu_turn_on);
    lm::uplink_landing(*current_computer, *current_lunar_module, landing.start, *program);
    landing.uplinked_at = current_computer->time();
    current_landing = std::move(landing);
}

agc::c_computer& app::computer()
{
    if (!current_computer)
    {
        assemble_program();
        start_computer();
    }
    return *current_computer;
}

lm::c_lunar_module& app::lunar_module()
{
    app::computer(); // made with it
    return *current_lunar_module;
}

const std::string& app::status()
{
    app::computer(); // assembled first
    return program_status;
}

void app::start_landing()
{
    restart_requested = restart_t::landing;
}

void app::power_on()
{
    restart_requested = restart_t::power_on;
}

const app::landing_t* app::landing()
{
    return current_landing ? &*current_landing : nullptr;
}

double app::mission_time()
{
    const double time = app::computer().time();
    return current_landing ? current_landing->start.time + time - current_landing->uplinked_at : time;
}

app::profile_point_t app::profile_point()
{
    if (!current_landing)
        return {};

    // the site ahead or behind along the orbit, the angle to it on the surface's sphere
    const lm::c_lunar_module& lunar_module = app::lunar_module();
    const lm::vec3_t site = lm::site_at(current_landing->start, app::mission_time());
    const lm::vec3_t& position = lunar_module.position();
    const lm::vec3_t normal = lm::cross(position, lunar_module.velocity());
    const lm::vec3_t across = lm::cross(position, site);
    const double sine = lm::length(across) / (lm::length(position) * lm::length(site));
    const double cosine = lm::dot(position, site) / (lm::length(position) * lm::length(site));
    const double angle = std::atan2(lm::dot(across, normal) >= 0.0 ? sine : -sine, cosine);
    return { angle * lunar_module.surface_radius, lunar_module.height() };
}

// a point of the landing's track every 2 s of it
static void record_track()
{
    static constexpr double every = 2.0;
    if (!current_landing)
        return;
    std::vector<app::profile_point_t>& track = current_landing->track;
    const double since = current_computer->time() - current_landing->uplinked_at;
    if (static_cast<double>(track.size()) * every <= since)
        track.push_back(app::profile_point());
}

int main()
{
    // dlls loaded at runtime only come from system32, and the ui scales itself to the monitor (see CelosUI's main.cpp)
    SetDefaultDllDirectories(LOAD_LIBRARY_SEARCH_SYSTEM32);
    SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);

    app::computer(); // assembled before the window opens
    celosia::initialize::context();
    celosia::variables::title = { "apollo ", "11" };
    celosia::style::general::main_color = ImColor(124, 255, 128); // the display's green
    celosia::core::keyboard_nav = false; // the keyboard is the dsky's keypad instead, see view::keyboard()

    if (!celosia::platform::create("apollo 11", ImVec2(0.f, 0.f), ImVec2(1092.f, 620.f)))
        return 1;

    celosia::initialize::fonts(); // at the scale create() picked for the monitor
    celosia::resources::fonts::add("dsky_label", { celosia::resources::font_data::poppins_bold, celosia::resources::font_data::poppins_bold_size, 11.f });
    celosia::resources::fonts::add("dsky_key", { celosia::resources::font_data::poppins_bold, celosia::resources::font_data::poppins_bold_size, 15.f });
    celosia::initialize::images();

    // the computer and the spacecraft keep real time, or time_warp times it: each frame they run for as long as the last
    // one took (a stall isn't caught up on)
    auto last_frame = std::chrono::steady_clock::now();
    while (celosia::platform::poll())
    {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - last_frame).count();
        last_frame = now;

        // a restart asked for last frame, between frames, where no view holds the computer
        if (restart_requested == restart_t::landing)
            start_landing();
        else if (restart_requested == restart_t::power_on)
            start_computer();
        restart_requested = restart_t::none;

        app::lunar_module().run(app::computer(), std::min(elapsed, 0.1) * app::time_warp);
        record_track();

        celosia::ui::render();
        celosia::platform::bring_to_top();
    }

    celosia::platform::destroy();
    ImGui::DestroyContext();
    return 0;
}

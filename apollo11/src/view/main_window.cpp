#include <cmath>
#include <format>
#include <string>

#include "ui/ui.h"

#include "app.hpp"
#include "view/dsky_view.hpp"
#include "view/spacecraft_view.hpp"

// what's running on the left, how long since power-on on the right, along the bottom
static void status_line(ImDrawList* drawlist, float top, float margin)
{
    const agc::c_computer& computer = app::computer();
    ImFont* small = celosia::resources::fonts::get("default_smaller");
    const ImU32 text = ImColor(celosia::style::themes::active.text.Value.x, celosia::style::themes::active.text.Value.y,
        celosia::style::themes::active.text.Value.z, 0.6f);

    std::string status = app::status();
    if (computer.standby())
        status += "  -  standby";
    if (computer.restarts() > 0)
        status += std::format("  -  {} hardware restart{}", computer.restarts(), computer.restarts() == 1 ? "" : "s");

    const int seconds = static_cast<int>(computer.time());
    const std::string clock = std::format("{:02}:{:02}:{:02}", seconds / 3600, seconds / 60 % 60, seconds % 60);

    const float line = celosia::render::text::size("A", nullptr, small).y;
    const float y = std::floor(top + (celosia::ui::size.y - top - line) * 0.5f);
    celosia::render::text::draw(drawlist, ImVec2(margin, y), text, status.c_str(), nullptr, small);

    const ImVec2 clock_size = celosia::render::text::size(clock.c_str(), nullptr, small);
    celosia::render::text::draw(drawlist, ImVec2(celosia::ui::size.x - margin - clock_size.x, y), text, clock.c_str(), nullptr, small);
}

// CelosUI's ui::render() draws this every frame, in place of its demo window
void celosia::render::main_window()
{
    const ImVec2 pos(0.f, 0.f);
    celosia::window::begin(celosia::variables::title.name, pos, celosia::ui::size);

    ImDrawList* drawlist = celosia::core::drawlist();
    if (celosia::style::window::background == celosia::style::window::shader)
    {
        const ImVec4 main = celosia::style::general::accent.Value;
        const ImVec4 accent = celosia::style::themes::rgb_accent(celosia::style::general::accent).Value;
        ImVec4 base = celosia::style::themes::active.background.Value;
        base.w = 1.f;
        celosia::effects::background(drawlist, pos, pos + celosia::ui::size, base,
            ImColor(main.x, main.y, main.z, 0.55f), ImColor(accent.x, accent.y, accent.z, 0.45f), static_cast<float>(celosia::style::general::rounding_window));
    }
    celosia::effects::noise(drawlist, pos, pos + celosia::ui::size, celosia::style::window::noise, static_cast<float>(celosia::style::general::rounding_window));
    celosia::render::titlebar();

    agc::c_dsky& dsky = app::computer().dsky();
    lm::c_lunar_module& lunar_module = app::lunar_module();
    view::keyboard(dsky);
    view::hand_controller(lunar_module);

    // the lunar module on the left, the dsky on the right
    const float margin = 16.f * celosia::style::scale;
    const float top = celosia::style::titlebar::height + margin;
    const float height = view::dsky_size().y;
    view::spacecraft(lunar_module, ImVec2(margin, top), height);
    view::dsky(dsky, ImVec2(margin + view::spacecraft_size().x + margin, top));
    status_line(drawlist, top + height, margin);

    celosia::window::end();
}

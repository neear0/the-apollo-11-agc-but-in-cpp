#include "view/descent_view.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <string>
#include <vector>

#include "app.hpp"
#include "view/controls.hpp"

using view::dim;
using view::px;

static constexpr ImU32 track_color   = IM_COL32(124, 255, 128, 255); // the display's green
static constexpr ImU32 site_color    = IM_COL32(255, 190, 70, 255);
static constexpr ImU32 contact_color = IM_COL32(120, 175, 255, 255); // the lunar contact lights were blue
static constexpr ImU32 light_white   = IM_COL32(236, 236, 224, 255);

// the units the crew had: feet, feet a second, nautical miles
static constexpr double feet = 0.3048;
static constexpr double nautical_miles = 1852.0;

// the full thrust the throttle's percent is of, 10500 lb: the throttle's stop (FMAX) is 93% of it
static constexpr double full_thrust = 46706.0; // n

// the descent so far, height over the surface against the distance to the landing site, the site on the right
static void draw_profile(ImDrawList* drawlist, const ImRect& plot, const app::landing_t* landing)
{
    ImFont* small = celosia::resources::fonts::get("dsky_label");
    const ImU32 text = dim(celosia::style::themes::active.text, 0.45f);
    drawlist->AddRectFilled(plot.Min, plot.Max, IM_COL32(0, 0, 0, 60), px(4.f));
    if (!landing)
    {
        view::draw_centered(drawlist, plot, text, "no landing flown\nLANDING below starts one", small);
        return;
    }

    const app::profile_point_t now = app::profile_point();
    double farthest = 1000.0, nearest = 0.0, highest = 100.0;
    for (const app::profile_point_t& point : landing->track)
    {
        farthest = std::max(farthest, point.downrange);
        nearest = std::min(nearest, point.downrange);
        highest = std::max(highest, point.height);
    }
    farthest = std::max(farthest, now.downrange);
    nearest = std::min(nearest, now.downrange);
    highest = std::max(highest, now.height) * 1.1;

    const ImRect inside(plot.Min + px(8.f, 8.f), plot.Max - px(8.f, 8.f));
    const auto screen = [&](const app::profile_point_t& point) {
        const double x = (point.downrange - nearest) / (farthest - nearest);
        const double y = std::max(point.height, 0.0) / highest;
        return ImVec2(inside.Max.x - static_cast<float>(x) * inside.GetWidth(), inside.Max.y - static_cast<float>(y) * inside.GetHeight());
    };

    // the surface, and the site on it
    drawlist->AddLine(ImVec2(inside.Min.x, inside.Max.y), inside.Max, dim(celosia::style::themes::active.text, 0.3f), px(1.f));
    const ImVec2 site = screen({ 0.0, 0.0 });
    drawlist->AddTriangleFilled(site, site + px(-4.f, 7.f), site + px(4.f, 7.f), site_color);

    std::vector<ImVec2> points;
    points.reserve(landing->track.size() + 1);
    for (const app::profile_point_t& point : landing->track)
        points.push_back(screen(point));
    points.push_back(screen(now));
    drawlist->AddPolyline(points.data(), static_cast<int>(points.size()), track_color, ImDrawFlags_None, px(1.5f));
    drawlist->AddCircleFilled(points.back(), px(3.f), track_color);

    const std::string top = std::format("{:.0f} ft", highest / feet);
    const std::string left = std::format("{:.0f} nmi", farthest / nautical_miles);
    celosia::render::text::draw(drawlist, plot.Min + px(6.f, 4.f), text, top.c_str(), nullptr, small);
    const float line = celosia::render::text::size("A", nullptr, small).y;
    celosia::render::text::draw(drawlist, ImVec2(plot.Min.x + px(6.f), plot.Max.y - line - px(12.f)), text, left.c_str(), nullptr, small);
}

// a label, with its value under it
static void draw_readout(ImDrawList* drawlist, ImVec2 pos, const char* label, const std::string& value)
{
    ImFont* small = celosia::resources::fonts::get("dsky_label");
    ImFont* large = celosia::resources::fonts::get("dsky_key");
    celosia::render::text::draw(drawlist, pos, dim(celosia::style::themes::active.text, 0.6f), label, nullptr, small);
    celosia::render::text::draw(drawlist, pos + px(0.f, 13.f), celosia::style::themes::active.text, value.c_str(), nullptr, large);
}

ImVec2 view::descent_size()
{
    return px(260.f, 0.f);
}

void view::descent(lm::c_lunar_module& lunar_module, ImVec2 pos, float height)
{
    ImDrawList* drawlist = celosia::core::drawlist();
    const ImColor panel = celosia::style::themes::active.background_darker;
    const float rounding = static_cast<float>(celosia::style::general::rounding) * 2.f;
    const float width = descent_size().x;
    ImFont* small = celosia::resources::fonts::get("dsky_label");
    const ImU32 text = dim(celosia::style::themes::active.text, 0.6f);
    const app::landing_t* landing = app::landing();
    const lm::touchdown_t& touchdown = lunar_module.touchdown();

    // the profile, the readouts under it, the lights at the bottom
    const ImRect top(pos, pos + ImVec2(width, px(312.f)));
    celosia::render::panel(drawlist, top.Min, top.Max, panel, rounding);
    celosia::render::text::draw(drawlist, top.Min + px(12.f, 10.f), text, "DESCENT", nullptr, small);
    if (touchdown.landed)
    {
        // what the gear was built to take: 10 ft/s down, 4 ft/s across
        const bool crashed = touchdown.descent_rate > 10.0 * feet || touchdown.drift > 4.0 * feet;
        const std::string landed = std::format("{} AT {:.1f} FT/S", crashed ? "CRASHED" : "LANDED", touchdown.descent_rate / feet);
        celosia::render::text::draw(drawlist, ImVec2(top.Max.x - celosia::render::text::size(landed.c_str(), nullptr, small).x - px(12.f), top.Min.y + px(10.f)),
            crashed ? site_color : track_color, landed.c_str(), nullptr, small);
    }

    const float left = top.Min.x + px(12.f), right = top.Max.x - px(12.f);
    draw_profile(drawlist, ImRect(ImVec2(left, top.Min.y + px(30.f)), ImVec2(right, top.Min.y + px(150.f))), landing);

    // over the surface: the footpads' height, the rate up or down, the speed across it, all as on the moon turning under
    // it. dashes without a landing, where the lunar module isn't anywhere in particular
    const lm::descent_engine_t& engine = lunar_module.descent_engine();
    std::array<std::string, 6> values;
    values.fill("-----");
    if (landing)
    {
        const lm::vec3_t over_surface = lunar_module.velocity() - lm::cross(lunar_module.moon_rotation, lunar_module.position());
        const lm::vec3_t up = lunar_module.position() / lm::length(lunar_module.position());
        const double rate = lm::dot(over_surface, up);
        values[0] = std::format("{:.0f} ft", std::max(lunar_module.height(), 0.0) / feet);
        values[1] = std::format("{:+.1f} ft/s", rate / feet);
        values[2] = std::format("{:.1f} ft/s", lm::length(over_surface - up * rate) / feet);
        values[3] = std::format("{:.2f} nmi", app::profile_point().downrange / nautical_miles);
    }
    values[4] = std::format("{:.0f}%", engine.thrust / full_thrust * 100.0);
    values[5] = std::format("{:.1f}%", engine.propellant / lm::descent_propellant * 100.0);

    static constexpr std::array<const char*, 6> labels = { "ALTITUDE", "ALT RATE", "HORIZ VEL", "TO SITE", "THRUST", "DES QTY" };
    const float column = (right - left) * 0.5f;
    for (std::size_t i = 0; i < labels.size(); ++i)
    {
        const ImVec2 at(left + column * static_cast<float>(i % 2), top.Min.y + px(160.f) + px(34.f) * static_cast<float>(i / 2));
        draw_readout(drawlist, at, labels[i], values[i]);
    }

    const lm::landing_radar_t& radar = lunar_module.landing_radar();
    struct light_t
    {
        const char* label;
        bool lit;
        ImU32 color;
    };
    const std::array<light_t, 3> lights = { {
        { "CONTACT", touchdown.contact, contact_color },
        { "ENGINE", engine.on, light_white },
        { "LR LOCK", radar.range_good && radar.velocity_good, light_white },
    } };
    const float gap = px(8.f);
    const float light_width = (right - left - gap * 2.f) / 3.f;
    for (std::size_t i = 0; i < lights.size(); ++i)
    {
        const ImVec2 min(left + (light_width + gap) * static_cast<float>(i), top.Min.y + px(270.f));
        view::draw_light(drawlist, ImRect(min, min + ImVec2(light_width, px(30.f))), lights[i].label, lights[i].lit, lights[i].color, small);
    }

    // the switches under it
    const ImRect controls(ImVec2(pos.x, top.Max.y + px(16.f)), ImVec2(pos.x + width, pos.y + height));
    celosia::render::panel(drawlist, controls.Min, controls.Max, panel, rounding);

    const float label_width = px(78.f), row_height = px(28.f), row_gap = px(9.f);
    float y = controls.Min.y + px(12.f);
    const auto row = [&](const char* label) {
        const ImRect whole(ImVec2(left, y), ImVec2(right, y + row_height));
        view::switch_label(drawlist, whole, label);
        y += row_height + row_gap;
        return ImRect(ImVec2(left + label_width, whole.Min.y), whole.Max);
    };

    static constexpr const char* arm_items[] = { "OFF", "DES" };
    int armed = lunar_module.engine_armed ? 1 : 0;
    if (view::switch_row("##engine_arm", row("ENG ARM"), arm_items, 2, &armed))
        lunar_module.engine_armed = armed == 1;

    static constexpr const char* stop_items[] = { "RUN", "STOP" };
    int stop = lunar_module.engine_stop ? 1 : 0;
    if (view::switch_row("##engine_stop", row("ENG STOP"), stop_items, 2, &stop))
        lunar_module.engine_stop = stop == 1;

    static constexpr const char* warp_items[] = { "1x", "2x", "5x", "10x" };
    static constexpr std::array<int, 4> warps = { 1, 2, 5, 10 };
    int warp = static_cast<int>(std::find(warps.begin(), warps.end(), app::time_warp) - warps.begin());
    if (view::switch_row("##time_warp", row("TIME"), warp_items, 4, &warp))
        app::time_warp = warps[static_cast<std::size_t>(warp)];

    const ImRect buttons = row("START");
    const float half = (buttons.GetWidth() - gap) * 0.5f;
    if (view::button("LANDING##start_landing", ImRect(buttons.Min, buttons.Min + ImVec2(half, buttons.GetHeight())), small))
        app::start_landing();
    if (view::button("POWER ON##power_on", ImRect(ImVec2(buttons.Max.x - half, buttons.Min.y), buttons.Max), small))
        app::power_on();
}

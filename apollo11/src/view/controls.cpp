#include "view/controls.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

static constexpr ImU32 light_off      = IM_COL32(255, 255, 255, 14);
static constexpr ImU32 light_text_lit = IM_COL32(24, 22, 18, 255);

float view::px(float value)
{
    return value * celosia::style::scale;
}

ImVec2 view::px(float x, float y)
{
    return ImVec2(x, y) * celosia::style::scale;
}

ImU32 view::dim(ImColor color, float alpha)
{
    color.Value.w *= alpha;
    return color;
}

void view::draw_centered(ImDrawList* drawlist, const ImRect& rect, ImU32 color, const char* text, ImFont* font)
{
    int lines = 1;
    for (const char* c = text; *c; ++c)
        lines += *c == '\n';

    const float line_height = celosia::render::text::size("A", nullptr, font).y;
    float y = std::floor(rect.GetCenter().y - line_height * static_cast<float>(lines) * 0.5f);

    for (const char* line = text; ; )
    {
        const char* line_end = std::strchr(line, '\n');
        if (!line_end)
            line_end = line + std::strlen(line);

        const ImVec2 size = celosia::render::text::size(line, line_end, font);
        celosia::render::text::draw(drawlist, ImVec2(std::floor(rect.GetCenter().x - size.x * 0.5f), y), color, line, line_end, font);

        if (!*line_end)
            break;

        line = line_end + 1;
        y += line_height;
    }
}

void view::draw_light(ImDrawList* drawlist, const ImRect& rect, const char* label, bool lit, ImU32 color, ImFont* font)
{
    drawlist->AddRectFilled(rect.Min, rect.Max, lit ? color : light_off, px(3.f));
    if (label[0] != '\0')
        draw_centered(drawlist, rect, lit ? light_text_lit : dim(celosia::style::themes::active.text, 0.35f), label, font);
}

bool view::button(const char* label, const ImRect& bb, ImFont* font, bool* held_down)
{
    const ImGuiID id = celosia::core::id(label);
    celosia::core::layout(bb);
    if (!celosia::core::add(bb, id))
        return false;

    bool hovered = false, held = false;
    const bool pressed = celosia::core::button(bb, id, &hovered, &held);
    if (held_down)
        *held_down = held;

    const float rounding = static_cast<float>(celosia::style::general::rounding);
    const float glow = celosia::animations::set(celosia::animations::key("dsky_key_glow", id), hovered ? 1.f : 0.f);
    if (glow > 0.01f)
    {
        const ImVec4 accent = celosia::style::general::accent.Value;
        celosia::effects::glow(celosia::core::drawlist(), bb.Min, bb.Max, ImColor(accent.x, accent.y, accent.z, 0.35f * glow), rounding, px(8.f));
    }

    const celosia::style::themes::t_theme& theme = celosia::style::themes::active;
    const ImColor color = (held && hovered) ? theme.held : hovered ? theme.hovered : theme.idle;

    ImVec4 fill = ImLerp(color.Value, theme.background_darker.Value, 0.25f);
    fill.w = color.Value.w;
    ImVec4 border = ImLerp(color.Value, theme.text.Value, 0.12f);
    border.w = color.Value.w;

    celosia::widgets::render_frame_animated(id, bb.Min, bb.Max, fill, rounding);
    celosia::widgets::render_frame_border_animated(id, bb.Min, bb.Max, border, rounding);

    // the label without what follows ## in it, as imgui does
    const char* hidden = std::strstr(label, "##");
    const std::string shown(label, hidden ? hidden : label + std::strlen(label));
    draw_centered(celosia::core::drawlist(), bb, theme.text, shown.c_str(), font);

    return pressed;
}

bool view::switch_row(const char* id_label, const ImRect& frame, const char* const items[], int count, int* current)
{
    const ImGuiID id = celosia::core::id(id_label);
    celosia::core::layout(frame);
    if (!celosia::core::add(frame, id, celosia::core::item_no_focus))
        return false;

    ImDrawList* drawlist = celosia::core::drawlist();
    const celosia::style::themes::t_theme& theme = celosia::style::themes::active;
    const ImVec4 accent = celosia::style::general::accent.Value;
    const float rounding = static_cast<float>(celosia::style::general::rounding);
    const float segment = frame.GetWidth() / static_cast<float>(count);

    ImVec4 border = ImLerp(theme.idle.Value, theme.text.Value, 0.12f);
    border.w = theme.idle.Value.w;
    drawlist->AddRectFilled(frame.Min, frame.Max, theme.idle, rounding);
    drawlist->AddRect(frame.Min, frame.Max, ImColor(border), rounding, 0, celosia::style::frame::border_size);

    bool changed = false;
    for (int i = 0; i < count; ++i)
    {
        const ImRect rect(ImVec2(frame.Min.x + segment * static_cast<float>(i), frame.Min.y), ImVec2(frame.Min.x + segment * static_cast<float>(i + 1), frame.Max.y));
        const ImGuiID item = ImHashData(&i, sizeof(i), id);
        if (!celosia::core::add(rect, item))
            continue;

        bool hovered = false, held = false;
        if (celosia::core::button(rect, item, &hovered, &held) && *current != i)
        {
            *current = i;
            changed = true;
        }
        if (hovered && *current != i)
            drawlist->AddRectFilled(rect.Min, rect.Max, dim(theme.text, 0.08f), rounding);
    }

    // the pill slides to the one picked
    const float inset = px(3.f);
    const float pill_x = celosia::animations::set(celosia::animations::key("switch_pill", id), segment * static_cast<float>(*current), 14.f);
    const ImVec2 pill_min(frame.Min.x + pill_x + inset, frame.Min.y + inset), pill_max(frame.Min.x + pill_x + segment - inset, frame.Max.y - inset);
    celosia::effects::glow(drawlist, pill_min, pill_max, ImColor(accent.x, accent.y, accent.z, 0.3f), rounding, px(6.f));
    drawlist->AddRectFilled(pill_min, pill_max, ImColor(accent), std::max(rounding - inset * 0.5f, 0.f));

    ImFont* font = celosia::resources::fonts::get("dsky_label");
    const ImVec4 on_accent = celosia::style::themes::text_on(accent).Value;
    for (int i = 0; i < count; ++i)
    {
        const float x = frame.Min.x + segment * static_cast<float>(i);
        const float covered = ImSaturate(1.f - std::fabs(pill_x - segment * static_cast<float>(i)) / segment);
        const ImVec2 size = celosia::render::text::size(items[i], nullptr, font);
        const ImVec2 at(std::floor(x + (segment - size.x) * 0.5f), std::floor(frame.GetCenter().y - size.y * 0.5f));
        celosia::render::text::draw(drawlist, at, ImColor(ImLerp(theme.text.Value, on_accent, covered)), items[i], nullptr, font);
    }
    return changed;
}

void view::switch_label(ImDrawList* drawlist, const ImRect& row, const char* label)
{
    ImFont* font = celosia::resources::fonts::get("dsky_label");
    const ImVec2 size = celosia::render::text::size(label, nullptr, font);
    celosia::render::text::draw(drawlist, ImVec2(row.Min.x, std::floor(row.GetCenter().y - size.y * 0.5f)), dim(celosia::style::themes::active.text, 0.6f), label, nullptr, font);
}

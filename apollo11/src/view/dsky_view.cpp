#include "view/dsky_view.hpp"

#include <array>
#include <cmath>
#include <format>
#include <span>
#include <string>

#include "ui/ui.h"
#include "view/controls.hpp"

using view::px;

// the electroluminescent display's green, and the lights' white and amber
static constexpr ImU32 el_lit          = IM_COL32(124, 255, 128, 255);
static constexpr ImU32 el_unlit        = IM_COL32(124, 255, 128, 16);
static constexpr ImU32 el_unlit_text   = IM_COL32(124, 255, 128, 70);
static constexpr ImU32 el_label_text   = IM_COL32(10, 26, 12, 255);
static constexpr ImU32 el_background   = IM_COL32(8, 12, 9, 235);
static constexpr ImU32 light_white     = IM_COL32(236, 236, 224, 255);
static constexpr ImU32 light_amber     = IM_COL32(255, 190, 70, 255);

static constexpr float digit_width  = 22.f;
static constexpr float digit_height = 36.f;
static constexpr float digit_gap    = 5.f;
static constexpr float segment      = 4.f; // how thick a segment is
static constexpr float sign_width   = 18.f;

// segments a to g as bits 0 to 6, a at the top going clockwise and g in the middle
static constexpr std::array<std::uint8_t, 10> segment_patterns = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F };

static void draw_digit(ImDrawList* drawlist, ImVec2 pos, std::int8_t digit)
{
    const float w = px(digit_width), h = px(digit_height), t = px(segment), gap = px(1.f);
    const float mid = h * 0.5f, half = t * 0.5f;

    const std::array<ImRect, 7> segments = {
        ImRect(pos + ImVec2(t + gap, 0.f),             pos + ImVec2(w - t - gap, t)),           // a
        ImRect(pos + ImVec2(w - t, t + gap),           pos + ImVec2(w, mid - half - gap)),      // b
        ImRect(pos + ImVec2(w - t, mid + half + gap),  pos + ImVec2(w, h - t - gap)),           // c
        ImRect(pos + ImVec2(t + gap, h - t),           pos + ImVec2(w - t - gap, h)),           // d
        ImRect(pos + ImVec2(0.f, mid + half + gap),    pos + ImVec2(t, h - t - gap)),           // e
        ImRect(pos + ImVec2(0.f, t + gap),             pos + ImVec2(t, mid - half - gap)),      // f
        ImRect(pos + ImVec2(t + gap, mid - half),      pos + ImVec2(w - t - gap, mid + half)),  // g
    };

    std::uint8_t lit = 0;
    if (digit >= 0 && digit <= 9)
        lit = segment_patterns[static_cast<std::size_t>(digit)];

    for (std::size_t i = 0; i < segments.size(); ++i)
        drawlist->AddRectFilled(segments[i].Min, segments[i].Max, (lit >> i) & 1 ? el_lit : el_unlit, half);
}

static void draw_digits(ImDrawList* drawlist, ImVec2 pos, std::span<const std::int8_t> digits)
{
    for (std::size_t i = 0; i < digits.size(); ++i)
        draw_digit(drawlist, pos + ImVec2(px(digit_width + digit_gap) * static_cast<float>(i), 0.f), digits[i]);
}

static void draw_sign(ImDrawList* drawlist, ImVec2 pos, agc::register_sign sign)
{
    const float w = px(sign_width), h = px(digit_height), t = px(segment), gap = px(1.f);
    const float mid = h * 0.5f, half = t * 0.5f, center = w * 0.5f;

    const bool minus = sign != agc::register_sign::blank; // plus is the minus with the vertical bar
    const bool plus  = sign == agc::register_sign::plus;

    drawlist->AddRectFilled(pos + ImVec2(0.f, mid - half), pos + ImVec2(w, mid + half), minus ? el_lit : el_unlit, half);
    drawlist->AddRectFilled(pos + ImVec2(center - half, mid - center), pos + ImVec2(center + half, mid - half - gap), plus ? el_lit : el_unlit, half);
    drawlist->AddRectFilled(pos + ImVec2(center - half, mid + half + gap), pos + ImVec2(center + half, mid + center), plus ? el_lit : el_unlit, half);
}

static void draw_el_label(ImDrawList* drawlist, const ImRect& rect, const char* text, bool lit, ImFont* font)
{
    drawlist->AddRectFilled(rect.Min, rect.Max, lit ? el_lit : el_unlit, px(2.f));
    view::draw_centered(drawlist, rect, lit ? el_label_text : el_unlit_text, text, font);
}

// prog, verb and noun: a label with two digits under it
static void draw_field(ImDrawList* drawlist, ImVec2 pos, const char* label, std::span<const std::int8_t> digits, ImFont* font, bool lit)
{
    const float label_width = px(64.f);
    draw_el_label(drawlist, ImRect(pos, pos + ImVec2(label_width, px(16.f))), label, lit, font);

    const float digits_width = px(digit_width * 2.f + digit_gap);
    draw_digits(drawlist, pos + ImVec2(std::floor((label_width - digits_width) * 0.5f), px(20.f)), digits);
}

static void draw_display(ImDrawList* drawlist, const ImRect& rect, const agc::dsky_state_t& lit_state)
{
    ImFont* font = celosia::resources::fonts::get("dsky_label");
    drawlist->AddRectFilled(rect.Min, rect.Max, el_background, px(6.f));

    // in standby the whole panel is dark, labels and separators too
    const bool on = !lit_state.display_off;
    agc::dsky_state_t dark_state;
    const agc::dsky_state_t& state = on ? lit_state : dark_state;

    const ImVec2 origin = rect.Min;
    const float width = rect.GetWidth();
    const float padding = px(12.f);
    const float right_column = width - padding - px(64.f);

    draw_el_label(drawlist, ImRect(origin + ImVec2(padding, padding), origin + ImVec2(padding + px(64.f), padding + px(56.f))),
        "COMP\nACTY", state.comp_acty, font);
    draw_field(drawlist, origin + ImVec2(right_column, padding), "PROG", state.prog, font, on);

    // a flashing verb and noun are off a third of the time, about 1.5 times a second, as the dsky's own oscillator does it
    static constexpr std::array<std::int8_t, 2> blank = { agc::blank_digit, agc::blank_digit };
    const bool flashed_off = state.verb_noun_flash && std::fmod(celosia::ui::time, 0.64) > 0.43;
    const float verb_noun = padding + px(56.f + 12.f);
    draw_field(drawlist, origin + ImVec2(padding, verb_noun), "VERB", flashed_off ? blank : state.verb, font, on);
    draw_field(drawlist, origin + ImVec2(right_column, verb_noun), "NOUN", flashed_off ? blank : state.noun, font, on);

    // r1 to r3, each under a separator
    const float register_width = px(sign_width + digit_gap + digit_width * 5.f + digit_gap * 4.f);
    float y = verb_noun + px(56.f + 10.f);
    for (const agc::display_register_t& display_register : state.registers)
    {
        drawlist->AddRectFilled(origin + ImVec2(padding, y), origin + ImVec2(width - padding, y + px(3.f)), on ? el_lit : el_unlit);

        const ImVec2 pos = origin + ImVec2(width - padding - register_width, y + px(8.f));
        draw_sign(drawlist, pos, display_register.sign);
        draw_digits(drawlist, pos + ImVec2(px(sign_width + digit_gap), 0.f), display_register.digits);

        y += px(48.f);
    }
}

struct light_t
{
    const char* label; // empty for a slot without a light
    bool lit;
    ImU32 color;
};

static void draw_lights(ImDrawList* drawlist, const ImRect& rect, const agc::lights_t& lights)
{
    // two columns, as on the lunar module's dsky
    const std::array<light_t, 14> slots = { {
        { "UPLINK\nACTY", lights.uplink_acty, light_white }, { "TEMP",         lights.temp,        light_amber },
        { "NO ATT",       lights.no_att,      light_white }, { "GIMBAL\nLOCK", lights.gimbal_lock, light_amber },
        { "STBY",         lights.stby,        light_white }, { "PROG",         lights.prog,        light_amber },
        { "KEY REL",      lights.key_rel,     light_white }, { "RESTART",      lights.restart,     light_amber },
        { "OPR ERR",      lights.opr_err,     light_white }, { "TRACKER",      lights.tracker,     light_amber },
        { "PRIO DISP",    lights.prio_disp,   light_white }, { "ALT",          lights.alt,         light_amber },
        { "NO DAP",       lights.no_dap,      light_white }, { "VEL",          lights.vel,         light_amber },
    } };

    ImFont* font = celosia::resources::fonts::get("dsky_label");
    const ImVec2 size = px(96.f, 36.f);
    const ImVec2 step = size + px(8.f, 6.f);
    const ImVec2 origin = rect.Min + px(12.f, 12.f);
    for (std::size_t i = 0; i < slots.size(); ++i)
    {
        const light_t& slot = slots[i];
        const ImVec2 min = origin + ImVec2(step.x * static_cast<float>(i % 2), step.y * static_cast<float>(i / 2));
        view::draw_light(drawlist, ImRect(min, min + size), slot.label, slot.lit, slot.color, font);
    }
}

struct key_slot_t
{
    const char* label;
    agc::dsky_key key;
    float column;
    float row; // verb, noun, key rel and entr sit half a key lower, between the rows
};

static constexpr std::array<key_slot_t, 19> key_slots = { {
    { "VERB",     agc::dsky_key::verb,        0.f, 0.5f }, { "NOUN", agc::dsky_key::noun,    0.f, 1.5f },
    { "+",        agc::dsky_key::plus,        1.f, 0.f  }, { "-",    agc::dsky_key::minus,   1.f, 1.f  }, { "0",    agc::dsky_key::zero,  1.f, 2.f },
    { "7",        agc::dsky_key::seven,       2.f, 0.f  }, { "4",    agc::dsky_key::four,    2.f, 1.f  }, { "1",    agc::dsky_key::one,   2.f, 2.f },
    { "8",        agc::dsky_key::eight,       3.f, 0.f  }, { "5",    agc::dsky_key::five,    3.f, 1.f  }, { "2",    agc::dsky_key::two,   3.f, 2.f },
    { "9",        agc::dsky_key::nine,        4.f, 0.f  }, { "6",    agc::dsky_key::six,     4.f, 1.f  }, { "3",    agc::dsky_key::three, 4.f, 2.f },
    { "CLR",      agc::dsky_key::clear,       5.f, 0.f  }, { "PRO",  agc::dsky_key::proceed, 5.f, 1.f  }, { "RSET", agc::dsky_key::reset, 5.f, 2.f },
    { "KEY\nREL", agc::dsky_key::key_release, 6.f, 0.5f }, { "ENTR", agc::dsky_key::enter,   6.f, 1.5f },
} };

ImVec2 view::dsky_size()
{
    return px(468.f, 312.f + 16.f + 48.f * 3.f + 8.f * 2.f);
}

void view::dsky(agc::c_dsky& dsky, ImVec2 pos)
{
    ImDrawList* drawlist = celosia::core::drawlist();
    const ImColor panel = celosia::style::themes::active.background_darker;
    const float rounding = static_cast<float>(celosia::style::general::rounding) * 2.f;
    const float right = pos.x + dsky_size().x;

    // lights on the left, the display on the right
    const ImRect lights(pos, pos + px(224.f, 312.f));
    celosia::render::panel(drawlist, lights.Min, lights.Max, panel, rounding);
    draw_lights(drawlist, lights, dsky.state().lights);

    const ImRect display(ImVec2(lights.Max.x + px(12.f), pos.y), ImVec2(right, lights.Max.y));
    celosia::render::panel(drawlist, display.Min, display.Max, panel, rounding);
    draw_display(drawlist, ImRect(display.Min + px(10.f, 10.f), display.Max - px(10.f, 10.f)), dsky.state());

    // the keypad under them. pro is held for as long as the mouse holds it down (or the p key, see keyboard())
    ImFont* key_font = celosia::resources::fonts::get("dsky_key");
    const ImVec2 keypad(pos.x, lights.Max.y + px(16.f));
    const ImVec2 key_size = px(60.f, 48.f);
    const ImVec2 key_step = key_size + px(8.f, 8.f);

    bool proceed_held = false;
    for (const key_slot_t& slot : key_slots)
    {
        const ImVec2 min = keypad + ImVec2(key_step.x * slot.column, key_step.y * slot.row);
        bool held = false;
        if (view::button(slot.label, ImRect(min, min + key_size), key_font, &held))
            dsky.press_key(slot.key);
        if (slot.key == agc::dsky_key::proceed && held)
        {
            proceed_held = true;
            if (celosia::input::mouse_clicked(celosia::input::left))
                dsky.press_key(agc::dsky_key::proceed); // seen at once, not only when let go
        }
    }

    proceed_held = proceed_held || (celosia::input::focused() && celosia::input::key_down('P'));
    dsky.hold_proceed(proceed_held);
}

void view::keyboard(agc::c_dsky& dsky)
{
    if (!celosia::input::focused())
        return;

    for (int digit = 0; digit <= 9; ++digit)
        if (celosia::input::key_pressed('0' + digit, false) || celosia::input::key_pressed(0x60 + digit, false)) // the row and the numpad
            dsky.press_key(static_cast<agc::dsky_key>(digit));

    struct binding_t
    {
        int key; // celosia::keys, the windows virtual key codes
        agc::dsky_key dsky_key;
    };

    static constexpr std::array<binding_t, 11> bindings = { {
        { 'V', agc::dsky_key::verb },  { 'N', agc::dsky_key::noun },        { 'C', agc::dsky_key::clear },
        { 'P', agc::dsky_key::proceed }, { 'K', agc::dsky_key::key_release }, { 'R', agc::dsky_key::reset },
        { celosia::keys::enter, agc::dsky_key::enter },
        { 0x6B, agc::dsky_key::plus },  { 0xBB, agc::dsky_key::plus },  // numpad +, and the = + key
        { 0x6D, agc::dsky_key::minus }, { 0xBD, agc::dsky_key::minus }, // numpad -, and the - _ key
    } };

    for (const binding_t& binding : bindings)
        if (celosia::input::key_pressed(binding.key, false))
            dsky.press_key(binding.dsky_key);
}

#pragma once

#include "ui/ui.h"

// what the panels are drawn with: sizes in units at scale 1 (px() scales them the way CelosUI scales its own style),
// text, lights, buttons and switches
namespace view
{
    float px(float value);
    ImVec2 px(float x, float y);
    ImU32 dim(ImColor color, float alpha);

    void draw_centered(ImDrawList* drawlist, const ImRect& rect, ImU32 color, const char* text, ImFont* font); // line by line

    // a lamp with its label, lit in color
    void draw_light(ImDrawList* drawlist, const ImRect& rect, const char* label, bool lit, ImU32 color, ImFont* font);

    // core's button with the frames and glow of widgets::button, and its label in the given font. held while the
    // mouse button is down on it
    bool button(const char* label, const ImRect& bb, ImFont* font, bool* held_down = nullptr);

    // a row of choices in one frame, the picked one in the accent, like widgets::segmented but sized to fit
    bool switch_row(const char* id_label, const ImRect& frame, const char* const items[], int count, int* current);
    void switch_label(ImDrawList* drawlist, const ImRect& row, const char* label); // left of a switch row
}

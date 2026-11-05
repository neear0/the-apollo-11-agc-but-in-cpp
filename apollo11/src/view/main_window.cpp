#include "ui/ui.h"

#include "app.hpp"
#include "view/dsky_view.hpp"

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

    view::keyboard(app::computer().dsky());
    view::dsky(app::computer().dsky(), app::status(), app::computer().time());

    celosia::window::end();
}

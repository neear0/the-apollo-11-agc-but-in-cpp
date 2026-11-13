#include "view/spacecraft_view.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <string>

#include "view/controls.hpp"

using view::dim;
using view::px;
using view::switch_label;
using view::switch_row;

static constexpr ImU32 ball_sky        = IM_COL32(214, 214, 206, 255);
static constexpr ImU32 ball_ground     = IM_COL32(46, 47, 52, 255);
static constexpr ImU32 ball_line_sky   = IM_COL32(40, 40, 40, 150);
static constexpr ImU32 ball_line_ground = IM_COL32(220, 220, 214, 120);
static constexpr ImU32 ball_horizon    = IM_COL32(20, 20, 20, 255);
static constexpr ImU32 ball_rim        = IM_COL32(10, 10, 12, 255);
static constexpr ImU32 index_color     = IM_COL32(255, 190, 70, 255);
static constexpr ImU32 jet_on          = IM_COL32(255, 190, 70, 255);
static constexpr ImU32 jet_off         = IM_COL32(255, 255, 255, 18);

static ImU32 shaded(ImU32 color, float light)
{
    ImColor c(color);
    c.Value.x *= light;
    c.Value.y *= light;
    c.Value.z *= light;
    return c;
}

// -----------------------------------------------------------------------------------------------------------------
// the ball: fixed to the stable member, seen from the vehicle. its middle is where the vehicle points (+z), up is +x and
// right +y, so it reads like the view out of the window: the light half is the stable member's +x side

// a point of the ball, in stable member axes: latitude up from the y-z plane, longitude about x from +z
static lm::vec3_t ball_point(double latitude, double longitude)
{
    return { std::sin(latitude), std::cos(latitude) * std::sin(longitude), std::cos(latitude) * std::cos(longitude) };
}

struct ball_t
{
    ImVec2 center;
    float radius;
    lm::mat3_t stable_to_body;

    lm::vec3_t body(double latitude, double longitude) const { return stable_to_body * ball_point(latitude, longitude); }

    // on the screen, the far side pushed out to the rim
    ImVec2 screen(const lm::vec3_t& d) const
    {
        double x = d.y, y = d.x;
        if (d.z < 0.0)
        {
            const double length = std::sqrt(x * x + y * y);
            if (length > 1e-9)
            {
                x /= length;
                y /= length;
            }
        }
        return ImVec2(center.x + static_cast<float>(x) * radius, center.y - static_cast<float>(y) * radius);
    }
};

static void draw_ball_faces(ImDrawList* drawlist, const ball_t& ball)
{
    static constexpr int bands = 24, segments = 48;
    const double band = std::numbers::pi / bands, segment = lm::two_pi / segments;

    // each face shaded corner to corner, a little darker toward the edge so it looks round
    const ImVec2 uv = drawlist->_Data->TexUvWhitePixel;
    for (int i = 0; i < bands; ++i)
    {
        const double latitude = -std::numbers::pi / 2.0 + band * i;
        const ImU32 color = latitude >= 0.0 ? ball_sky : ball_ground;
        for (int j = 0; j < segments; ++j)
        {
            const double longitude = segment * j;
            const std::array<lm::vec3_t, 4> corners = {
                ball.body(latitude, longitude), ball.body(latitude, longitude + segment),
                ball.body(latitude + band, longitude + segment), ball.body(latitude + band, longitude),
            };
            const double nearest = std::max({ corners[0].z, corners[1].z, corners[2].z, corners[3].z });
            if (nearest <= 0.0)
                continue;

            drawlist->PrimReserve(6, 4);
            const ImDrawIdx first = static_cast<ImDrawIdx>(drawlist->_VtxCurrentIdx);
            for (const int k : { 0, 1, 2, 0, 2, 3 })
                drawlist->PrimWriteIdx(static_cast<ImDrawIdx>(first + k));
            for (const lm::vec3_t& corner : corners)
            {
                const float light = 0.58f + 0.42f * static_cast<float>(std::clamp(corner.z, 0.0, 1.0));
                drawlist->PrimWriteVtx(ball.screen(corner), uv, shaded(color, light));
            }
        }
    }
}

// a line on the ball from one point to the next, as far as it's on the near side
static void draw_ball_line(ImDrawList* drawlist, const ball_t& ball, double latitude_a, double longitude_a, double latitude_b, double longitude_b,
    ImU32 color, float thickness)
{
    static constexpr int steps = 6;
    lm::vec3_t last = ball.body(latitude_a, longitude_a);
    for (int i = 1; i <= steps; ++i)
    {
        const double t = static_cast<double>(i) / steps;
        const lm::vec3_t next = ball.body(latitude_a + (latitude_b - latitude_a) * t, longitude_a + (longitude_b - longitude_a) * t);
        if (last.z > 0.0 && next.z > 0.0)
            drawlist->AddLine(ball.screen(last), ball.screen(next), color, thickness);
        last = next;
    }
}

static void draw_ball_markings(ImDrawList* drawlist, const ball_t& ball, ImFont* font)
{
    const double step = 5.0 * lm::degrees;

    // latitude circles every 30 degrees and the horizon
    for (int latitude_degrees = -60; latitude_degrees <= 60; latitude_degrees += 30)
    {
        const double latitude = latitude_degrees * lm::degrees;
        const bool horizon = latitude_degrees == 0;
        const ImU32 color = horizon ? ball_horizon : latitude > 0.0 ? ball_line_sky : ball_line_ground;
        for (double longitude = 0.0; longitude < lm::two_pi - 1e-9; longitude += step)
            draw_ball_line(drawlist, ball, latitude, longitude, latitude, longitude + step, color, horizon ? px(2.5f) : px(1.f));
    }

    // meridians every 30 degrees, numbered in tens of degrees along the horizon as on the real ball
    for (int longitude_degrees = 0; longitude_degrees < 360; longitude_degrees += 30)
    {
        const double longitude = longitude_degrees * lm::degrees;
        for (double latitude = -std::numbers::pi / 2.0; latitude < std::numbers::pi / 2.0 - 1e-9; latitude += step)
            draw_ball_line(drawlist, ball, latitude, longitude, latitude + step, longitude, latitude >= 0.0 ? ball_line_sky : ball_line_ground, px(1.f));

        for (const double latitude : { 8.0 * lm::degrees, -8.0 * lm::degrees })
        {
            const lm::vec3_t at = ball.body(latitude, longitude);
            if (at.z < 0.35)
                continue;
            const std::string label = std::to_string(longitude_degrees / 10);
            const ImVec2 size = celosia::render::text::size(label.c_str(), nullptr, font);
            const ImU32 color = latitude > 0.0 ? IM_COL32(30, 30, 30, 220) : IM_COL32(230, 230, 224, 200);
            celosia::render::text::draw(drawlist, ball.screen(at) - size * 0.5f, dim(color, static_cast<float>(std::min(1.0, (at.z - 0.35) * 3.0))),
                label.c_str(), nullptr, font);
        }
    }

    // pitch numbers up and down the zero meridian
    for (const int latitude_degrees : { -60, -30, 30, 60 })
    {
        const lm::vec3_t at = ball.body(latitude_degrees * lm::degrees, 4.0 * lm::degrees);
        if (at.z < 0.35)
            continue;
        const std::string label = std::to_string(latitude_degrees > 0 ? latitude_degrees / 10 : 36 + latitude_degrees / 10);
        const ImVec2 size = celosia::render::text::size(label.c_str(), nullptr, font);
        const ImU32 color = latitude_degrees > 0 ? IM_COL32(30, 30, 30, 220) : IM_COL32(230, 230, 224, 200);
        celosia::render::text::draw(drawlist, ball.screen(at) - ImVec2(0.f, size.y * 0.5f), color, label.c_str(), nullptr, font);
    }
}

static void draw_fdai(ImDrawList* drawlist, ImVec2 center, float radius, const lm::mat3_t& stable_to_body, bool powered)
{
    ImFont* font = celosia::resources::fonts::get("dsky_label");
    const ball_t ball{ center, radius, stable_to_body };

    drawlist->AddCircleFilled(center, radius + px(6.f), ball_rim, 96);
    draw_ball_faces(drawlist, ball);
    draw_ball_markings(drawlist, ball, font);
    if (!powered)
        drawlist->AddCircleFilled(center, radius, IM_COL32(0, 0, 0, 120), 96); // no power to the ball: dimmed, it stays put
    drawlist->AddCircle(center, radius + px(1.f), ball_rim, 96, px(3.f));

    // the vehicle's own index, fixed in the middle: wings and a dot
    const float wing = radius * 0.32f, gap = radius * 0.1f, t = px(3.f);
    drawlist->AddLine(center - ImVec2(wing + gap, 0.f), center - ImVec2(gap, 0.f), index_color, t);
    drawlist->AddLine(center + ImVec2(gap, 0.f), center + ImVec2(wing + gap, 0.f), index_color, t);
    drawlist->AddLine(center - ImVec2(gap, 0.f), center + ImVec2(0.f, gap * 0.8f), index_color, t);
    drawlist->AddLine(center + ImVec2(gap, 0.f), center + ImVec2(0.f, gap * 0.8f), index_color, t);
    drawlist->AddCircleFilled(center, px(2.5f), index_color);

    // ticks around the rim every 30 degrees of roll, the top one longer
    for (int i = 0; i < 12; ++i)
    {
        const float angle = static_cast<float>(i) * static_cast<float>(lm::two_pi) / 12.f;
        const ImVec2 direction(std::sin(angle), -std::cos(angle));
        const float length = i == 0 ? px(10.f) : px(6.f);
        drawlist->AddLine(center + direction * (radius + px(7.f)), center + direction * (radius + px(7.f) + length), dim(celosia::style::themes::active.text, 0.5f), px(2.f));
    }
}

// -----------------------------------------------------------------------------------------------------------------
// the 16 jets, channel 5's on top, lit while they fire
static void draw_jets(ImDrawList* drawlist, const ImRect& rect, std::uint16_t jets)
{
    const float size = px(14.f), gap = px(6.f);
    for (int i = 0; i < 16; ++i)
    {
        const ImVec2 min(rect.Min.x + static_cast<float>(i % 8) * (size + gap), rect.Min.y + static_cast<float>(i / 8) * (size + gap));
        const bool on = (jets >> i) & 1;
        if (on)
            celosia::effects::glow(drawlist, min, min + ImVec2(size, size), ImColor(255, 190, 70, 90), px(3.f), px(6.f));
        drawlist->AddRectFilled(min, min + ImVec2(size, size), on ? jet_on : jet_off, px(3.f));
    }
}

// -----------------------------------------------------------------------------------------------------------------

ImVec2 view::spacecraft_size()
{
    return px(300.f, 0.f);
}

void view::spacecraft(lm::c_lunar_module& lunar_module, ImVec2 pos, float height)
{
    ImDrawList* drawlist = celosia::core::drawlist();
    const ImColor panel = celosia::style::themes::active.background_darker;
    const float rounding = static_cast<float>(celosia::style::general::rounding) * 2.f;
    const float width = spacecraft_size().x;
    ImFont* small = celosia::resources::fonts::get("dsky_label");
    const ImU32 text = dim(celosia::style::themes::active.text, 0.6f);
    const lm::c_imu& imu = lunar_module.imu();

    // the ball, with what the imu is doing over it and the gimbal angles and rates under it
    const ImRect ball_panel(pos, pos + ImVec2(width, px(312.f)));
    celosia::render::panel(drawlist, ball_panel.Min, ball_panel.Max, panel, rounding);

    const float radius = px(104.f);
    const ImVec2 center(ball_panel.GetCenter().x, ball_panel.Min.y + px(18.f) + radius + px(8.f));
    draw_fdai(drawlist, center, radius, lm::gimbal_matrix(imu.gimbals()), imu.operate);

    const char* imu_state = !imu.operate ? "IMU OFF" : imu.turn_on_request() ? "IMU TURN-ON" : imu.coarse_align() ? "COARSE ALIGN" : "INERTIAL";
    celosia::render::text::draw(drawlist, ball_panel.Min + px(12.f, 10.f), text, imu_state, nullptr, small);
    celosia::render::text::draw(drawlist, ImVec2(ball_panel.Max.x - celosia::render::text::size("FDAI", nullptr, small).x - px(12.f), ball_panel.Min.y + px(10.f)),
        text, "FDAI", nullptr, small);

    // the gimbals 0-360 as the computer shows them in noun 20, the rates in degrees a second
    const auto positive = [](double radians) { return std::fmod(radians / lm::degrees + 360.0, 360.0); };
    const lm::gimbal_angles_t& gimbals = imu.gimbals();
    const lm::vec3_t& rates = lunar_module.rates();
    const std::string gimbal_line = std::format("OG {:06.2f}   IG {:06.2f}   MG {:06.2f}", positive(gimbals.outer), positive(gimbals.inner), positive(gimbals.middle));
    const std::string rate_line = std::format("RATE  P {:+.2f}   Q {:+.2f}   R {:+.2f}", rates.x / lm::degrees, rates.y / lm::degrees, rates.z / lm::degrees);
    const float line = celosia::render::text::size("A", nullptr, small).y;
    const float lines_y = ball_panel.Max.y - px(12.f) - line * 2.f - px(4.f);
    for (int i = 0; i < 2; ++i)
    {
        const std::string& s = i == 0 ? gimbal_line : rate_line;
        const ImVec2 size = celosia::render::text::size(s.c_str(), nullptr, small);
        celosia::render::text::draw(drawlist, ImVec2(std::floor(ball_panel.GetCenter().x - size.x * 0.5f), lines_y + (line + px(4.f)) * static_cast<float>(i)),
            text, s.c_str(), nullptr, small);
    }

    // the switches and the jets under it
    const ImRect controls(ImVec2(pos.x, ball_panel.Max.y + px(16.f)), ImVec2(pos.x + width, pos.y + height));
    celosia::render::panel(drawlist, controls.Min, controls.Max, panel, rounding);

    const float label_width = px(78.f), row_height = px(28.f), row_gap = px(9.f);
    const float row_left = controls.Min.x + px(12.f), row_right = controls.Max.x - px(12.f);
    float y = controls.Min.y + px(12.f);
    const auto row = [&](const char* label) {
        const ImRect whole(ImVec2(row_left, y), ImVec2(row_right, y + row_height));
        switch_label(drawlist, whole, label);
        y += row_height + row_gap;
        return ImRect(ImVec2(row_left + label_width, whole.Min.y), whole.Max);
    };

    static constexpr const char* imu_items[] = { "OFF", "OPERATE" };
    int imu_operate = imu.operate ? 1 : 0;
    if (switch_row("##imu", row("IMU"), imu_items, 2, &imu_operate))
        lunar_module.imu().operate = imu_operate == 1;

    static constexpr const char* mode_items[] = { "OFF", "ATT HOLD", "AUTO" };
    int mode = static_cast<int>(lunar_module.mode);
    if (switch_row("##mode", row("MODE"), mode_items, 3, &mode))
        lunar_module.mode = static_cast<lm::mode_control>(mode);

    static constexpr const char* guidance_items[] = { "PGNS", "AGS" };
    int guidance = lunar_module.guidance_pgns ? 0 : 1;
    if (switch_row("##guidance", row("GUIDANCE"), guidance_items, 2, &guidance))
        lunar_module.guidance_pgns = guidance == 0;

    const ImRect jets_row(ImVec2(row_left, y), ImVec2(row_right, y + px(34.f)));
    switch_label(drawlist, ImRect(jets_row.Min, ImVec2(jets_row.Max.x, jets_row.Min.y + px(14.f))), "RCS");
    draw_jets(drawlist, ImRect(ImVec2(row_left + label_width, jets_row.Min.y), jets_row.Max), lunar_module.jets());
}

void view::hand_controller(lm::c_lunar_module& lunar_module)
{
    lm::hand_controller_t stick;
    if (celosia::input::focused())
    {
        // a third of the way over, for flying it down in P66; all the way with shift
        const double travel = celosia::input::key_down(celosia::keys::shift) ? 1.0 : 1.0 / 3.0;
        const auto axis = [travel](int negative, int positive) {
            return ((celosia::input::key_down(positive) ? 1.0 : 0.0) - (celosia::input::key_down(negative) ? 1.0 : 0.0)) * travel;
        };
        stick.pitch = axis(celosia::keys::up, celosia::keys::down); // back on the stick, the down arrow, is nose up
        stick.roll = axis(celosia::keys::left, celosia::keys::right);
        stick.yaw = axis('Q', 'E');

        // the rate of descent switch: w up (coming down slower), s down
        if (celosia::input::key_pressed('W', false))
            lunar_module.click_rate_of_descent(1);
        if (celosia::input::key_pressed('S', false))
            lunar_module.click_rate_of_descent(-1);
    }
    lunar_module.hand_controller = stick;
}

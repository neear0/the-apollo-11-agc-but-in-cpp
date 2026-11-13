#pragma once

#include <array>
#include <cmath>
#include <numbers>

// the little vector math the spacecraft needs: 3-vectors and rotation matrices, in doubles
namespace lm
{
    struct vec3_t
    {
        double x = 0.0, y = 0.0, z = 0.0;

        constexpr vec3_t operator+(const vec3_t& o) const { return { x + o.x, y + o.y, z + o.z }; }
        constexpr vec3_t operator-(const vec3_t& o) const { return { x - o.x, y - o.y, z - o.z }; }
        constexpr vec3_t operator-() const { return { -x, -y, -z }; }
        constexpr vec3_t operator*(double s) const { return { x * s, y * s, z * s }; }
        constexpr vec3_t operator/(double s) const { return { x / s, y / s, z / s }; }
        constexpr vec3_t& operator+=(const vec3_t& o) { x += o.x; y += o.y; z += o.z; return *this; }
        constexpr vec3_t& operator-=(const vec3_t& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
        constexpr double operator[](int i) const { return i == 0 ? x : i == 1 ? y : z; }
    };

    constexpr double dot(const vec3_t& a, const vec3_t& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    constexpr vec3_t cross(const vec3_t& a, const vec3_t& b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
    inline double length(const vec3_t& v) { return std::sqrt(dot(v, v)); }

    // a matrix by rows. the rotations here are of coordinate frames: frame_b = m * frame_a takes a vector's components
    // in frame a to its components in frame b
    struct mat3_t
    {
        std::array<vec3_t, 3> rows{ vec3_t{ 1.0, 0.0, 0.0 }, vec3_t{ 0.0, 1.0, 0.0 }, vec3_t{ 0.0, 0.0, 1.0 } };

        constexpr vec3_t operator*(const vec3_t& v) const { return { dot(rows[0], v), dot(rows[1], v), dot(rows[2], v) }; }
        constexpr mat3_t operator*(const mat3_t& o) const
        {
            const mat3_t t = o.transposed();
            mat3_t m;
            for (std::size_t i = 0; i < 3; ++i)
                m.rows[i] = { dot(rows[i], t.rows[0]), dot(rows[i], t.rows[1]), dot(rows[i], t.rows[2]) };
            return m;
        }
        constexpr mat3_t transposed() const
        {
            return { { vec3_t{ rows[0].x, rows[1].x, rows[2].x }, vec3_t{ rows[0].y, rows[1].y, rows[2].y }, vec3_t{ rows[0].z, rows[1].z, rows[2].z } } };
        }
        constexpr double at(int row, int column) const { return rows[static_cast<std::size_t>(row)][column]; }
    };

    // the frame turned by angle (right handed) about axis, a unit vector in the frame's own coordinates
    inline mat3_t frame_rotation(const vec3_t& axis, double angle)
    {
        const double c = std::cos(angle), s = std::sin(angle), t = 1.0 - c;
        const vec3_t& n = axis;
        // the transpose of the rotation of a vector by angle
        return { { vec3_t{ t * n.x * n.x + c,       t * n.x * n.y + s * n.z, t * n.x * n.z - s * n.y },
                   vec3_t{ t * n.x * n.y - s * n.z, t * n.y * n.y + c,       t * n.y * n.z + s * n.x },
                   vec3_t{ t * n.x * n.z + s * n.y, t * n.y * n.z - s * n.x, t * n.z * n.z + c } } };
    }

    inline mat3_t frame_rotation_x(double angle) { return frame_rotation({ 1.0, 0.0, 0.0 }, angle); }
    inline mat3_t frame_rotation_y(double angle) { return frame_rotation({ 0.0, 1.0, 0.0 }, angle); }
    inline mat3_t frame_rotation_z(double angle) { return frame_rotation({ 0.0, 0.0, 1.0 }, angle); }

    // rows made unit length and square to each other again, after many small rotations have worn them
    inline mat3_t orthonormalized(const mat3_t& m)
    {
        const vec3_t x = m.rows[0] / length(m.rows[0]);
        vec3_t y = m.rows[1] - x * dot(x, m.rows[1]);
        y = y / length(y);
        return { { x, y, cross(x, y) } };
    }

    inline constexpr double two_pi = 2.0 * std::numbers::pi;
    inline constexpr double degrees = std::numbers::pi / 180.0;
}

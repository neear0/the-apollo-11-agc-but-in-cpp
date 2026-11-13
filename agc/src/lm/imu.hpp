#pragma once

#include <array>

#include "lm/vector.hpp"

namespace agc
{
    class c_computer;
}

// the inertial measurement unit: a stable member the gyros hold still in space, on three gimbals around it. the
// computer reads the gimbal angles through the cdus (counters 32-34) and the velocity change through the pipas
// (counters 37-41), turns the stable member with gyro torquing pulses (fine align) and drives the gimbals with cdu
// pulses (coarse align). channels 12 and 14 command it, channel 30 has its status
namespace lm
{
    // radians. the outer gimbal is cdu x, the inner cdu y, the middle cdu z
    struct gimbal_angles_t
    {
        double outer = 0.0;
        double inner = 0.0;
        double middle = 0.0;
    };

    // stable member to navigation base (the vehicle's axes): the gimbals turn about y (inner), then z (middle), then
    // x (outer), from the stable member out. gimbal_angles() takes the matrix apart again, the middle in +-90 degrees
    mat3_t gimbal_matrix(const gimbal_angles_t& angles);
    gimbal_angles_t gimbal_angles(const mat3_t& stable_to_body);

    class c_imu
    {
    public:
        // the crew's side
        bool operate = true; // the imu switched on. on, it asks the computer for its turn-on sequence: caged for 90 s
        bool cage = false;   // the cage button: the gimbals driven to zero, whatever the computer says

        // body is inertial to the vehicle's axes, delta_v the velocity change the vehicle's thrust gave it this step,
        // in its own axes (m/s)
        void step(agc::c_computer& computer, const mat3_t& body, const vec3_t& delta_v, double dt);

        const mat3_t& stable_member() const;  // inertial to stable member
        const gimbal_angles_t& gimbals() const;
        bool coarse_align() const;            // the gimbals driven, not held by the gyros
        bool turn_on_request() const;

    private:
        void count_cdus(agc::c_computer& computer, bool zero, double dt);
        void count_pipas(agc::c_computer& computer, const mat3_t& body, const vec3_t& delta_v);

        mat3_t m_stable_member{};
        gimbal_angles_t m_gimbals{};
        gimbal_angles_t m_coarse_target{};   // where coarse align is driving the gimbals
        bool m_coarse = false;
        bool m_was_operating = false;
        bool m_turn_on_request = false;

        std::array<int, 3> m_cdu_counts{};    // what the cdus have counted into the computer, 2^-15 revolutions
        std::array<double, 3> m_cdu_credit{}; // pulses the cdus can send yet, they count at most 6400 a second
        vec3_t m_pipa_residual{};             // velocity change not counted yet, cm/s
    };
}

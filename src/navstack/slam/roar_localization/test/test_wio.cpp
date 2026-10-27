//
// test_wio.cpp
//
// Tests for the WIO (Wheel-Inertial Odometry) module:
//   - First call seeds without producing output
//   - Forward motion produces correct vx
//   - Yaw rotation produces correct dp_world (uses BNO055 yaw, not differential)
//   - Encoder spikes are rejected
//   - v_L and v_R are reported correctly
//   - Pitch produces a non-zero dp_z component
//

#include <iostream>
#include <cmath>
#include "roar_localization/wio.hpp"

using namespace roar;

static int passed = 0;
static int failed = 0;

#define CHECK(name, cond) \
    do { \
        if (cond) { std::cout << "  PASS  " << (name) << "\n"; ++passed; } \
        else       { std::cout << "  FAIL  " << (name) << "\n"; ++failed; } \
    } while(0)

// Standard test rover (Gazebo sim values)
static WIOParams makeParams()
{
    WIOParams p;
    p.wheel_radius     = 0.13;
    p.wheelbase        = 0.709;
    p.pulses_per_rev   = 6.2831853;     // encoder publishes radians
    p.max_vx           = 2.0;
    p.max_delta_pulses = 50.0;
    return p;
}

// Identity orientation quaternion (rover facing world x)
static const Quat Q_IDENTITY = Quat::Identity();

// ─────────────────────────────────────────────────────────────────────────────
// 1. Forward motion: equal encoder ticks on both wheels
// ─────────────────────────────────────────────────────────────────────────────
void test_forward_motion()
{
    std::cout << "\n[1. Forward motion]\n";
    WIO wio(makeParams());

    const double dt = 0.01;

    // First call seeds (no real output expected)
    wio.compute(0.0, 0.0, Q_IDENTITY, dt);

    // Second call: 0.1 rad each side in 10 ms = 10 rad/s
    // surface speed = 10 × 0.13 m = 1.3 m/s
    WIOOutput out = wio.compute(0.1, 0.1, Q_IDENTITY, dt);

    CHECK("valid", out.valid);
    CHECK("vx ≈ 1.3 m/s",   std::abs(out.vx  - 1.3) < 0.01);
    CHECK("v_L ≈ 1.3",       std::abs(out.v_L - 1.3) < 0.01);
    CHECK("v_R ≈ 1.3",       std::abs(out.v_R - 1.3) < 0.01);
    CHECK("dp_x ≈ vx·dt",    std::abs(out.dp_world.x() - 1.3 * dt) < 1e-4);
    CHECK("dp_y ≈ 0",        std::abs(out.dp_world.y()) < 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. Yaw 90°: dp_world should point in world-y direction
// ─────────────────────────────────────────────────────────────────────────────
void test_yaw_90()
{
    std::cout << "\n[2. Yaw 90° → dp in world-y]\n";
    WIO wio(makeParams());

    const double dt = 0.01;
    Quat q_yaw90(Eigen::AngleAxisd(M_PI/2, Vec3::UnitZ()));

    // Seed (note: even first call uses provided quaternion for prev_yaw)
    wio.compute(0.0, 0.0, q_yaw90, dt);
    WIOOutput out = wio.compute(0.1, 0.1, q_yaw90, dt);

    CHECK("valid",    out.valid);
    CHECK("vx ≈ 1.3", std::abs(out.vx - 1.3) < 0.01);

    // With yaw=90° and constant orientation, dp_x ≈ 0, dp_y ≈ vx·dt
    CHECK("dp_x ≈ 0",         std::abs(out.dp_world.x()) < 1e-3);
    CHECK("dp_y ≈ vx·dt",     std::abs(out.dp_world.y() - 1.3 * dt) < 1e-3);
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. Pitch produces dp_z (slope)
// ─────────────────────────────────────────────────────────────────────────────
void test_pitch_dpz()
{
    std::cout << "\n[3. Pitch produces dp_z]\n";
    WIO wio(makeParams());

    const double dt = 0.01;
    Quat q_pitch(Eigen::AngleAxisd(0.3, Vec3::UnitY()));   // nose-down 0.3 rad

    wio.compute(0.0, 0.0, q_pitch, dt);
    WIOOutput out = wio.compute(0.1, 0.1, q_pitch, dt);

    CHECK("valid", out.valid);
    // dp_z = vx · sin(pitch) · dt  (here positive pitch convention from rot_to_euler)
    const double expected_dpz = 1.3 * std::sin(0.3) * dt;
    CHECK("dp_z proportional to sin(pitch)",
          std::abs(out.dp_world.z() - expected_dpz) < 1e-3);
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. Encoder spike rejection
// ─────────────────────────────────────────────────────────────────────────────
void test_spike_rejection()
{
    std::cout << "\n[4. Encoder spike rejection]\n";
    WIO wio(makeParams());

    const double dt = 0.01;
    wio.compute(0.0, 0.0, Q_IDENTITY, dt);

    // Delta = 100 rad >> max_delta_pulses (50.0) → must be rejected
    WIOOutput out = wio.compute(100.0, 100.0, Q_IDENTITY, dt);
    CHECK("invalid on huge spike", !out.valid);
    CHECK("dp_world is zero on rejection",
          out.dp_world.norm() < 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. Differential wheel speeds: v_L ≠ v_R reported individually
// ─────────────────────────────────────────────────────────────────────────────
void test_differential_speeds()
{
    std::cout << "\n[5. Differential wheel speeds]\n";
    WIO wio(makeParams());

    const double dt = 0.01;
    wio.compute(0.0, 0.0, Q_IDENTITY, dt);

    // L moves 0.05 rad, R moves 0.15 rad
    // v_L = 0.05/0.01 × 0.13 = 0.65 m/s
    // v_R = 0.15/0.01 × 0.13 = 1.95 m/s — over max_vx? 1.95 < 2.0 OK
    WIOOutput out = wio.compute(0.05, 0.15, Q_IDENTITY, dt);

    CHECK("valid",        out.valid);
    CHECK("v_L ≈ 0.65",   std::abs(out.v_L - 0.65) < 0.01);
    CHECK("v_R ≈ 1.95",   std::abs(out.v_R - 1.95) < 0.01);
    CHECK("vx is average ≈ 1.3", std::abs(out.vx - 1.3) < 0.01);
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. Reset clears stored state
// ─────────────────────────────────────────────────────────────────────────────
void test_reset()
{
    std::cout << "\n[6. reset() restarts seeding]\n";
    WIO wio(makeParams());

    const double dt = 0.01;
    wio.compute(0.0, 0.0, Q_IDENTITY, dt);
    wio.compute(0.1, 0.1, Q_IDENTITY, dt);   // first real reading

    wio.reset();

    // After reset, this acts as a first-call seed again — the next "real"
    // reading should still be valid, but treating 0.5 → 0.6 as a delta from
    // RESET should be valid (delta = 0.1 < max 50).
    wio.compute(0.5, 0.5, Q_IDENTITY, dt);   // re-seed
    WIOOutput out = wio.compute(0.6, 0.6, Q_IDENTITY, dt);

    CHECK("post-reset works", out.valid);
    CHECK("vx ≈ 1.3 after re-seed", std::abs(out.vx - 1.3) < 0.01);
}

// ─────────────────────────────────────────────────────────────────────────────
int main()
{
    std::cout << "=== WIO unit tests ===\n";

    test_forward_motion();
    test_yaw_90();
    test_pitch_dpz();
    test_spike_rejection();
    test_differential_speeds();
    test_reset();

    std::cout << "\n=== Results: "
              << passed << " passed, " << failed << " failed ===\n";

    return failed == 0 ? 0 : 1;
}

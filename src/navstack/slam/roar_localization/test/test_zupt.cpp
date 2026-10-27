//
// test_zupt.cpp
//
// Tests for the ZuptDetector — multi-condition stationary detection.
//
// Scenarios:
//   1. Idle long enough → stationary
//   2. Idle but not for long enough → NOT stationary
//   3. Wheels moving → never stationary even if IMU quiet
//   4. Gyro spinning → never stationary even if wheels quiet
//   5. Noisy IMU → never stationary
//   6. Reset clears the detector
//

#include <iostream>
#include <cmath>
#include "roar_localization/zupt.hpp"

using namespace roar;

static int passed = 0;
static int failed = 0;

#define CHECK(name, cond) \
    do { \
        if (cond) { std::cout << "  PASS  " << (name) << "\n"; ++passed; } \
        else       { std::cout << "  FAIL  " << (name) << "\n"; ++failed; } \
    } while(0)

// ─────────────────────────────────────────────────────────────────────────────
// Helper: feed N quiet samples and return current detector state
// ─────────────────────────────────────────────────────────────────────────────
static bool feedQuiet(ZuptDetector& d, int n_samples, double dt,
                      double start_time = 0.0)
{
    double t = start_time;
    for (int i = 0; i < n_samples; ++i) {
        d.update(0.0, 0.0, 0.0, Vec3::Zero(), t);
        t += dt;
    }
    return d.isStationary();
}

// ─────────────────────────────────────────────────────────────────────────────
// 1. Long-enough idle period triggers stationary
// ─────────────────────────────────────────────────────────────────────────────
void test_idle_long_enough()
{
    std::cout << "\n[1. Idle long enough → stationary]\n";
    ZuptParams p;
    p.min_stationary_duration = 0.5;
    p.accel_buffer_size       = 10;
    ZuptDetector d(p);

    bool stationary_at_end = feedQuiet(d, 100, 0.01);   // 1 s of quiet
    CHECK("stationary after 1 s of quiet", stationary_at_end);
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. Brief idle does NOT trigger stationary
// ─────────────────────────────────────────────────────────────────────────────
void test_brief_idle()
{
    std::cout << "\n[2. Brief idle below threshold → NOT stationary]\n";
    ZuptParams p;
    p.min_stationary_duration = 0.5;
    p.accel_buffer_size       = 10;
    ZuptDetector d(p);

    // 0.2 s of quiet — half of what's required
    bool stationary = feedQuiet(d, 20, 0.01);
    CHECK("NOT stationary at 0.2 s", !stationary);
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. Wheels moving disqualifies stationary
// ─────────────────────────────────────────────────────────────────────────────
void test_wheels_moving()
{
    std::cout << "\n[3. Wheels moving → NOT stationary]\n";
    ZuptParams p;
    p.min_stationary_duration   = 0.5;
    p.encoder_speed_threshold   = 0.02;
    p.accel_buffer_size         = 10;
    ZuptDetector d(p);

    double t = 0.0;
    for (int i = 0; i < 100; ++i) {
        // v_L = 0.1 m/s — above threshold
        d.update(0.1, 0.1, 0.0, Vec3::Zero(), t);
        t += 0.01;
    }
    CHECK("not stationary when wheels move", !d.isStationary());
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. Yaw rate above threshold disqualifies
// ─────────────────────────────────────────────────────────────────────────────
void test_yaw_rate()
{
    std::cout << "\n[4. Yaw rate above threshold → NOT stationary]\n";
    ZuptParams p;
    p.min_stationary_duration       = 0.5;
    p.angular_velocity_threshold    = 0.05;
    p.accel_buffer_size             = 10;
    ZuptDetector d(p);

    double t = 0.0;
    for (int i = 0; i < 100; ++i) {
        d.update(0.0, 0.0, 0.2, Vec3::Zero(), t);   // spinning at 0.2 rad/s
        t += 0.01;
    }
    CHECK("not stationary when yaw rate high", !d.isStationary());
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. Noisy IMU disqualifies even if wheels and gyro report zero
// ─────────────────────────────────────────────────────────────────────────────
void test_noisy_imu()
{
    std::cout << "\n[5. Noisy IMU → NOT stationary]\n";
    ZuptParams p;
    p.min_stationary_duration   = 0.5;
    p.accel_variance_threshold  = 0.05;     // very tight
    p.accel_buffer_size         = 30;
    ZuptDetector d(p);

    double t = 0.0;
    for (int i = 0; i < 100; ++i) {
        // Alternating accel values produce high variance
        Vec3 a = (i % 2 == 0) ? Vec3( 2.0, 0.0, 0.0)
                              : Vec3(-2.0, 0.0, 0.0);
        d.update(0.0, 0.0, 0.0, a, t);
        t += 0.01;
    }
    CHECK("not stationary with noisy accel", !d.isStationary());
    CHECK("computed variance is large", d.currentAccelVariance() > 0.1);
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. Transitions: quiet → moving → quiet (must re-arm correctly)
// ─────────────────────────────────────────────────────────────────────────────
void test_transitions()
{
    std::cout << "\n[6. Transitions re-arm timer correctly]\n";
    ZuptParams p;
    p.min_stationary_duration = 0.5;
    p.accel_buffer_size       = 10;
    ZuptDetector d(p);

    double t = 0.0;

    // Phase 1: 1 s quiet — should be stationary
    for (int i = 0; i < 100; ++i) {
        d.update(0.0, 0.0, 0.0, Vec3::Zero(), t);
        t += 0.01;
    }
    bool stat_after_phase1 = d.isStationary();

    // Phase 2: 1 s moving — must lose stationary
    for (int i = 0; i < 100; ++i) {
        d.update(0.3, 0.3, 0.0, Vec3::Zero(), t);
        t += 0.01;
    }
    bool stat_during_phase2 = d.isStationary();

    // Phase 3: 0.2 s quiet — not enough yet
    for (int i = 0; i < 20; ++i) {
        d.update(0.0, 0.0, 0.0, Vec3::Zero(), t);
        t += 0.01;
    }
    bool stat_brief_quiet_again = d.isStationary();

    // Phase 4: 0.5 s quiet — should be stationary again
    for (int i = 0; i < 50; ++i) {
        d.update(0.0, 0.0, 0.0, Vec3::Zero(), t);
        t += 0.01;
    }
    bool stat_long_quiet_again = d.isStationary();

    CHECK("stationary after first quiet",   stat_after_phase1);
    CHECK("not stationary while moving",   !stat_during_phase2);
    CHECK("not stationary after brief re-quiet", !stat_brief_quiet_again);
    CHECK("stationary after long re-quiet", stat_long_quiet_again);
}

// ─────────────────────────────────────────────────────────────────────────────
// 7. reset() clears the detector
// ─────────────────────────────────────────────────────────────────────────────
void test_reset()
{
    std::cout << "\n[7. reset() clears state]\n";
    ZuptParams p;
    p.min_stationary_duration = 0.3;
    p.accel_buffer_size       = 10;
    ZuptDetector d(p);

    feedQuiet(d, 50, 0.01);
    CHECK("stationary before reset", d.isStationary());

    d.reset();
    CHECK("not stationary right after reset", !d.isStationary());

    // Needs another full window to trigger again
    feedQuiet(d, 5, 0.01, 1.0);
    CHECK("still not stationary after short feed post-reset", !d.isStationary());
}

// ─────────────────────────────────────────────────────────────────────────────
int main()
{
    std::cout << "=== ZuptDetector unit tests ===\n";

    test_idle_long_enough();
    test_brief_idle();
    test_wheels_moving();
    test_yaw_rate();
    test_noisy_imu();
    test_transitions();
    test_reset();

    std::cout << "\n=== Results: "
              << passed << " passed, " << failed << " failed ===\n";

    return failed == 0 ? 0 : 1;
}

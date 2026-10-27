//
// test_wvo.cpp
//
// Tests for the WVO (Wheel-Visual Odometry) module:
//   - First call seeds without producing a real output
//   - Normal motion: outlier rejection accepts, blend works, sliding window
//   - Outlier rejection: large jump rejected, jump_ratio set to 5.0
//   - jump_ratio: agreement → ~1, divergence → larger
//   - Window-size = 1 (sim passthrough)
//   - prev_p_zed_ advances even on rejection (recovery behaviour)
//

#include <iostream>
#include <cmath>
#include "roar_localization/wvo.hpp"

using namespace roar;

static int passed = 0;
static int failed = 0;

#define CHECK(name, cond) \
    do { \
        if (cond) { std::cout << "  PASS  " << (name) << "\n"; ++passed; } \
        else       { std::cout << "  FAIL  " << (name) << "\n"; ++failed; } \
    } while(0)

static WVOParams makeParams(int window = 5,
                            double w_wo = 0.3,
                            double max_jump = 0.5)
{
    WVOParams p;
    p.max_jump        = max_jump;
    p.window_size     = window;
    p.w_wo            = w_wo;
    p.min_window_fill = std::min(3, window);
    return p;
}

// ─────────────────────────────────────────────────────────────────────────────
// 1. First call seeds, no real output ready
// ─────────────────────────────────────────────────────────────────────────────
void test_first_call_seeds()
{
    std::cout << "\n[1. First call seeds]\n";
    WVO wvo(makeParams());

    WVOOutput out = wvo.compute(Vec3(1.0, 2.0, 0.0), Vec3::Zero());
    CHECK("valid (no rejection)", out.valid);
    CHECK("not ready yet (window empty)", !out.ready);
    CHECK("z == seed pose", (out.z - Vec3(1.0, 2.0, 0.0)).norm() < 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. Outlier rejection: large jump → invalid
// ─────────────────────────────────────────────────────────────────────────────
void test_outlier_rejection()
{
    std::cout << "\n[2. Outlier rejection]\n";
    WVO wvo(makeParams(5, 0.0, 0.5));   // max_jump = 0.5

    wvo.compute(Vec3(0.0, 0.0, 0.0), Vec3::Zero());            // seed
    // Move 10 m in one step — way above max_jump
    WVOOutput out = wvo.compute(Vec3(10.0, 0.0, 0.0), Vec3::Zero());

    CHECK("invalid on huge jump", !out.valid);
    CHECK("jump_ratio = 5.0 (clamped max)",
          std::abs(out.jump_ratio - 5.0) < 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. Normal motion fills window over multiple frames
// ─────────────────────────────────────────────────────────────────────────────
void test_window_fills()
{
    std::cout << "\n[3. Sliding window fills]\n";
    WVOParams p = makeParams(5, 0.0, 0.5);
    p.min_window_fill = 3;
    WVO wvo(p);

    wvo.compute(Vec3(0.0, 0.0, 0.0), Vec3::Zero());   // seed

    bool ready_at[6] = {false};
    for (int i = 0; i < 6; ++i) {
        const double x = 0.05 * (i + 1);
        WVOOutput o = wvo.compute(Vec3(x, 0.0, 0.0), Vec3(0.05, 0.0, 0.0));
        ready_at[i] = o.ready;
    }

    // Frame 0: window has 1 entry < 3 → not ready
    CHECK("frame 0: not ready", !ready_at[0]);
    CHECK("frame 1: not ready", !ready_at[1]);
    CHECK("frame 2: ready",      ready_at[2]);
    CHECK("frame 5: ready",      ready_at[5]);
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. Window-size = 1 passes ZED through directly (sim mode)
// ─────────────────────────────────────────────────────────────────────────────
void test_passthrough()
{
    std::cout << "\n[4. window_size=1 → passthrough]\n";
    WVOParams p = makeParams(1, 0.0, 5.0);
    p.min_window_fill = 1;
    WVO wvo(p);

    wvo.compute(Vec3(0.0, 0.0, 0.0), Vec3::Zero());   // seed
    WVOOutput o = wvo.compute(Vec3(0.05, 0.0, 0.0), Vec3(0.05, 0.0, 0.0));

    CHECK("valid", o.valid);
    CHECK("ready immediately", o.ready);
    // With w_wo = 0, p_wvo_ = prev_p_wvo_ + dp_VO, then averaged over the
    // single window entry. After one step from origin, p_wvo_ = 0.05.
    CHECK("z ≈ 0.05 (passthrough)", std::abs(o.z.x() - 0.05) < 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. jump_ratio: VO and WO agree → ratio ~1
// ─────────────────────────────────────────────────────────────────────────────
void test_jump_ratio_agreement()
{
    std::cout << "\n[5. jump_ratio: ZED and WO agree → ~1]\n";
    WVOParams p = makeParams(1, 0.0, 1.0);
    p.min_window_fill = 1;
    WVO wvo(p);

    wvo.compute(Vec3(0.0, 0.0, 0.0), Vec3::Zero());   // seed

    // VO moved 0.1 m, WO moved 0.1 m → ratio ~ 0.1/(0.1 + 1e-3) ≈ 0.99
    WVOOutput o = wvo.compute(Vec3(0.1, 0.0, 0.0), Vec3(0.1, 0.0, 0.0));

    CHECK("valid", o.valid);
    CHECK("jump_ratio near 1.0", std::abs(o.jump_ratio - 1.0) < 0.05);
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. jump_ratio: VO and WO disagree (ZED drifting) → large ratio
// ─────────────────────────────────────────────────────────────────────────────
void test_jump_ratio_divergence()
{
    std::cout << "\n[6. jump_ratio: ZED says 0.4m, WO says 0.1m → ratio ~4]\n";
    WVOParams p = makeParams(1, 0.0, 1.0);   // max_jump big enough
    p.min_window_fill = 1;
    WVO wvo(p);

    wvo.compute(Vec3(0.0, 0.0, 0.0), Vec3::Zero());

    WVOOutput o = wvo.compute(Vec3(0.4, 0.0, 0.0), Vec3(0.1, 0.0, 0.0));
    CHECK("valid (under max_jump)", o.valid);
    CHECK("jump_ratio > 3", o.jump_ratio > 3.0);
    CHECK("jump_ratio ≤ 5 (clamped)", o.jump_ratio <= 5.0 + 1e-9);
}

// ─────────────────────────────────────────────────────────────────────────────
// 7. prev_p_zed_ advances even on rejection (recovery)
//
// If ZED jumps wildly then returns to a sensible position, WVO should:
//   - Reject the wild frame
//   - Update prev_p_zed_ to the wild value (so the next diff is small again)
//   - Accept the next sensible frame
// ─────────────────────────────────────────────────────────────────────────────
void test_recovery_after_rejection()
{
    std::cout << "\n[7. prev_p_zed advances on rejection]\n";
    WVOParams p = makeParams(1, 0.0, 0.5);
    p.min_window_fill = 1;
    WVO wvo(p);

    wvo.compute(Vec3(0.0, 0.0, 0.0), Vec3::Zero());          // seed at 0

    // Reject wildly far frame
    WVOOutput o_bad = wvo.compute(Vec3(10.0, 0.0, 0.0), Vec3(0.1, 0.0, 0.0));
    CHECK("rejected wild frame", !o_bad.valid);

    // Next frame: 10.05 — only 5cm from previous (wild) ZED reading.
    // Since prev_p_zed_ advanced to 10.0, this should be accepted.
    WVOOutput o_recover = wvo.compute(Vec3(10.05, 0.0, 0.0), Vec3(0.05, 0.0, 0.0));
    CHECK("accepts frame after recovery", o_recover.valid);
}

// ─────────────────────────────────────────────────────────────────────────────
int main()
{
    std::cout << "=== WVO unit tests ===\n";

    test_first_call_seeds();
    test_outlier_rejection();
    test_window_fills();
    test_passthrough();
    test_jump_ratio_agreement();
    test_jump_ratio_divergence();
    test_recovery_after_rejection();

    std::cout << "\n=== Results: "
              << passed << " passed, " << failed << " failed ===\n";

    return failed == 0 ? 0 : 1;
}

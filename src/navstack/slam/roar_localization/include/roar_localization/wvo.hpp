#pragma once

#include "roar_localization/math_utils.hpp"
#include <deque>

namespace roar {

// ─────────────────────────────────────────────────────────────────────────────
// WVOParams
// ─────────────────────────────────────────────────────────────────────────────
struct WVOParams
{
    // Maximum plausible frame-to-frame position jump [m].
    // If the ZED2i moves more than this between two consecutive frames,
    // it has lost tracking. The frame is rejected entirely.
    double max_jump = 0.5;

    // Sliding window size for the mean filter [frames].
    // Averages the last N WVO positions to suppress VO jitter.
    // Larger N = smoother but more latency. 5 frames ≈ 0.17 s at 30 Hz.
    int window_size = 5;

    // Blend weight for WO (dead-reckoning) vs VO (visual) delta.
    // dp_blend = w_wo * dp_WO  +  (1 - w_wo) * dp_VO
    // 0.3 means: 30% encoder dead-reckoning, 70% visual odometry.
    double w_wo = 0.3;

    // Minimum number of frames in the window before output is trusted.
    int min_window_fill = 3;
};

// ─────────────────────────────────────────────────────────────────────────────
// WVOOutput
// ─────────────────────────────────────────────────────────────────────────────
struct WVOOutput
{
    Vec3 z;           // filtered position measurement [m] world frame
    bool valid;       // false if this frame was rejected (tracking lost)
    bool ready;       // false if the window is not yet full enough to trust

    // ── NEW: jump_ratio (needed by VO-FIS) ────────────────────────────────────
    // jump_ratio = |dp_VO| / (|dp_WO| + ε)
    // ~1.0 → ZED and encoders agree (healthy)
    // >> 1.0 → ZED is moving much faster than encoders (ZED drifting)
    // Set to 0.0 on first call or when dp_WO is zero (no encoder data).
    // Set to 5.0 (clamped max) on outlier rejection (tracking lost).
    double jump_ratio;
};

// ─────────────────────────────────────────────────────────────────────────────
// WVO — Wheel-Visual Odometry
//
// Converts the raw ZED2i absolute pose into a clean filtered position
// measurement z that feeds the IESKF update step.
// Also computes jump_ratio for the VO-FIS.
//
// Three processing stages:
//   1. Outlier rejection  — drops frames where ZED lost tracking
//   2. WO+VO blend        — anchors VO delta to encoder dead-reckoning scale
//   3. Sliding window     — smooths jitter from lighting / texture changes
//
// Usage:
//   WVO wvo(params);
//
//   // In ZED2i callback:
//   WVOOutput out = wvo.compute(p_zed, dp_wo);
//   if (out.valid && out.ready) {
//       double k_V = vo_fis.evaluate(zed_confidence, out.jump_ratio);
//       ieskf.update(out.z, k_V);
//   }
// ─────────────────────────────────────────────────────────────────────────────
class WVO
{
public:
    explicit WVO(const WVOParams& params = WVOParams{});

    // Reset internal state (call on startup or after long pause).
    void reset();

    // compute() — call once per ZED2i pose message (~30 Hz).
    //
    // p_zed   : raw absolute position from ZED2i  [m]  world frame
    //           (already transformed to base_link by the node)
    //
    // dp_wo   : dead-reckoning position delta since last ZED frame  [m]
    //           world frame, accumulated from WIO between ZED callbacks.
    //           Pass Vec3::Zero() if encoders are not yet available.
    WVOOutput compute(const Vec3& p_zed, const Vec3& dp_wo);

    const WVOParams& getParams() const { return params_; }

    int windowFill() const { return static_cast<int>(window_.size()); }

private:
    WVOParams params_;

    Vec3 prev_p_zed_;
    Vec3 p_wvo_;
    bool first_call_;

    std::deque<Vec3> window_;
};

} // namespace roar

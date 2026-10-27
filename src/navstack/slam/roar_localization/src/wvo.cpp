#include "roar_localization/wvo.hpp"

namespace roar {

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / reset
// ─────────────────────────────────────────────────────────────────────────────
WVO::WVO(const WVOParams& params)
    : params_(params)
{
    reset();
}

void WVO::reset()
{
    prev_p_zed_ = Vec3::Zero();
    p_wvo_      = Vec3::Zero();
    first_call_ = true;
    window_.clear();
}

// ─────────────────────────────────────────────────────────────────────────────
// compute()
//
// Called at ~30 Hz whenever the ZED2i publishes a new pose.
//
// Three processing stages:
//   Stage 1 — Outlier rejection
//   Stage 2 — WO + VO blend
//   Stage 3 — Sliding window mean filter
//
// CHANGE from original:
//   jump_ratio is now computed and stored in WVOOutput.
//   jump_ratio = |dp_VO| / (|dp_WO| + ε)
//
//   This ratio tells the VO-FIS how much the visual delta compares to the
//   encoder dead-reckoning delta:
//     ~1.0  → ZED and encoders agree → ZED is reliable
//     >> 1  → ZED says rover moved much further than encoders → ZED drifting
//     0.0   → on first call or when dp_wo is exactly zero
//     5.0   → clamped max, set on outlier rejection (tracking lost)
// ─────────────────────────────────────────────────────────────────────────────
WVOOutput WVO::compute(const Vec3& p_zed, const Vec3& dp_wo)
{
    WVOOutput out;
    out.valid      = true;
    out.ready      = false;
    out.jump_ratio = 0.0;   // default for first call

    // ── First call: seed from ZED pose ───────────────────────────────────────
    if (first_call_) {
        prev_p_zed_ = p_zed;
        p_wvo_      = p_zed;
        first_call_ = false;
        out.z = p_wvo_;
        return out;
    }

    // ── Stage 1: Outlier rejection ────────────────────────────────────────────
    const Vec3   dp_VO     = p_zed - prev_p_zed_;
    const double jump_dist = dp_VO.norm();

    // Compute jump_ratio regardless of acceptance/rejection.
    // Use 1e-3 epsilon to avoid division by zero when dp_wo is tiny.
    const double dp_wo_norm    = dp_wo.norm();
    const double jump_ratio_raw = jump_dist / (dp_wo_norm + 1e-3);
    out.jump_ratio = std::min(jump_ratio_raw, 5.0);   // clamp to VO-FIS universe [0,5]

    if (jump_dist > params_.max_jump) {
        // ZED lost visual tracking (dust, glare, fast motion).
        // Do NOT update p_wvo_ — coast on the last good position.
        // Advance prev_p_zed_ so the next good frame diffs correctly.
        prev_p_zed_    = p_zed;
        out.valid      = false;
        out.jump_ratio = 5.0;    // maximum distrust for VO-FIS
        out.z          = p_wvo_; // return last good estimate
        return out;
    }

    // Accepted frame. Advance stored previous pose.
    prev_p_zed_ = p_zed;

    // ── Stage 2: WO + VO blend ────────────────────────────────────────────────
    // The blend anchors the VO delta to the encoder scale. If ZED is slowly
    // drifting but the jump is below max_jump, the WO component limits how far
    // p_wvo_ can move from the encoder estimate.
    // dp_wo is Vec3::Zero() when encoders are unavailable — the blend reduces
    // to pure VO in that case.
    const Vec3 dp_blend = params_.w_wo * dp_wo
                        + (1.0 - params_.w_wo) * dp_VO;
    p_wvo_ += dp_blend;

    // ── Stage 3: Sliding window mean filter ───────────────────────────────────
    window_.push_back(p_wvo_);

    while (static_cast<int>(window_.size()) > params_.window_size) {
        window_.pop_front();
    }

    Vec3 z_mean = Vec3::Zero();
    for (const Vec3& pos : window_) z_mean += pos;
    z_mean /= static_cast<double>(window_.size());

    out.ready = (static_cast<int>(window_.size()) >= params_.min_window_fill);
    out.z     = z_mean;

    return out;
}

} // namespace roar

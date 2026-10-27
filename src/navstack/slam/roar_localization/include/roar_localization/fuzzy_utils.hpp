#pragma once

// ═════════════════════════════════════════════════════════════════════════════
// fuzzy_utils.hpp
// Self-contained Zero-Order Sugeno Fuzzy Inference System.
// No external dependencies beyond C++17 standard library.
//
// ─────────────────────────────────────────────────────────────────────────────
// FUZZY LOGIC — WHAT IT IS AND WHY WE USE IT
// ─────────────────────────────────────────────────────────────────────────────
//
// Classical (crisp) logic asks: "Is the wheel slip HIGH?" → yes/no (1 or 0).
// The problem: a slip_ratio of 0.36 is not dramatically different from 0.35,
// yet a hard threshold would treat them completely differently. This creates
// sharp, jittery behavior in the filter.
//
// Fuzzy logic asks instead: "HOW MUCH is the wheel slip HIGH?" → a degree
// in [0.0, 1.0]. A slip_ratio of 0.36 might be 70% "Medium" and 30% "High"
// simultaneously. Both membership labels fire in proportion, producing a
// smooth, continuous output.
//
// ─────────────────────────────────────────────────────────────────────────────
// THE THREE COMPONENTS OF A FIS
// ─────────────────────────────────────────────────────────────────────────────
//
// 1. MEMBERSHIP FUNCTIONS (MFs)
//    Each MF answers: "how much does value x belong to label L?"
//    Example for slip_ratio:
//      LOW    = lshoulder(x, 0.0, 0.15)  →  1.0 at x=0, falls to 0 at x=0.15
//      MEDIUM = trimf(x,  0.0, 0.25, 0.40) →  0 at edges, peaks at 0.25
//      HIGH   = rshoulder(x, 0.35, 1.0)  →  0 at x=0.35, rises to 1 at x=1.0
//
//    At x=0.30: LOW=0.0, MEDIUM=0.67, HIGH=0.0  → clearly Medium
//    At x=0.38: LOW=0.0, MEDIUM=0.15, HIGH=0.09 → mostly High, a bit Medium
//
// 2. RULES
//    "IF slip is HIGH THEN k_Q = 10.0"
//    "IF slip is MEDIUM AND omega is HIGH THEN k_Q = 8.5"
//    Each rule fires with strength = product of its antecedent MF values.
//    (Product t-norm chosen over min because it is smooth and differentiable.)
//
// 3. INFERENCE — ZERO-ORDER SUGENO
//    Each rule has a CONSTANT (crisp) output value — not a fuzzy set.
//    Final output = weighted average: Σ(w_i × c_i) / Σ(w_i)
//    where w_i = firing strength of rule i, c_i = constant output of rule i.
//
//    WHY SUGENO (not Mamdani)?
//    Mamdani rules output fuzzy sets that need a centroid defuzzification
//    (numerically expensive loop). Sugeno defuzzifies analytically in O(n_rules).
//    Better for real-time embedded use (100 Hz predict loop).
//
// ─────────────────────────────────────────────────────────────────────────────
// THE TWO FIS SYSTEMS IN THIS FILE
// ─────────────────────────────────────────────────────────────────────────────
//
// WO-FIS (Wheel-Odometry FIS):
//   Inputs:  slip_ratio [0,1], |ω_z| normalized [0,1], vx normalized [0,1]
//   Output:  k_Q ∈ [1, 10]  — multiplier for process noise Q_v
//   Effect:  When wheels slip → inflate Q → filter relies more on ZED update
//
// VO-FIS (Visual-Odometry FIS):
//   Inputs:  ZED confidence [0,100], jump_ratio [0,5]
//   Output:  k_V ∈ [1, 100]  — multiplier for observation noise V
//   Effect:  When ZED is unreliable → inflate V → filter relies more on WIO
//
// ═════════════════════════════════════════════════════════════════════════════

#include <vector>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace roar {
namespace fuzzy {

// ─────────────────────────────────────────────────────────────────────────────
// MEMBERSHIP FUNCTIONS
// Each returns a value in [0.0, 1.0].
// ─────────────────────────────────────────────────────────────────────────────

// Left-shoulder MF: 1.0 for x ≤ b, linearly drops to 0.0 at x = c
//
//   1 ──────*
//            \
//             \
//   0          *──────
//          b   c
inline double lshoulder(double x, double b, double c)
{
    if (x <= b) return 1.0;
    if (x >= c) return 0.0;
    return (c - x) / (c - b);
}

// Triangular MF: 0 at a and c, peaks at 1.0 at b
//
//   1          *
//             / \
//            /   \
//   0 ──────*     *──────
//           a  b  c
inline double trimf(double x, double a, double b, double c)
{
    if (x <= a || x >= c) return 0.0;
    if (x < b) return (x - a) / (b - a);
    if (x == b) return 1.0;
    return (c - x) / (c - b);
}

// Right-shoulder MF: 0.0 for x ≤ a, linearly rises to 1.0 at x = b
//
//   1               *──────
//                  /
//                 /
//   0 ────────*──*
//             a  b
inline double rshoulder(double x, double a, double b)
{
    if (x <= a) return 0.0;
    if (x >= b) return 1.0;
    return (x - a) / (b - a);
}

// ─────────────────────────────────────────────────────────────────────────────
// MembershipFunc — stores one MF as a type + parameters (avoids std::function
// heap overhead, safe for real-time use after construction)
// ─────────────────────────────────────────────────────────────────────────────
struct MembershipFunc
{
    enum class Type { LEFT_SHOULDER, TRIMF, RIGHT_SHOULDER } type;
    double p0 = 0.0;   // for LEFT_SHOULDER: b;  TRIMF: a;  RIGHT_SHOULDER: a
    double p1 = 0.0;   // for LEFT_SHOULDER: c;  TRIMF: b;  RIGHT_SHOULDER: b
    double p2 = 0.0;   // for TRIMF: c (ignored by shoulder types)

    double eval(double x) const
    {
        switch (type) {
            case Type::LEFT_SHOULDER:  return lshoulder(x, p0, p1);
            case Type::TRIMF:          return trimf(x, p0, p1, p2);
            case Type::RIGHT_SHOULDER: return rshoulder(x, p0, p1);
        }
        return 0.0;
    }

    // Factory helpers (use these instead of constructing directly)
    static MembershipFunc makeLeft  (double b, double c)
    { return {Type::LEFT_SHOULDER, b, c, 0.0}; }

    static MembershipFunc makeTriangle(double a, double b, double c)
    { return {Type::TRIMF, a, b, c}; }

    static MembershipFunc makeRight (double a, double b)
    { return {Type::RIGHT_SHOULDER, a, b, 0.0}; }
};

// ─────────────────────────────────────────────────────────────────────────────
// SugenoRule — one IF-THEN rule
//
// antecedents[i] = index into inputs_[i].mfs
//                  -1 = "don't care" (MF always fires at 1.0)
// output         = constant crisp output for this rule
// ─────────────────────────────────────────────────────────────────────────────
struct SugenoRule
{
    std::vector<int> antecedents;
    double           output = 0.0;
};

// ─────────────────────────────────────────────────────────────────────────────
// SugenoFIS — generic zero-order Sugeno Fuzzy Inference System
// ─────────────────────────────────────────────────────────────────────────────
class SugenoFIS
{
public:
    // One input variable: a collection of membership functions.
    // Add inputs in order; rules reference inputs by index.
    void addInput(std::vector<MembershipFunc> mfs)
    {
        inputs_.push_back(std::move(mfs));
    }

    void addRule(SugenoRule rule)
    {
        if (static_cast<int>(rule.antecedents.size()) !=
            static_cast<int>(inputs_.size()))
        {
            throw std::invalid_argument(
                "SugenoFIS: rule antecedent count must equal input count");
        }
        rules_.push_back(std::move(rule));
    }

    // Evaluate the FIS for a given set of crisp input values.
    // Returns the defuzzified (weighted average) output.
    // If all rule strengths are zero, returns fallback_output.
    double evaluate(const std::vector<double>& vals,
                    double fallback_output = 1.0) const
    {
        if (vals.size() != inputs_.size()) {
            throw std::invalid_argument(
                "SugenoFIS: input value count must equal input count");
        }

        double numer = 0.0;
        double denom = 0.0;

        for (const auto& rule : rules_)
        {
            // Product t-norm: firing strength = product of all MF values
            double w = 1.0;
            for (size_t i = 0; i < inputs_.size(); ++i)
            {
                int mf_idx = rule.antecedents[i];
                if (mf_idx < 0) continue;  // don't care — contributes 1.0
                w *= inputs_[i][mf_idx].eval(vals[i]);
                if (w < 1e-12) { w = 0.0; break; }  // early exit
            }
            numer += w * rule.output;
            denom += w;
        }

        if (denom < 1e-12) return fallback_output;
        return numer / denom;
    }

    int numInputs() const { return static_cast<int>(inputs_.size()); }
    int numRules()  const { return static_cast<int>(rules_.size());  }

private:
    std::vector<std::vector<MembershipFunc>> inputs_;
    std::vector<SugenoRule>                  rules_;
};

// ═════════════════════════════════════════════════════════════════════════════
// WO-FIS — Wheel-Odometry Fuzzy Inference System
// ═════════════════════════════════════════════════════════════════════════════
//
// WHAT IT DOES:
//   Monitors encoder reliability and inflates the IESKF velocity process noise
//   Q_v when wheel slip or aggressive dynamics are detected.
//
// PHYSICAL INTUITION:
//   The IESKF predict step advances the nominal position via dp_world (from
//   encoder dead-reckoning). If the wheels slip, dp_world is wrong — the rover
//   thinks it moved X metres but it actually moved less (or sideways).
//   Inflating Q_v (via k_Q > 1) widens the uncertainty ellipsoid around the
//   predicted position. This makes the Kalman gain K = P·H'·(H·P·H'+V)⁻¹
//   larger — so the ZED2i update correction moves the state estimate further
//   toward the visual measurement and away from the corrupt dead-reckoning.
//
// INPUTS (all pre-normalized to [0,1] before calling evaluate()):
//   slip_ratio   = |v_R - v_L| / (|v_R| + |v_L| + ε)
//   omega_z_norm = |ω_z| / max_omega_z
//   vx_norm      = vx   / max_vx
//
// OUTPUT:
//   k_Q ∈ [k_Q_min, k_Q_max]   (applied only to Q_v, not Q_ba or Q_bw)
//
// MEMBERSHIP FUNCTIONS (on [0,1] range for slip, omega_norm, vx_norm):
//
//   slip:
//     LOW    = left shoulder,  0→1 → flat, starts falling at slip_low_upper
//     MEDIUM = triangle,       peak at slip_med_peak
//     HIGH   = right shoulder, rises from slip_high_lower → 1.0
//
//   omega_z_norm: symmetric structure (LOW=0.0–0.3, MED=0.0–1.0, HIGH=0.7–1.0)
//   vx_norm:      symmetric structure (LOW=0.0–0.4, MED=0.0–1.0, HIGH=0.6–1.0)
//
// RULE SUMMARY (10 rules):
//   slip=HIGH                          → k_Q = MAX          (always inflate on heavy slip)
//   slip=MED  + omega=HIGH             → k_Q = MAX×0.85     (turning + partial slip)
//   slip=MED  + vx=HIGH                → k_Q = MID+         (fast + partial slip)
//   slip=LOW  + omega=HIGH             → k_Q = MID          (fast turn, stable so far)
//   slip=MED  + omega=MED              → k_Q = MID          (moderate conditions)
//   slip=LOW  + omega=MED              → k_Q = MID-         (gentle turn, clean)
//   slip=MED  + omega=LOW + vx=LOW     → k_Q = MID-         (slow + partial slip)
//   slip=LOW  + omega=LOW + vx=LOW     → k_Q = MIN          (ideal: stationary or slow clean)
//   slip=LOW  + omega=LOW + vx=HIGH    → k_Q = MIN+         (fast but clean)
//   slip=LOW  + omega=MED + vx=MED     → k_Q = MID-         (moderate speed + gentle turn)
// ═════════════════════════════════════════════════════════════════════════════

struct WOFISParams
{
    // Output range
    double k_Q_min = 1.0;    // no inflation: encoders fully trusted
    double k_Q_max = 10.0;   // maximum inflation: heavy slip, ignore dead-reckoning

    // Slip membership function parameters (universe [0, 1])
    double slip_low_upper   = 0.15;   // slip fully "Low"  below this
    double slip_med_peak    = 0.25;   // "Medium" peaks here
    double slip_high_lower  = 0.35;   // slip starts becoming "High" above this

    // Omega (normalised [0,1]) membership
    double omega_low_upper  = 0.30;
    double omega_med_peak   = 0.50;
    double omega_high_lower = 0.70;

    // Vx (normalised [0,1]) membership
    double vx_low_upper     = 0.40;
    double vx_med_peak      = 0.55;
    double vx_high_lower    = 0.60;
};

class WOFIS
{
public:
    explicit WOFIS(const WOFISParams& params = WOFISParams{})
        : params_(params)
    {
        buildFIS();
    }

    // Evaluate k_Q.
    // slip_ratio   ∈ [0, 1]  — clamped internally
    // omega_z_norm ∈ [0, 1]  — |ω_z| / max_omega_z
    // vx_norm      ∈ [0, 1]  — vx / max_vx
    double evaluate(double slip_ratio,
                    double omega_z_norm,
                    double vx_norm) const
    {
        slip_ratio   = std::clamp(slip_ratio,   0.0, 1.0);
        omega_z_norm = std::clamp(omega_z_norm, 0.0, 1.0);
        vx_norm      = std::clamp(vx_norm,      0.0, 1.0);

        return fis_.evaluate({slip_ratio, omega_z_norm, vx_norm},
                             params_.k_Q_min);
    }

    const WOFISParams& getParams() const { return params_; }
    const SugenoFIS&   getFIS()    const { return fis_; }

private:
    void buildFIS()
    {
        // Copy thresholds to local constants for lambda-capture safety
        const double sl  = params_.slip_low_upper;
        const double sm  = params_.slip_med_peak;
        const double sh  = params_.slip_high_lower;
        const double ol  = params_.omega_low_upper;
        const double om  = params_.omega_med_peak;
        const double oh  = params_.omega_high_lower;
        const double vl  = params_.vx_low_upper;
        const double vm  = params_.vx_med_peak;
        const double vh  = params_.vx_high_lower;

        const double Kn  = params_.k_Q_min;
        const double Kx  = params_.k_Q_max;
        const double Km  = 0.5 * (Kn + Kx);   // midpoint

        // ── Input 0: slip_ratio ───────────────────────────────────────────────
        // Index 0 = LOW, 1 = MEDIUM, 2 = HIGH
        fis_.addInput({
            MembershipFunc::makeLeft    (0.0, sl),          // 0: LOW
            MembershipFunc::makeTriangle(0.0, sm, sh + 0.05), // 1: MEDIUM
            MembershipFunc::makeRight   (sl,  1.0)          // 2: HIGH
        });

        // ── Input 1: omega_z_norm ─────────────────────────────────────────────
        // Index 0 = LOW, 1 = MEDIUM, 2 = HIGH
        fis_.addInput({
            MembershipFunc::makeLeft    (0.0, ol),           // 0: LOW
            MembershipFunc::makeTriangle(0.0, om, 1.0),      // 1: MEDIUM
            MembershipFunc::makeRight   (oh,  1.0)           // 2: HIGH
        });

        // ── Input 2: vx_norm ──────────────────────────────────────────────────
        // Index 0 = LOW, 1 = MEDIUM, 2 = HIGH
        fis_.addInput({
            MembershipFunc::makeLeft    (0.0, vl),           // 0: LOW
            MembershipFunc::makeTriangle(0.0, vm, 1.0),      // 1: MEDIUM
            MembershipFunc::makeRight   (vh,  1.0)           // 2: HIGH
        });

        // ── Rules ─────────────────────────────────────────────────────────────
        // Antecedents: [slip_idx, omega_idx, vx_idx]   (-1 = don't care)
        //
        // Rule 1: Heavy slip → always inflate (regardless of omega, vx)
        fis_.addRule({{2, -1, -1}, Kx});

        // Rule 2: Partial slip + aggressive turning → near-max inflate
        fis_.addRule({{1, 2, -1}, Kx * 0.85});

        // Rule 3: Partial slip + high speed → moderate-high inflate
        fis_.addRule({{1, -1, 2}, Km + (Kx - Km) * 0.40});

        // Rule 4: Clean + fast turn → moderate inflate (preemptive)
        fis_.addRule({{0, 2, -1}, Km});

        // Rule 5: Partial slip + moderate turn → moderate inflate
        fis_.addRule({{1, 1, -1}, Km});

        // Rule 6: Clean + moderate turn → small inflate
        fis_.addRule({{0, 1, -1}, Kn + (Km - Kn) * 0.5});

        // Rule 7: Partial slip + slow + straight → small inflate
        fis_.addRule({{1, 0, 0}, Kn + (Km - Kn) * 0.6});

        // Rule 8: Clean + slow + straight → no inflation (ideal conditions)
        fis_.addRule({{0, 0, 0}, Kn});

        // Rule 9: Clean + fast + straight → tiny inflate (speed alone adds uncertainty)
        fis_.addRule({{0, 0, 2}, Kn + (Km - Kn) * 0.3});

        // Rule 10: Clean + moderate speed + gentle turn
        fis_.addRule({{0, 1, 1}, Kn + (Km - Kn) * 0.4});
    }

    WOFISParams params_;
    SugenoFIS   fis_;
};

// ═════════════════════════════════════════════════════════════════════════════
// VO-FIS — Visual-Odometry Fuzzy Inference System
// ═════════════════════════════════════════════════════════════════════════════
//
// WHAT IT DOES:
//   Monitors ZED2i tracking quality and inflates the observation noise V
//   when the camera is struggling, so the Kalman filter relies more on
//   the encoder dead-reckoning and less on the unreliable visual measurement.
//
// PHYSICAL INTUITION:
//   V = σ_pos² × k_V × I₃. Larger V → smaller Kalman gain K → the ZED
//   measurement pulls the state estimate less. When dust, glare, textureless
//   terrain, or fast motion degrade tracking, inflating V prevents a corrupted
//   ZED reading from snapping the rover's estimated position to a wrong place.
//
// INPUTS:
//   zed_confidence ∈ [0, 100]
//     Derived from ZED SDK pose covariance:
//       confidence = 100 × exp(−trace(P_pos) / cov_scale)
//     High = ZED tracking is accurate. Low = ZED is struggling.
//
//   jump_ratio ∈ [0, 5]
//     = |dp_VO| / (|dp_WO| + ε)
//     Ratio of the visual delta to the encoder dead-reckoning delta.
//     Near 1.0 = ZED and encoders agree. >> 1.0 = ZED is drifting.
//     This is computed inside WVO and returned in WVOOutput.jump_ratio.
//
// OUTPUT:
//   k_V ∈ [k_V_min, k_V_max]   (multiplier for observation noise V)
//
// RULE SUMMARY (9 rules — full 3×3 grid):
//   conf=HIGH  + jump=CONSISTENT  → k_V = MIN   (fully trust ZED)
//   conf=HIGH  + jump=DIVERGING   → k_V = MIN×8%  (SDK confident but VO drifting slightly)
//   conf=HIGH  + jump=UNRELIABLE  → k_V = 25%   (SDK confident but jumps too big — suspect)
//   conf=MED   + jump=CONSISTENT  → k_V = 10%   (moderate confidence, clean jump)
//   conf=MED   + jump=DIVERGING   → k_V = 30%   (moderate confidence, VO drifting)
//   conf=MED   + jump=UNRELIABLE  → k_V = 60%   (moderate confidence + bad jump)
//   conf=LOW   + jump=CONSISTENT  → k_V = 25%   (poor SDK confidence alone is enough)
//   conf=LOW   + jump=DIVERGING   → k_V = 70%   (ZED is clearly unreliable)
//   conf=LOW   + jump=UNRELIABLE  → k_V = MAX   (maximum distrust: ignore ZED)
// ═════════════════════════════════════════════════════════════════════════════

struct VOFISParams
{
    // Output range
    double k_V_min = 1.0;     // fully trust ZED
    double k_V_max = 100.0;   // completely distrust ZED

    // ZED confidence membership parameters (universe [0, 100])
    double conf_low_upper   = 40.0;   // confidence fully "Low"  below this
    double conf_med_peak    = 55.0;   // "Medium" confidence peaks here
    double conf_high_lower  = 65.0;   // confidence fully "High" above this

    // jump_ratio membership parameters (universe [0, 5])
    double jump_cons_upper  = 1.5;    // jump is "Consistent" below this
    double jump_div_peak    = 2.5;    // "Diverging" peaks here
    double jump_unrel_lower = 3.0;    // jump is "Unreliable" above this
};

class VOFIS
{
public:
    explicit VOFIS(const VOFISParams& params = VOFISParams{})
        : params_(params)
    {
        buildFIS();
    }

    // Evaluate k_V.
    // zed_confidence ∈ [0, 100]  — clamped internally
    // jump_ratio     ∈ [0, 5]    — clamped internally
    double evaluate(double zed_confidence, double jump_ratio) const
    {
        zed_confidence = std::clamp(zed_confidence, 0.0, 100.0);
        jump_ratio     = std::clamp(jump_ratio,     0.0,   5.0);

        return fis_.evaluate({zed_confidence, jump_ratio}, params_.k_V_max);
    }

    const VOFISParams& getParams() const { return params_; }
    const SugenoFIS&   getFIS()    const { return fis_; }

private:
    void buildFIS()
    {
        const double cl   = params_.conf_low_upper;
        const double cm   = params_.conf_med_peak;
        const double ch   = params_.conf_high_lower;
        const double jc   = params_.jump_cons_upper;
        const double jd   = params_.jump_div_peak;
        const double ju   = params_.jump_unrel_lower;

        const double Kn   = params_.k_V_min;
        const double Kx   = params_.k_V_max;
        const double span = Kx - Kn;

        // ── Input 0: zed_confidence [0, 100] ─────────────────────────────────
        // Index 0 = LOW, 1 = MEDIUM, 2 = HIGH
        fis_.addInput({
            MembershipFunc::makeLeft    (0.0, cl),          // 0: LOW
            MembershipFunc::makeTriangle(0.0, cm, 100.0),   // 1: MEDIUM
            MembershipFunc::makeRight   (ch,  100.0)        // 2: HIGH
        });

        // ── Input 1: jump_ratio [0, 5] ────────────────────────────────────────
        // Index 0 = CONSISTENT, 1 = DIVERGING, 2 = UNRELIABLE
        fis_.addInput({
            MembershipFunc::makeLeft    (0.0, jc),          // 0: CONSISTENT
            MembershipFunc::makeTriangle(1.0, jd, 5.0),     // 1: DIVERGING
            MembershipFunc::makeRight   (ju,  5.0)          // 2: UNRELIABLE
        });

        // ── Rules [conf_idx, jump_idx] ────────────────────────────────────────
        fis_.addRule({{2, 0}, Kn});                 // HIGH conf + CONSISTENT
        fis_.addRule({{2, 1}, Kn + span * 0.08});  // HIGH conf + DIVERGING
        fis_.addRule({{2, 2}, Kn + span * 0.25});  // HIGH conf + UNRELIABLE
        fis_.addRule({{1, 0}, Kn + span * 0.10});  // MED  conf + CONSISTENT
        fis_.addRule({{1, 1}, Kn + span * 0.30});  // MED  conf + DIVERGING
        fis_.addRule({{1, 2}, Kn + span * 0.60});  // MED  conf + UNRELIABLE
        fis_.addRule({{0, 0}, Kn + span * 0.25});  // LOW  conf + CONSISTENT
        fis_.addRule({{0, 1}, Kn + span * 0.70});  // LOW  conf + DIVERGING
        fis_.addRule({{0, 2}, Kx});                 // LOW  conf + UNRELIABLE
    }

    VOFISParams params_;
    SugenoFIS   fis_;
};

// ─────────────────────────────────────────────────────────────────────────────
// Utility: compute slip ratio from wheel speeds
// slip_ratio ∈ [0, 1]
// ─────────────────────────────────────────────────────────────────────────────
inline double computeSlipRatio(double v_L, double v_R,
                                double epsilon = 1e-3)
{
    const double diff = std::abs(v_R - v_L);
    const double sum  = std::abs(v_R) + std::abs(v_L) + epsilon;
    return std::clamp(diff / sum, 0.0, 1.0);
}

} // namespace fuzzy
} // namespace roar

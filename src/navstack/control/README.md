# 🎮 ROAR 2026 — Model Predictive Path Integral (MPPI) Local Controller

[![Package](https://img.shields.io/badge/Package-nav2__mppi__controller-E10600?logo=ros&logoColor=white)](file:///home/draaven/ROAR_2026/src/navstack/control/nav2_mppi_controller)
[![Math](https://img.shields.io/badge/Optimization-SIMD_MPPI-00D9FF)](https://navigation.ros.org/configuration/packages/configuring-mppi-controller.html)

The `nav2_mppi_controller` package provides high-frequency local trajectory optimization for differential-drive navigation over rough Martian terrain. It samples thousands of potential control sequences in parallel, scores them using mathematical cost critics, and computes the optimal control command.

---

## 🧮 MPPI Mathematical Formulation

At each control cycle ($\Delta t = 0.05\,\text{s}$ / $20\,\text{Hz}$), MPPI perturbs a nominal control sequence with Gaussian noise:

$$\mathbf{u}_t^{(k)} = \mathbf{v}_t + \boldsymbol{\epsilon}_t^{(k)}, \quad \boldsymbol{\epsilon}_t^{(k)} \sim \mathcal{N}(\mathbf{0}, \boldsymbol{\Sigma})$$

Each candidate trajectory is projected forward across the prediction horizon ($T = 56$ steps, $2.8\,\text{seconds}$) and evaluated using the total trajectory cost:

$$S\left(\tau^{(k)}\right) = \phi\left(\mathbf{x}_T^{(k)}\right) + \sum_{t=0}^{T-1} \left[ q\left(\mathbf{x}_t^{(k)}\right) + \lambda \mathbf{v}_t^T \boldsymbol{\Sigma}^{-1} \boldsymbol{\epsilon}_t^{(k)} \right]$$

The optimal control update is the cost-weighted expectation:

$$\mathbf{u}_t^* = \mathbf{v}_t + \sum_{k=1}^K w_k \boldsymbol{\epsilon}_t^{(k)}, \quad w_k = \frac{\exp\left(-\frac{1}{\lambda} S(\tau^{(k)})\right)}{\sum_{j=1}^K \exp\left(-\frac{1}{\lambda} S(\tau^{(j)})\right)}$$

---

## ⚖️ Cost Critics Architecture

```mermaid
graph TD
    classDef critic fill:#1A1A24,stroke:#00D9FF,stroke-width:1.5px,color:#FFFFFF;
    classDef opt fill:#241A1A,stroke:#E10600,stroke-width:2px,color:#FFFFFF;

    subgraph CRITICS ["MPPI Cost Critic Functions"]
        C1["ConstraintCritic: Kinematic acceleration & velocity limits"]
        C2["CostCritic: 2.5D elevation roughness & obstacle collision cost"]
        C3["GoalCritic: Distance to terminal goal pose"]
        C4["PathAlignCritic: Lateral distance deviation from global plan"]
        C5["PathFollowCritic: Forward progression along reference path"]
        C6["PathAngleCritic: Orientation alignment with global path tangent"]
        C7["PreferForwardCritic: Penalizes reverse driving on slopes"]
        C8["TwirlingCritic: Penalizes rapid angular oscillations"]
    end

    C1 & C2 & C3 & C4 & C5 & C6 & C7 & C8 --> OPT["Cost-Weighted Softmax Accumulator"]
    OPT --> CMD["Optimal Velocity Command: /cmd_vel"]

    class C1,C2,C3,C4,C5,C6,C7,C8 critic;
    class OPT,CMD opt;
```

---

## ⚡ Bundled SIMD Hardware Acceleration (`local_deps`)

To achieve $20\,\text{Hz}$ real-time control rollouts without dedicated GPU dependencies:
- **`xsimd`**: C++ wrapper for portable SIMD intrinsics (AVX2, FMA, SSE4.2).
- **`xtensor`**: Multi-dimensional tensor expressions evaluating thousands of trajectory candidates concurrently on CPU SIMD vector registers.
- Pre-compiled and bundled inside `src/navstack/local_deps` for zero-install portability.

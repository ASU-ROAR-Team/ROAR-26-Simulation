# ⚖️ ROAR 2026 — Automated ERC Referee Scoring Engine

[![Package](https://img.shields.io/badge/Package-referee__engine-E10600?logo=ros&logoColor=white)](file://${ROAR_WS}/src/navigation/referee_engine)
[![Rules](https://img.shields.io/badge/Rules-ERC_2026_Official-00D9FF)](https://roverchallenge.eu/)

The `referee_engine` package implements an automated, deterministic scoring evaluator adhering strictly to the official **European Rover Challenge Navigation Task Rules** (Traverse Sub-Task: Maximum 300 Points).

---

## 📊 Scoring System Formulation

```mermaid
flowchart TD
    classDef start fill:#1E293B,stroke:#38BDF8,stroke-width:1.5px,color:#F8FAFC;
    classDef check fill:#0F172A,stroke:#818CF8,stroke-width:1.5px,color:#F8FAFC;
    classDef score fill:#1E1B4B,stroke:#10B981,stroke-width:2px,color:#F8FAFC;
    classDef penalty fill:#450A0A,stroke:#EF4444,stroke-width:2px,color:#F8FAFC;

    A["Rover Reaches Waypoint Stop Pose"] --> B{"Positional Error e_pos = ||p_rover - p_wp||"}
    B -- "e_pos <= 0.5m" --> C["Full Points (100% Leg Score)"]
    B -- "0.5m < e_pos <= 2.0m" --> D["Linear Score Decay: 100% -> 20%"]
    B -- "e_pos > 2.0m" --> E["0 Points for Leg"]

    C & D & E --> F{"Time Spent vs Time Budget"}
    F -- "T <= T_max" --> G["No Time Penalty"]
    F -- "T > T_max" --> H["Time Penalty Decay"]
    F -- "T >= 20 min" --> I["Hard Stop (Simulation Timeout)"]

    G & H --> J{"Collision / Boundary Check"}
    J -- "Rock Collision Detected" --> K["Penalty: -50 Points"]
    J -- "Out of Bounds" --> L["Disqualification / Abort"]
    J -- "Clean Traverse" --> M["Leg Score Confirmed"]

    class A start;
    class B,F,J check;
    class C,G,M score;
    class D,E,H,I,K,L penalty;
```

---

## 🧮 Mathematical Scoring Rules

### 1. Positional Error ($e_{\text{pos}}$)
$$\text{Error} = \|\mathbf{p}_{\text{rover}} - \mathbf{p}_{\text{target}}\|_2 = \sqrt{(x_{\text{rover}} - x_{\text{wp}})^2 + (y_{\text{rover}} - y_{\text{wp}})^2}$$

- **Target Radius**: $r_{\text{inner}} = 0.5\,\text{m}$ (Maximum Points).
- **Tolerance Radius**: $r_{\text{outer}} = 2.0\,\text{m}$.
- Points for leg $i$:
$$\text{Score}_i = \begin{cases} 
S_{\text{base}} & \text{if } e_{\text{pos}} \le 0.5 \\
S_{\text{base}} \cdot \left(1.0 - 0.8 \cdot \frac{e_{\text{pos}} - 0.5}{1.5}\right) & \text{if } 0.5 < e_{\text{pos}} \le 2.0 \\
0 & \text{if } e_{\text{pos}} > 2.0
\end{cases}$$

### 2. Time Budget & Speed Caps
- Path length of leg: $L_i$.
- Minimum speed: $v_{\text{min}} = 0.15\,\text{m/s} \implies T_{\text{max}} = \frac{L_i}{v_{\text{min}}}$.
- Maximum speed: $v_{\text{max}} = 0.80\,\text{m/s} \implies T_{\text{min}} = \frac{L_i}{v_{\text{max}}}$.
- Hard Competition Time Cap: **20 minutes (1200 seconds)**.

---

## 📄 Live Outputs & Scorecards

- **Real-Time HTML Scorecard**: `reports/referee/ERC_Report_Live.html`  
  Continuously auto-refreshes during live runs to display leg-by-leg breakdowns, trajectory plots, and current accumulated points.
- **Structured Data Summary**: `reports/referee/run_summary.npy`  
  Stores terminal status (`SUCCESS`, `COLLISION`, `TIMEOUT`), point breakdown, execution times, and trajectory coordinates for batch aggregation.

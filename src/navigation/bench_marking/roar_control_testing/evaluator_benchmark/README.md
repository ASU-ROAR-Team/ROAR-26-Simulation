# Controller Evaluator & Motion Benchmark Suite

A high-performance **Passive Global Controller Evaluator, Motion Smoothness Analyzer & Visualizer** for ROS 2 navigation stacks (e.g., Nav2 MPPI, DWB, TEB, Pure Pursuit).

This suite subscribes dynamically to target planning trajectories, robot odometry feedback, and controller velocity command topics to evaluate motion precision, dynamic jerk limits, signal smoothness, and physical envelope safety without altering robot behavior.

---

## 🚀 Key Features

* **Real-time Path Tracking Metrics**: Evaluates lateral cross-track error, heading angular deviation, and turn overshoot in real time.
* **Jerk & Motion Smoothness Analysis**: Measures linear jerk ($\text{m/s}^3$), steering angular jerk ($\text{deg/s}^2$), and control chatter (total variation of $v$ and $\omega$).
* **Velocity Envelope Safety Monitoring**: Automatically flags linear/angular velocity limit breaches and jerk instability events.
* **Multi-Format Telemetry Exports**: Automatically generates outputs in multiple formats:
  * **`.csv`**: Raw tabular time-series telemetry.
  * **`.npy`**: Binary structured NumPy array preserving datatypes and array shapes.
  * **`.npz`**: High-compression archive of column arrays (over 10x smaller file size).
  * **`.png`**: High-resolution 3-panel performance diagnostic plot.
  * **`.html`**: Self-contained interactive HTML evaluation report.
* **RViz2 & TF Integration**: Automatically spawns RViz2 with preconfigured 3D robot markers, path visualizations, and optional TF broadcasting (`odom -> base_link`).

---

## 📁 Package Architecture

```text
evaluator_benchmark/
├── launch/
│   └── evaluator.launch.py       # Configurable ROS 2 launch file
├── src/
│   ├── integrated_evaluator.py   # Core evaluator ROS 2 node & report generator
│   └── export_npy.py             # CSV-to-NumPy (.npy & .npz) exporter utility
└── README.md                     # Documentation & usage guide
```

---

## 🛠️ Installation & Requirements

### Dependencies
* **ROS 2**: Humble / Iron / Rolling
* **Python**: 3.8+
* **Required Libraries**:
  ```bash
  pip install numpy matplotlib
  ```

---

## ⚙️ Launch Configuration & Parameters

Launch the evaluator node with custom topic names or physical limits:

```bash
ros2 launch evaluator_benchmark evaluator.launch.py \
    path_topic:=/plan \
    odom_topic:=/odom \
    cmd_topic:=/cmd_vel \
    velocity_limit_m_s:=0.7 \
    angular_velocity_limit_rad_s:=1.2 \
    launch_rviz:=true
```

### Configurable Launch Arguments

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `path_topic` | `/plan` | ROS topic for target path/trajectory (`nav_msgs/msg/Path`) |
| `odom_topic` | `/odom` | ROS topic for robot odometry (`nav_msgs/msg/Odometry`) |
| `cmd_topic` | `/cmd_vel` | ROS topic for controller commands (`geometry_msgs/msg/Twist`) |
| `velocity_limit_m_s` | `0.7` | Maximum linear velocity limit threshold ($\text{m/s}$) |
| `angular_velocity_limit_rad_s` | `1.2` | Maximum angular velocity limit threshold ($\text{rad/s}$) |
| `max_allowed_jerk_deg_s2` | `150.0` | Steering jerk warning threshold ($\text{deg/s}^2$) |
| `controller_param_file` | `""` | Optional YAML file (e.g., `nav2_mppi_params.yaml`) to auto-extract limits |
| `csv_output_path` | `/tmp/controller_global_behavior.csv` | Output file path for CSV telemetry |
| `launch_rviz` | `true` | Automatically launch RViz2 visualizer |
| `publish_tf` | `false` | Broadcast `odom -> base_link` TF frame |

---

## 📊 Telemetry Output Formats

### 1. NumPy Format (`.npy` & `.npz`)
* **`.npy` File**: [`/tmp/evaluator_control_data.npy`](file:///tmp/evaluator_control_data.npy) (Structured array)
* **`.npz` Archive**: [`/tmp/evaluator_control_data.npz`](file:///tmp/evaluator_control_data.npz) (Compressed column dictionary)

#### Loading `.npy` Data in Python:
```python
import numpy as np

# Load structured array
data = np.load('/tmp/evaluator_control_data.npy')

# Access fields directly
print(f"Total samples: {len(data)}")
print(f"Max Lateral Error: {data['lat_error_m'].max():.4f} m")
print(f"Max Steering Jerk: {data['steering_jerk_deg_s2'].max():.2f} deg/s²")
```

#### Loading `.npz` Compressed Data in Python:
```python
import numpy as np

# Load compressed dictionary archive
archive = np.load('/tmp/evaluator_control_data.npz')

# Extract individual array channels
times = archive['timestamp_s']
cmd_v = archive['cmd_linear_v']
actual_v = archive['actual_linear_v']
lat_err = archive['lat_error_m']
```

### 2. Standalone Converter Tool
Convert any evaluator CSV file to NumPy `.npy` and `.npz` formats manually using the exporter script:

```bash
python3 src/export_npy.py /tmp/controller_global_behavior.csv
```

### 3. Diagnostic Plots (`.png`)
Generates a 3-panel Matplotlib plot showing:
1. **Linear Velocity Tracking**: Command vs. Actual velocity against envelope limits.
2. **Jerk & Smoothness Dynamics**: Steering jerk ($\text{deg/s}^2$) & linear jerk ($\text{m/s}^3$).
3. **Path Tracking Errors**: Lateral cross-track error ($\text{m}$) & turn overshoot ($\text{m}$).

### 4. Interactive HTML Report (`.html`)
Generates a self-contained HTML report summarizing:
* **Trajectory Accuracy**: Similarity percentage, average/peak lateral error, max turn overshoot.
* **Control Chatter**: RMS steering jerk, peak jerk, total variation velocity chatter.
* **Safety Violations**: Total velocity limit breaches and jerk instability count.

---

## 📝 Data Schema Reference

| Field Name | Type | Description |
| :--- | :--- | :--- |
| `timestamp_s` | `float64` | Time elapsed since evaluation start (seconds) |
| `case_id` | `string` | Active test case identifier |
| `cmd_linear_v` | `float32` | Commanded linear velocity $v_{\text{cmd}}$ ($\text{m/s}$) |
| `actual_linear_v` | `float32` | Actual robot linear velocity $v_{\text{act}}$ ($\text{m/s}$) |
| `cmd_angular_w` | `float32` | Commanded angular velocity $\omega_{\text{cmd}}$ ($\text{rad/s}$) |
| `actual_angular_w` | `float32` | Actual robot angular velocity $\omega_{\text{act}}$ ($\text{rad/s}$) |
| `linear_jerk_m_s3` | `float32` | Rate of change of linear acceleration ($\text{m/s}^3$) |
| `steering_jerk_deg_s2` | `float32` | Rate of change of angular acceleration ($\text{deg/s}^2$) |
| `lat_error_m` | `float32` | Distance from robot position to target trajectory segment ($\text{m}$) |
| `pos_error_m` | `float32` | Direct Euclidean distance to active lookahead path point ($\text{m}$) |
| `heading_error_deg` | `float32` | Angular error between robot heading and path segment tangent ($\text{deg}$) |
| `overshoot_m` | `float32` | Perpendicular drift during turning maneuvers ($\text{m}$) |
| `linear_violation` | `int32` | Flag ($1$ or $0$) indicating linear velocity limit breach |
| `angular_violation` | `int32` | Flag ($1$ or $0$) indicating angular velocity limit breach |
| `jerk_instability` | `int32` | Flag ($1$ or $0$) indicating unstable control signal chatter |
| `similarity_ratio` | `float32` | Cumulative fraction of samples within target error tolerance |

---

## 📜 License
BSD-3-Clause / Apache 2.0

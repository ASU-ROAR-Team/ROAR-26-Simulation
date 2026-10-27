# ROAR SLAM Testing Benchmarking

This package provides a SLAM Evaluator and Benchmarking Node for the ROAR-26 rover simulation. It is designed to evaluate the performance of SLAM algorithms by comparing estimated odometry against ground truth data, calculating trajectory errors, drift rates, and monitoring static drift. It also generates a comprehensive PDF report.

## Installation

Ensure you have the required dependencies installed:
```bash
pip install numpy matplotlib pandas scipy reportlab
```

Build the package using `colcon`:
```bash
cd ~/Navigation_Mission_Workspace
colcon build --packages-select roar_slam_testing
source install/setup.bash
```

## Usage

You can run the SLAM evaluator node using `ros2 run`:

```bash
ros2 run roar_slam_testing slam_evaluator
```

The node will start listening to the specified topics, compute SLAM evaluation metrics (RMSE, mean error, yaw error, etc.), and upon termination (Ctrl+C), it will finalize and generate a PDF report in the specified output directory.

### Parameters

The node accepts the following parameters which can be passed via command line or a launch file:

- `ground_truth_topic`: Topic name for the ground truth pose (`nav_msgs/Odometry`). (default: `/ground_truth/pose`)
- `odom_topic`: Topic name for the estimated odometry messages (`nav_msgs/Odometry`). (default: `/odom`)
- `cmd_vel_topic`: Topic name for the command velocity. (default: `/cmd_vel`)
- `occupancy_topic`: Topic name for the active map occupancy grid. (default: `/active_map/occupancy`)
- `output_dir`: Directory where the final PDF report will be generated. (default: `/tmp/slam_report`)
- `truth_heightmap_path`: Path to the ground truth heightmap for topography validation. (default: `''`)

Ground truth and odom are tracked independently (no `ApproximateTimeSynchronizer`): the node
remembers the latest ground truth pose and scores each incoming odom message against it. If the
report always shows zero error, check that a publisher actually exists on `ground_truth_topic`
with the right type — `ros2 topic info ground_truth_topic` — since a type mismatch connects zero
publishers with no error from ROS 2.

#### Example with custom parameters

```bash
ros2 run roar_slam_testing slam_evaluator --ros-args -p output_dir:=/home/user/slam_reports -p odom_topic:=/slam_odom
```

## Features

- **Trajectory Error Tracking**: Calculates Root Mean Square Error (RMSE) and mean errors along the X, Y axes, and Yaw.
- **Drift Rate Calculation**: Computes the drift rate over time.
- **Static Drift Monitoring**: Monitors drift when the rover is stationary (commanded velocity is zero).
- **Automated Reporting**: Generates a professional PDF report summarizing the SLAM performance using `reportlab`.

# 💻 Terminal Tutorial: Running the Simulation

This is a quick-start, step-by-step tutorial on exactly what to type into your terminals to get the Full Simulation, the Referee Engine, and the Teleop Controller running together.

You will need **3 separate terminal windows** open simultaneously. 

---

### 🖥️ Terminal 1: Launch Gazebo & The Rover
This terminal spins up the 3D physics engine, spawns the Marsyard environment, and injects your rover into the world.

1. Open a new terminal.
2. Navigate to your Dev Environment:
   ```bash
   cd ~/ROAR-26-Simulation/dev_environment
   ```
3. Run the automated testing script. We will launch the `noise` rover configuration in `world1` with `rviz` enabled:
   ```bash
   bash launch_test.sh noise world1 rviz
   ```
4. *Wait until the Gazebo GUI pops up and RViz fully loads the rover model.*

---

### 🖥️ Terminal 2: Build & Launch the Referee Engine
This terminal acts as the "judge." It runs silently in the background, watching your rover's position on the map and calculating your score in real time.

1. Open a second terminal.
2. Navigate to the Navigation workspace and **build the referee engine**:
   ```bash
   cd ~/Navigation_Mission_Workspace
   colcon build --packages-select referee_engine --symlink-install
   source install/setup.bash
   ```
3. Launch the Referee Engine. (Make sure `run_mock:=false` so it listens to your real Gazebo rover, not a fake testing script!)
   ```bash
   ros2 launch referee_engine erc_evaluation.launch.py mission:=mock_mission run_mock:=false
   ```
4. *You should see `[DEBUG EVAL]` logs printing `x: 0.0, y: 0.0` (or whatever your rover's starting coordinates are).*

---

### 🖥️ Terminal 3: Drive the Rover
Because you don't have a fully autonomous Navigation Stack driving the rover yet, we will use this terminal to manually drive the rover using your keyboard!

1. Open a third terminal.
2. Source your Dev Environment:
   ```bash
   source ~/ROAR-26-Simulation/dev_environment/install/setup.bash
   ```
3. Run your custom teleop GUI:
   ```bash
   ros2 run roar_rover_noise teleop.py
   ```
4. A small Tkinter window titled "Rover Teleop" will pop up. **Click on this window with your mouse to focus it.**
5. Use your keyboard's **Arrow Keys** to drive the rover forward towards `x=10m`!

---

### 🚦 Terminal 4 (Optional): The "NEXT" Command
When you drive into a waypoint and come to a complete stop, the Referee Engine in Terminal 2 will lock your score and pause the clock. It simulates a human judge verifying your parking. 

To tell the referee you are ready to drive to the next waypoint, run this command in a 4th terminal:
```bash
ros2 topic pub --once /referee/command std_msgs/String "{data: 'NEXT'}"
```

#!/usr/bin/env python3
import subprocess
import time
import os
import sys

def kill_leftovers():
    cmds = [
        "pkill -f dstar_node",
        "pkill -f dynamic_map_publisher",
        "pkill -f path_simulator",
        "pkill -f static_transform_publisher",
        "pkill -f rviz2",
        "pkill -f test_harness_fallback"
    ]
    for cmd in cmds:
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)

def main():
    print("==================================================")
    print("D* Lite — Near-Goal Fallback Test Runner")
    print("==================================================")
    
    kill_leftovers()
    
    # We send a goal (4.5, 4.5) which becomes dynamically blocked by a box of obstacles.
    # The planner must snap it to a fallback pose within 5.0m.
    cmd = [
        "ros2", "launch", "dstar_navigation", "test_fallback.launch.py",
        "scenario:=lidar_unreachable_goal",
        "start_x:=0.5",
        "start_y:=0.5",
        "goal_x:=4.5",
        "goal_y:=4.5"
    ]
    
    bash_cmd = "source /opt/ros/humble/setup.bash && source install/setup.bash && " + " ".join(cmd)
    
    print("\n---> Launching fallback test...")
    proc = subprocess.Popen(bash_cmd, shell=True, executable='/bin/bash')
    
    timeout = 90.0
    start_time = time.time()
    passed = False
    
    while time.time() - start_time < timeout:
        ret = proc.poll()
        if ret is not None:
            if ret == 0:
                print("Scenario 'lidar_unreachable_goal': PASSED")
                passed = True
            else:
                print(f"Scenario 'lidar_unreachable_goal': FAILED (Exit Code: {ret})")
                passed = False
            break
        time.sleep(0.5)
        
    if proc.poll() is None:
        print(f"Scenario 'lidar_unreachable_goal': TIMEOUT after {timeout}s")
        proc.terminate()
        proc.wait()
        passed = False
        
    kill_leftovers()
    sys.exit(0 if passed else 1)

if __name__ == '__main__':
    os.chdir('/home/amrtamer/nav-stack_2026')
    main()

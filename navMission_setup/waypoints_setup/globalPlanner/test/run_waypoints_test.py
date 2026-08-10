#!/usr/bin/env python3
import subprocess
import time
import os
import sys

def kill_leftovers():
    cmds = [
        "pkill -f dstar_node",
        "pkill -f erc_map_generator",
        "pkill -f path_simulator",
        "pkill -f static_transform_publisher",
        "pkill -f rviz2",
        "pkill -f test_harness_waypoints"
    ]
    for cmd in cmds:
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)

def main():
    print("==================================================")
    print("D* Lite — Multi-Waypoint Navigation Test Runner")
    print("==================================================")
    
    kill_leftovers()
    
    cmd = [
        "ros2", "launch", "dstar_navigation", "test_waypoints.launch.py",
        "start_x:=1.0",
        "start_y:=1.0"
    ]
    
    bash_cmd = "source /opt/ros/humble/setup.bash && source install/setup.bash && " + " ".join(cmd)
    
    print("\n---> Launching multi-waypoint test on complex 400x400 map...")
    proc = subprocess.Popen(bash_cmd, shell=True, executable='/bin/bash')
    
    timeout = 300.0  # 5 minutes because it visits 4 waypoints across 20mx20m map
    start_time = time.time()
    passed = False
    
    while time.time() - start_time < timeout:
        ret = proc.poll()
        if ret is not None:
            if ret == 0:
                print("Scenario 'multi_waypoint': PASSED")
                passed = True
            else:
                print(f"Scenario 'multi_waypoint': FAILED (Exit Code: {ret})")
                passed = False
            break
        time.sleep(0.5)
        
    if proc.poll() is None:
        print(f"Scenario 'multi_waypoint': TIMEOUT after {timeout}s")
        proc.terminate()
        proc.wait()
        passed = False
        
    kill_leftovers()
    sys.exit(0 if passed else 1)

if __name__ == '__main__':
    os.chdir('/home/amrtamer/nav-stack_2026')
    main()

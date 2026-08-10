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
        "pkill -f test_harness_erc"
    ]
    for cmd in cmds:
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)

def main():
    print("==================================================")
    print("D* Lite Navigation — Phase 5 ERC Simulation Runner")
    print("==================================================")
    
    kill_leftovers()
    
    cmd = [
        "ros2", "launch", "dstar_navigation", "test_erc.launch.py",
        "start_x:=1.0",
        "start_y:=1.0",
        "goal_x:=19.0",
        "goal_y:=19.0"
    ]
    
    bash_cmd = "source /opt/ros/humble/setup.bash && source install/setup.bash && " + " ".join(cmd)
    
    print("\n---> Launching full ERC Simulation (400x400 map)...")
    proc = subprocess.Popen(bash_cmd, shell=True, executable='/bin/bash')
    
    # Large scale navigation requires a longer timeout limit
    timeout = 180.0
    start_time = time.time()
    passed = False
    
    while time.time() - start_time < timeout:
        ret = proc.poll()
        if ret is not None:
            if ret == 0:
                print("\nERC Simulation: PASSED")
                passed = True
            else:
                print(f"\nERC Simulation: FAILED (Exit Code: {ret})")
                passed = False
            break
        time.sleep(1.0)
        
    if proc.poll() is None:
        print(f"\nERC Simulation: TIMEOUT after {timeout}s")
        proc.terminate()
        proc.wait()
        passed = False
        
    kill_leftovers()
    sys.exit(0 if passed else 1)

if __name__ == '__main__':
    os.chdir('/home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning')
    main()

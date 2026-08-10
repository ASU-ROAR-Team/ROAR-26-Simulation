#!/usr/bin/env python3
import subprocess
import time
import os
import sys

# Test definitions: (scenario, start_x, start_y, goal_x, goal_y)
TESTS = [
    ('known_maze_blocked', 0.5, 2.0, 4.5, 2.0),
    ('lidar_maze_120', 0.5, 0.5, 4.5, 4.5)
]

def kill_leftovers():
    # Kill any leftover nodes from previous runs to avoid ports/names collision
    cmds = [
        "pkill -f dstar_node",
        "pkill -f dynamic_map_publisher",
        "pkill -f path_simulator",
        "pkill -f static_transform_publisher",
        "pkill -f rviz2",
        "pkill -f test_harness_dynamic"
    ]
    for cmd in cmds:
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)

def main():
    print("==================================================")
    print("D* Lite Navigation — Phase 2 Dynamic Tests Runner")
    print("==================================================")
    
    results = {}
    
    for scenario, sx, sy, gx, gy in TESTS:
        kill_leftovers()
        
        print(f"\n---> Starting Test Scenario: '{scenario}'")
        print(f"     Start: ({sx}, {sy}) -> Goal: ({gx}, {gy})")
        
        cmd = [
            "ros2", "launch", "dstar_navigation", "test_dynamic.launch.py",
            f"scenario:={scenario}",
            f"start_x:={sx}",
            f"start_y:={sy}",
            f"goal_x:={gx}",
            f"goal_y:={gy}"
        ]
        
        # Source environments inside the bash execution
        bash_cmd = f"source /opt/ros/humble/setup.bash && source install/setup.bash && " + " ".join(cmd)
        
        proc = subprocess.Popen(bash_cmd, shell=True, executable='/bin/bash')
        
        # Monitor the process. It will exit when the test_harness_dynamic node terminates.
        timeout = 90.0
        start_time = time.time()
        passed = False
        
        while time.time() - start_time < timeout:
            ret = proc.poll()
            if ret is not None:
                # Process ended
                if ret == 0:
                    print(f"Scenario '{scenario}': PASSED")
                    passed = True
                else:
                    print(f"Scenario '{scenario}': FAILED (Exit Code: {ret})")
                    passed = False
                break
            time.sleep(0.5)
            
        if proc.poll() is None:
            print(f"Scenario '{scenario}': TIMEOUT after {timeout}s")
            proc.terminate()
            proc.wait()
            passed = False
            
        results[scenario] = "PASS" if passed else "FAIL"
        kill_leftovers()
        
    print("\n==================================================")
    print("                  TEST RESULTS                    ")
    print("==================================================")
    all_pass = True
    for scenario, res in results.items():
        print(f"Scenario: {scenario:<18} -> {res}")
        if res == "FAIL":
            all_pass = False
            
    print("==================================================")
    sys.exit(0 if all_pass else 1)

if __name__ == '__main__':
    # Make sure we are in the workspace directory when running
    os.chdir('/home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning')
    main()

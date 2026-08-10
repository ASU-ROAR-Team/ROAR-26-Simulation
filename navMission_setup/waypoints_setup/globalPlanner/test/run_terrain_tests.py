#!/usr/bin/env python3
import subprocess
import time
import os
import sys

# Test definitions: (scenario, start_x, start_y, goal_x, goal_y)
TESTS = [
    ('rough_patch', 0.5, 2.5, 4.5, 2.5),
    ('gradient_slope', 0.5, 2.5, 4.5, 2.5),
    ('forced_rough', 0.5, 2.5, 4.5, 2.5),
    ('perlin_terrain', 0.5, 2.5, 4.5, 2.5)
]

def kill_leftovers():
    cmds = [
        "pkill -f dstar_node",
        "pkill -f terrain_map_generator",
        "pkill -f path_simulator",
        "pkill -f static_transform_publisher",
        "pkill -f rviz2",
        "pkill -f test_harness_terrain"
    ]
    for cmd in cmds:
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)

def main():
    print("==================================================")
    print("D* Lite Navigation — Phase 4 Terrain Tests Runner")
    print("==================================================")
    
    results = {}
    
    for scenario, sx, sy, gx, gy in TESTS:
        kill_leftovers()
        
        print(f"\n---> Starting Test Scenario: '{scenario}'")
        print(f"     Start: ({sx}, {sy}) -> Goal: ({gx}, {gy})")
        
        cmd = [
            "ros2", "launch", "dstar_navigation", "test_terrain.launch.py",
            f"scenario:={scenario}",
            f"start_x:={sx}",
            f"start_y:={sy}",
            f"goal_x:={gx}",
            f"goal_y:={gy}"
        ]
        
        bash_cmd = "source /opt/ros/humble/setup.bash && source install/setup.bash && " + " ".join(cmd)
        
        proc = subprocess.Popen(bash_cmd, shell=True, executable='/bin/bash')
        
        timeout = 90.0
        start_time = time.time()
        passed = False
        
        while time.time() - start_time < timeout:
            ret = proc.poll()
            if ret is not None:
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
        print(f"Scenario: {scenario:<20} -> {res}")
        if res == "FAIL":
            all_pass = False
            
    print("==================================================")
    sys.exit(0 if all_pass else 1)

if __name__ == '__main__':
    os.chdir('/home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning')
    main()

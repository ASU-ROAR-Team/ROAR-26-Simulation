import os
import glob

workspace = '/home/draaven/ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion'
files = glob.glob(f'{workspace}/**/*.sdf', recursive=True) + glob.glob(f'{workspace}/**/*.world', recursive=True)

patch = '    <plugin filename="ignition-gazebo-contact-system" name="ignition::gazebo::systems::Contact"/>\n    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>'
patch2 = '    <plugin filename="ignition-gazebo-contact-system" name="ignition::gazebo::systems::Contact" />\n    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics" />'

for fpath in files:
    if 'install/' in fpath or 'build/' in fpath:
        continue # skip built files
    # Only patch files that have Physics system but no Contact system
    if os.path.exists(fpath):
        with open(fpath, 'r') as f:
            content = f.read()
        if 'ignition-gazebo-contact-system' not in content:
            target = '<plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>'
            target2 = '<plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics" />'
            
            if target in content:
                content = content.replace(target, patch, 1)
                with open(fpath, 'w') as f:
                    f.write(content)
                print(f"Patched {fpath}")
            elif target2 in content:
                content = content.replace(target2, patch2, 1)
                with open(fpath, 'w') as f:
                    f.write(content)
                print(f"Patched {fpath}")

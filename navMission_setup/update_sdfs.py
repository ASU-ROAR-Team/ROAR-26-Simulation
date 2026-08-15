import os
import xml.etree.ElementTree as ET

rocks_dir = os.path.expanduser("~/ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion/navMission_setup/world_setup/rocks_ws")
for i in range(1, 10):
    sdf_file = f"{rocks_dir}/rock_{i}/model.sdf"
    if not os.path.exists(sdf_file):
        continue
    
    with open(sdf_file, 'r') as f:
        content = f.read()
    
    if i in [6, 7, 8, 9]:
        content = content.replace("<mass>5000.0</mass>", "<mass>10.0</mass>")
        content = content.replace("<ixx>1000.0</ixx>", "<ixx>0.1</ixx>")
        content = content.replace("<iyy>1000.0</iyy>", "<iyy>0.1</iyy>")
        content = content.replace("<izz>1000.0</izz>", "<izz>0.1</izz>")
    
    with open(sdf_file, 'w') as f:
        f.write(content)
print("SDFs updated successfully!")

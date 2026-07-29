import numpy as np
import os
import xml.etree.ElementTree as ET

# ==========================================
# 1. LANDMARK-TO-MODEL MAPPING CONFIGURATION
# ==========================================
# Map Landmark index (1 to 15) to its specific model folder name:
# e.g., Landmark 1 (L1) -> model folder name "aruco_1"
# Change these values directly to specify which model folder each landmark uses.
LANDMARK_MODELS = {
    1: "aruco_1",
    2: "aruco_2",
    3: "aruco_3",
    4: "aruco_4",
    5: "aruco_5",
    6: "aruco_6",
    7: "aruco_7",
    8: "aruco_8",
    9: "aruco_9",
    10: "aruco_10",
    11: "aruco_11",
    12: "aruco_12",
    13: "aruco_13",
    14: "aruco_14",
    15: "aruco_15",
}

# ==========================================
# 2. FUSION PIPELINE
# ==========================================
dir_path = "/home/saif/Desktop/ROAR/simulation_ws/src/navMission_setup/world_setup/TempArucoGen"
base_world_path = "/home/saif/Desktop/ROAR/simulation_ws/src/marsyards/worlds/worlds/world_Rotated.world"
npy_path = os.path.join(dir_path, "aruco_data", "aruco_data.npy")
output_world_path = os.path.join(dir_path, "world", "world_Rotated_Aruco.world")

# Load mapped coordinates
coords = np.load(npy_path)

# Parse base world XML
tree = ET.parse(base_world_path)
root = tree.getroot()
world_elem = root.find('world')
if world_elem is None:
    raise ValueError("Invalid world file: missing <world> element")

# Fuse each landmark
for i, coord in enumerate(coords, 1):
    x, y, z = coord
    
    # Landmark IDs are 51 to 65 (for L1 to L15)
    model_id = 50 + i
    model_name = f"ArUco_{model_id}"
    
    model_el = ET.SubElement(world_elem, 'model', attrib={'name': model_name})
    static_el = ET.SubElement(model_el, 'static')
    static_el.text = 'true'
    
    pose_el = ET.SubElement(model_el, 'pose')
    pose_el.text = f"{x:.4f} {y:.4f} {z:.4f} 0.0000 0.0000 0.0000"
    
    include_el = ET.SubElement(model_el, 'include')
    uri_el = ET.SubElement(include_el, 'uri')
    
    # Look up the model name from config mapping (default to aruco_1 if not found)
    assigned_model = LANDMARK_MODELS.get(i, f"aruco_{i}")
    uri_el.text = f"model://{assigned_model}"

# Format and save
ET.indent(tree, space="  ", level=0)
tree.write(output_world_path, encoding='utf-8', xml_declaration=True)
print(f"Successfully fused {len(coords)} markers and saved world to: {output_world_path}")

import numpy as np
import os
import xml.etree.ElementTree as ET

# ==========================================
# 1. LANDMARK-TO-MODEL MAPPING CONFIGURATION
# ==========================================
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
script_dir = os.path.dirname(os.path.abspath(__file__))
dir_path = os.path.dirname(script_dir)
base_world_path = "/home/saif/Desktop/ROAR/simulation_ws/src/marsyards/worlds/worlds/world_Rotated.world"
npy_path = os.path.join(dir_path, "aruco_data", "aruco_data.npy")
output_world_path = os.path.join(dir_path, "world", "world_Rotated_Aruco.world")

# Load mapped coordinates / markers dataset
raw_data = np.load(npy_path, allow_pickle=True)
if hasattr(raw_data, "item"):
    loaded_data = raw_data.item()
else:
    loaded_data = raw_data

if isinstance(loaded_data, dict) and "markers" in loaded_data:
    markers = loaded_data["markers"]
else:
    markers = loaded_data

# Parse base world XML
tree = ET.parse(base_world_path)
root = tree.getroot()
world_elem = root.find('world')
if world_elem is None:
    raise ValueError("Invalid world file: missing <world> element")

# Fuse each landmark
for i, item in enumerate(markers, 1):
    if isinstance(item, dict):
        marker_id = item.get("id", i)
        pos = item.get("position", item)
        x, y, z = float(pos.get("x", 0.0)), float(pos.get("y", 0.0)), float(pos.get("z", 0.0))
        roll = float(item.get("roll", 0.0))
        pitch = float(item.get("pitch", 0.0))
        yaw = float(item.get("yaw", 0.0))
    else:
        marker_id = i
        x, y, z = float(item[0]), float(item[1]), float(item[2])
        roll, pitch, yaw = 0.0, 0.0, 0.0
    
    # Landmark IDs are 51 to 65 (for L1 to L15)
    model_id = 50 + marker_id
    model_name = f"ArUco_{model_id}"
    
    model_el = ET.SubElement(world_elem, 'model', attrib={'name': model_name})
    static_el = ET.SubElement(model_el, 'static')
    static_el.text = 'true'
    
    pose_el = ET.SubElement(model_el, 'pose')
    pose_el.text = f"{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}"
    
    include_el = ET.SubElement(model_el, 'include')
    uri_el = ET.SubElement(include_el, 'uri')
    
    assigned_model = LANDMARK_MODELS.get(marker_id, f"aruco_{marker_id}")
    uri_el.text = f"model://{assigned_model}"

# Format and save
ET.indent(tree, space="  ", level=0)
tree.write(output_world_path, encoding='utf-8', xml_declaration=True)
print(f"Successfully fused {len(markers)} markers and saved world to: {output_world_path}")

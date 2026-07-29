import os
import shutil
import subprocess

# Paths
pdf_path = "/home/saif/Desktop/ROAR/simulation_ws/src/navMission_setup/world_setup/TempArucoGen/Markers/marker_tags.pdf"
marsyard_models_dir = "/home/saif/Desktop/ROAR/simulation_ws/src/marsyards/marsyard/models"
temp_models_dir = "/home/saif/Desktop/ROAR/simulation_ws/src/navMission_setup/world_setup/TempArucoGen/models"

# 1. Render PDF pages to PNG
print("Rendering PDF pages to PNG...")
tmp_prefix = "/tmp/aruco_tag"
# Clean up any existing ones in /tmp
for f in os.listdir("/tmp"):
    if f.startswith("aruco_tag"):
        try:
            os.remove(os.path.join("/tmp", f))
        except Exception:
            pass

subprocess.run([
    "pdftoppm", "-png", "-r", "150", pdf_path, tmp_prefix
], check=True)

# Find generated files
generated_files = sorted([f for f in os.listdir("/tmp") if f.startswith("aruco_tag")])
print(f"Generated {len(generated_files)} PNG files in /tmp")

# 2. Generate models for 1 to 15
for i in range(1, 16):
    model_name = f"aruco_{i}"
    # Page index: page 1 of PDF is '0', page 2 is '1', ..., page 16 is '15'
    # So for marker i, we use page i+1.
    # pdftoppm prints page index with zero-padding (e.g. -01, -02, ..., -10, -11, etc.)
    # Let's dynamically find the correct filename
    page_filename = f"aruco_tag-{i+1:02d}.png"
    # Fallback to single digit if needed (but standard pdftoppm outputs padding based on total pages)
    src_img_path = os.path.join("/tmp", page_filename)
    if not os.path.exists(src_img_path):
        page_filename = f"aruco_tag-{i+1}.png"
        src_img_path = os.path.join("/tmp", page_filename)
    
    if not os.path.exists(src_img_path):
        print(f"Warning: Expected image {src_img_path} not found!")
        continue
        
    # We will write to both marsyard models and temp models directories
    for base_dir in [marsyard_models_dir, temp_models_dir]:
        model_dir = os.path.join(base_dir, model_name)
        textures_dir = os.path.join(model_dir, "materials", "textures")
        os.makedirs(textures_dir, exist_ok=True)
        
        # Copy image
        shutil.copy(src_img_path, os.path.join(textures_dir, f"marker_{i}.png"))
        
        # Write model.config
        config_content = f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Antigravity</name>
    <email>antigravity@google.com</email>
  </author>
  <description>ArUco Marker {i} from ERC 2025 Handbook</description>
</model>
"""
        with open(os.path.join(model_dir, "model.config"), "w") as f:
            f.write(config_content)
            
        # Write model.sdf
        sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <!-- Concrete Base -->
      <visual name="base_visual">
        <pose>0 0 0.025 0 0 0</pose>
        <geometry>
          <box>
            <size>0.35 0.35 0.05</size>
          </box>
        </geometry>
        <material>
          <ambient>0.7 0.7 0.7 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
        </material>
      </visual>
      <collision name="base_collision">
        <pose>0 0 0.025 0 0 0</pose>
        <geometry>
          <box>
            <size>0.35 0.35 0.05</size>
          </box>
        </geometry>
      </collision>

      <!-- Wooden Pole -->
      <visual name="pole_visual">
        <pose>0 0 0.26 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.015</radius>
            <length>0.42</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.6 0.4 0.2 1</ambient>
          <diffuse>0.6 0.4 0.2 1</diffuse>
        </material>
      </visual>
      <collision name="pole_collision">
        <pose>0 0 0.26 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.015</radius>
            <length>0.42</length>
          </cylinder>
        </geometry>
      </collision>

      <!-- Head Box with ArUco Texture -->
      <visual name="head_visual">
        <pose>0 0 0.575 0 0 0</pose>
        <geometry>
          <box>
            <size>0.21 0.21 0.21</size>
          </box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <pbr>
            <metal>
              <albedo_map>model://{model_name}/materials/textures/marker_{i}.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
      <collision name="head_collision">
        <pose>0 0 0.575 0 0 0</pose>
        <geometry>
          <box>
            <size>0.21 0.21 0.21</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""
        with open(os.path.join(model_dir, "model.sdf"), "w") as f:
            f.write(sdf_content)

print("ArUco models created successfully!")

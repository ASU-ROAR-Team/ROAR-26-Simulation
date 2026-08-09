import os
import glob

workspace = '/home/draaven/ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion'
files = glob.glob(f'{workspace}/**/rover_gazebo.xacro', recursive=True) + glob.glob(f'{workspace}/**/rover_gazebo_clean.xacro', recursive=True)

patch = """    </gazebo>

    <xacro:contact_sensor_gazebo name="base_link" />
    <xacro:contact_sensor_gazebo name="wheel_rhs_front" />
    <xacro:contact_sensor_gazebo name="wheel_rhs_mid" />
    <xacro:contact_sensor_gazebo name="wheel_rhs_rear" />
    <xacro:contact_sensor_gazebo name="wheel_lhs_front" />
    <xacro:contact_sensor_gazebo name="wheel_lhs_mid" />
    <xacro:contact_sensor_gazebo name="wheel_lhs_rear" />
  </xacro:macro>

  <xacro:macro name="contact_sensor_gazebo" params="name">
    <gazebo reference="${name}">
      <sensor name="${name}_contact" type="contact">
        <always_on>true</always_on>
        <update_rate>50</update_rate>
        <contact>
          <collision>${name}_collision</collision>
        </contact>
      </sensor>
    </gazebo>
  </xacro:macro>"""

for fpath in files:
    if 'install/' in fpath or 'build/' in fpath:
        continue # skip built files
    if os.path.exists(fpath):
        with open(fpath, 'r') as f:
            content = f.read()
        if 'contact_sensor_gazebo' not in content:
            target = "    </gazebo>\n  </xacro:macro>"
            if target in content:
                content = content.replace(target, patch, 1)
                with open(fpath, 'w') as f:
                    f.write(content)
                print(f"Patched {fpath}")
            else:
                target2 = "    </gazebo>\n\n  </xacro:macro>"
                if target2 in content:
                    content = content.replace(target2, patch, 1)
                    with open(fpath, 'w') as f:
                        f.write(content)
                    print(f"Patched {fpath}")
                else:
                    print(f"Target not found in {fpath}")
        else:
            print(f"Already patched {fpath}")

# ArucoDetection.msg
# A single ArUco marker detection from the perception team.
#
# Coordinate frame conventions:
#   - pose_in_camera is the marker's pose expressed in the ZED2i camera frame.
#   - Standard ROS optical frame convention: x=right, y=down, z=forward
#     (the localization node converts this to the rover body frame internally).
#   - The position component is what the IESKF actually uses; the orientation
#     component is informational (we don't observe orientation since BNO055
#     owns it).
#
# distance: euclidean distance from camera to marker [m].
#   Useful for:
#     - filtering out far-away detections (low accuracy)
#     - scaling the observation noise (closer = more accurate)

std_msgs/Header header           # frame_id should be the ZED optical frame

int32 id                          # ArUco marker ID (e.g., 51-64 per ERC rules)

geometry_msgs/Pose pose_in_camera # marker pose in the camera optical frame

float32 distance                  # distance from camera to marker [m]

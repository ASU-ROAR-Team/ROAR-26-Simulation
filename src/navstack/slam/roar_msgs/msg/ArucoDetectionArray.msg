# ArucoDetectionArray.msg
# Array of ArUco detections from a single camera frame.
# The perception team publishes one of these per frame, with zero or more
# ArucoDetection entries inside.

std_msgs/Header header               # frame timestamp + frame_id

roar_msgs/ArucoDetection[] detections

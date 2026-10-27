std_msgs/Header header
geometry_msgs/Point position       # center of bounding box (x, y, z) in source frame
geometry_msgs/Vector3 dimensions   # width (x), length (y), height (z) of bbox in meters
float32 confidence                 # detection confidence [0.0, 1.0]
uint32 class_id                    # obstacle class identifier

import math


def ref_to_gazebo_position(x, y, z):
    return x, -y, z


def gazebo_to_ref_position(x, y, z):
    return x, -y, z


def ref_to_gazebo_yaw(yaw):
    return -yaw


def gazebo_to_ref_yaw(yaw):
    return -yaw


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

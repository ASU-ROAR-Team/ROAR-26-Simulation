import sys
import types


if 'rclpy' not in sys.modules:
    rclpy_stub = types.ModuleType('rclpy')
    rclpy_stub.init = lambda *args, **kwargs: None
    rclpy_stub.shutdown = lambda *args, **kwargs: None
    rclpy_stub.ok = lambda: True
    sys.modules['rclpy'] = rclpy_stub

    node_stub = types.ModuleType('rclpy.node')
    class DummyNode:  # pragma: no cover - simple stub for import-time compatibility
        pass
    node_stub.Node = DummyNode
    sys.modules['rclpy.node'] = node_stub

    parameter_stub = types.ModuleType('rclpy.parameter')
    class DummyParameter:  # pragma: no cover - simple stub for import-time compatibility
        pass
    parameter_stub.Parameter = DummyParameter
    sys.modules['rclpy.parameter'] = parameter_stub

    msg_stub = types.ModuleType('rcl_interfaces.msg')
    class DummySetParametersResult:  # pragma: no cover - simple stub for import-time compatibility
        def __init__(self, successful: bool = True):
            self.successful = successful
    msg_stub.SetParametersResult = DummySetParametersResult
    sys.modules['rcl_interfaces.msg'] = msg_stub

    tf2_stub = types.ModuleType('tf2_ros')
    class DummyTransformBroadcaster:  # pragma: no cover - simple stub for import-time compatibility
        def __init__(self, *args, **kwargs):
            pass
    tf2_stub.TransformBroadcaster = DummyTransformBroadcaster
    sys.modules['tf2_ros'] = tf2_stub

    geometry_msgs_stub = types.ModuleType('geometry_msgs.msg')
    geometry_msgs_stub.PoseStamped = type('PoseStamped', (), {})
    geometry_msgs_stub.Quaternion = type('Quaternion', (), {})
    geometry_msgs_stub.TransformStamped = type('TransformStamped', (), {})
    geometry_msgs_stub.Twist = type('Twist', (), {})
    sys.modules['geometry_msgs.msg'] = geometry_msgs_stub

    nav_msgs_stub = types.ModuleType('nav_msgs.msg')
    nav_msgs_stub.Odometry = type('Odometry', (), {})
    nav_msgs_stub.Path = type('Path', (), {})
    sys.modules['nav_msgs.msg'] = nav_msgs_stub

    std_msgs_stub = types.ModuleType('std_msgs.msg')
    std_msgs_stub.String = type('String', (), {})
    sys.modules['std_msgs.msg'] = std_msgs_stub

from adaptive_pure_pursuit.test_orchestrator import _resolve_case_deadline


def test_case_deadline_uses_the_longer_of_the_two_limits() -> None:
    assert _resolve_case_deadline(30.0, 40.0) == 40.0
    assert _resolve_case_deadline(60.0, 40.0) == 60.0

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class CameraDetectorNode(Node):
    def __init__(self):
        super().__init__('camera_detector_node')

        # Parameters
        self.declare_parameter('model_version', 'yolov8n-seg')  # yolov8n-seg for masks
        self.declare_parameter('conf_threshold', 0.25)

        model_ver = self.get_parameter('model_version').value
        self.conf_threshold = self.get_parameter('conf_threshold').value

        # Handle absolute paths or standard model names
        if model_ver.endswith('.pt'):
            model_path = model_ver
        else:
            model_path = f"{model_ver}.pt"
            
        self.model = YOLO(model_path)
        
        # Override all class names to 'rock' as requested
        custom_names = {k: 'rock' for k in self.model.names.keys()}
        self.model.model.names = custom_names
        self.class_names = custom_names

        self.bridge = CvBridge()

        # Subscription
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.listener_callback,
            10
        )

        # Publishers
        self.detection_pub = self.create_publisher(Detection2DArray, '/detection/camera_2d', 10)
        self.mask_pub      = self.create_publisher(Image, '/detection/camera_mask', 10)
        self.annotated_pub = self.create_publisher(Image, '/detection/camera_annotated', 10)

        self.get_logger().info(f'Camera Detector Node started with model: {model_path}')

    def listener_callback(self, msg):
        self.conf_threshold = self.get_parameter('conf_threshold').value

        # Convert ROS Image to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        # Run YOLO
        results = self.model.predict(cv_image, conf=self.conf_threshold, verbose=False)

        detections = Detection2DArray()
        detections.header = msg.header
        
        # Create a blank mask for this frame (1 channel, 8-bit)
        H, W = cv_image.shape[:2]
        import numpy as np
        mask_image = np.zeros((H, W), dtype=np.uint8)

        if len(results) > 0 and results[0].boxes is not None:
            result = results[0]
            
            # If model supports segmentation masks
            if hasattr(result, 'masks') and result.masks is not None:
                for i, (box, segment) in enumerate(zip(result.boxes, result.masks.xy)):
                    xywh = box.xywh[0].cpu().numpy()
                    cls_id = int(box.cls[0].cpu().item())
                    conf = float(box.conf[0].cpu().item())

                    detection = Detection2D()
                    detection.header = msg.header
                    detection.id = str(i + 1)  # Links to pixel value (1, 2, 3...)

                    detection.bbox.center.position.x = float(xywh[0])
                    detection.bbox.center.position.y = float(xywh[1])
                    detection.bbox.size_x = float(xywh[2])
                    detection.bbox.size_y = float(xywh[3])

                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = self.class_names[cls_id]
                    hypothesis.hypothesis.score = conf

                    detection.results.append(hypothesis)
                    detections.detections.append(detection)

                    # Draw precise outline into the mask
                    pts = np.array(segment, dtype=np.int32)
                    cv2.fillPoly(mask_image, [pts], i + 1)
            else:
                # Fallback if a standard bounding-box model is used
                for i, box in enumerate(result.boxes):
                    xywh = box.xywh[0].cpu().numpy()
                    cls_id = int(box.cls[0].cpu().item())
                    conf = float(box.conf[0].cpu().item())

                    detection = Detection2D()
                    detection.header = msg.header
                    detection.id = str(i + 1)  # Links to pixel value

                    detection.bbox.center.position.x = float(xywh[0])
                    detection.bbox.center.position.y = float(xywh[1])
                    detection.bbox.size_x = float(xywh[2])
                    detection.bbox.size_y = float(xywh[3])

                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = self.class_names[cls_id]
                    hypothesis.hypothesis.score = conf

                    detection.results.append(hypothesis)
                    detections.detections.append(detection)

                    # Draw bounding box into mask
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    cv2.rectangle(mask_image, (int(x1), int(y1)), (int(x2), int(y2)), i + 1, -1)

        self.detection_pub.publish(detections)
        
        # Publish mask image
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(mask_image, encoding='mono8')
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish mask image: {e}')

        # Annotated image publishing
        try:
            annotated_frame = results[0].plot()
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish annotated image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
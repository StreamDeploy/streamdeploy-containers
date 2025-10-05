#!/usr/bin/env bash
set -euo pipefail

# Default environment variables
: "${ROS_DOMAIN_ID:=0}"
: "${RMW_IMPLEMENTATION:=rmw_cyclonedx_cpp}"
: "${MODEL_PATH:=/opt/openmind/models}"
: "${DETECTION_MODEL:=yolov8n.pt}"
: "${INPUT_TOPIC:=/camera/image_raw}"
: "${OUTPUT_TOPIC:=/perception/detections}"
: "${CONFIDENCE_THRESHOLD:=0.5}"

# Create necessary directories
mkdir -p "${MODEL_PATH}" /opt/openmind/config

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

echo "Starting OpenMind Perception with:"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "  RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
echo "  MODEL_PATH: ${MODEL_PATH}"
echo "  DETECTION_MODEL: ${DETECTION_MODEL}"
echo "  INPUT_TOPIC: ${INPUT_TOPIC}"
echo "  OUTPUT_TOPIC: ${OUTPUT_TOPIC}"
echo "  CONFIDENCE_THRESHOLD: ${CONFIDENCE_THRESHOLD}"

# Download default model if not present
MODEL_FILE="${MODEL_PATH}/${DETECTION_MODEL}"
if [[ ! -f "${MODEL_FILE}" ]]; then
    echo "Downloading default model: ${DETECTION_MODEL}"
    python3 -c "
import torch
from ultralytics import YOLO
model = YOLO('${DETECTION_MODEL}')
model.export(format='pt')
"
    # Move the model to the correct location if needed
    find . -name "${DETECTION_MODEL}" -exec mv {} "${MODEL_FILE}" \; 2>/dev/null || true
fi

# Generate perception node Python script
PERCEPTION_NODE="/opt/openmind/perception_node.py"
cat > "${PERCEPTION_NODE}" <<'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ultralytics import YOLO

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('openmind_perception')
        
        # Parameters
        self.model_path = os.getenv('MODEL_PATH', '/opt/openmind/models')
        self.model_name = os.getenv('DETECTION_MODEL', 'yolov8n.pt')
        self.input_topic = os.getenv('INPUT_TOPIC', '/camera/image_raw')
        self.output_topic = os.getenv('OUTPUT_TOPIC', '/perception/detections')
        self.confidence_threshold = float(os.getenv('CONFIDENCE_THRESHOLD', '0.5'))
        
        # Initialize model
        model_file = os.path.join(self.model_path, self.model_name)
        self.get_logger().info(f'Loading model: {model_file}')
        self.model = YOLO(model_file)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Publishers and subscribers
        self.image_sub = self.create_subscription(
            Image, self.input_topic, self.image_callback, 10)
        self.detection_pub = self.create_publisher(
            Detection2DArray, self.output_topic, 10)
        
        self.get_logger().info('OpenMind Perception Node initialized')
    
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Run inference
            results = self.model(cv_image, conf=self.confidence_threshold)
            
            # Convert to ROS detections
            detection_array = Detection2DArray()
            detection_array.header = msg.header
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        detection = Detection2D()
                        
                        # Bounding box
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        detection.bbox.center.position.x = float((x1 + x2) / 2)
                        detection.bbox.center.position.y = float((y1 + y2) / 2)
                        detection.bbox.size_x = float(x2 - x1)
                        detection.bbox.size_y = float(y2 - y1)
                        
                        # Confidence and class
                        confidence = float(box.conf[0])
                        class_id = int(box.cls[0])
                        class_name = self.model.names[class_id]
                        
                        # Add to detection array
                        detection_array.detections.append(detection)
            
            # Publish detections
            self.detection_pub.publish(detection_array)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

chmod +x "${PERCEPTION_NODE}"

# Set environment variables
export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION
export MODEL_PATH
export DETECTION_MODEL
export INPUT_TOPIC
export OUTPUT_TOPIC
export CONFIDENCE_THRESHOLD

# Launch the perception node
echo "Launching OpenMind Perception Node..."
exec python3 "${PERCEPTION_NODE}"

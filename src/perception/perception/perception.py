import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
from rcl_interfaces.msg import SetParametersResult

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        #Declare parameters
        self.declare_parameter('model_filename', 'bestmodel.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('bark_distance_threshold', 3.0)
        self.declare_parameter('target_classes', ["apple", "banana", "pizza"])
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('model_dir', '') 

        #Read parameters
        filename = self.get_parameter('model_filename').get_parameter_value().string_value
        model_dir = self.get_parameter('model_dir').get_parameter_value().string_value
        self.conf_thres = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.bark_dist_thres = self.get_parameter('bark_distance_threshold').get_parameter_value().double_value
        self.edible_items = self.get_parameter('target_classes').get_parameter_value().string_array_value
        self.debug = self.get_parameter('debug_mode').get_parameter_value().bool_value
        self.add_on_set_parameters_callback(self.parameter_callback)

        # --- 2. Load YOLO Model ---
        self.model = None
        model_path = os.path.join(model_dir, filename)
        self.get_logger().info(f"Loading image detection model from: {model_path}")
        try:
            self.model = YOLO(model_path)
            self.get_logger().info("Model loaded successfully!")
        except Exception as e:
            self.get_logger().error(f"FATAL: Could not load model. Error: {e}")

        # --- 3. Subscribers ---
        self.bridge = CvBridge()
        #Subscribe to RGB image from camera
        self.create_subscription(Image, '/camera_top/image', self.rgb_callback, 10)
        #Subscribe to Depth image from camera
        self.create_subscription(Image, '/camera_top/depth', self.depth_callback, 10)

        # --- 4. Publishers ---
        #Publish bounded boxes image, labels of detected images, distance to detected objects and speech option
        self.pub_detection_img = self.create_publisher(Image, '/camera/detections/image', 10)
        self.pub_labels = self.create_publisher(String, '/detections/labels', 10)
        self.pub_distance = self.create_publisher(Float32, '/detections/distance', 10)
        self.pub_speech = self.create_publisher(String, '/robot_dog/speech', 10)

        # Variables to store latest data
        self.latest_depth_image = None

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'target_classes':
                if param.type_ == param.Type.STRING_ARRAY:
                    self.edible_items = param.value
                    self.get_logger().info(f"Now looking for: {self.edible_items}")
                    return SetParametersResult(successful=True)
        return SetParametersResult(successful=True)

    def depth_callback(self, msg):
        try:
            #Convert ROS Image to OpenCV (32FC1)
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth error: {e}")

    def rgb_callback(self, msg):
        if self.latest_depth_image is None or self.model is None:
            return 
        try:
            #Convert RGB to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, _ = cv_image.shape

            #Run Inference
            results = self.model(cv_image, verbose=False, conf=self.conf_thres)

            labels_found = []
            bark_command = "..."
            closest_dist = -1.0
            
            #Process Detections
            for result in results:
                for box in result.boxes:
                    #1.Get Coordinates
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls_id = int(box.cls[0])
                    label = self.model.names[cls_id]
                    labels_found.append(label)

                    #2.Calculate Center
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)

                    #3.Get Distance from Depth Image
                    cx = min(max(0, center_x), width - 1)
                    cy = min(max(0, center_y), height - 1)
                    
                    dist = float(self.latest_depth_image[cy, cx])
                    
                    #Handle invalid depth (Inf/NaN)
                    if np.isinf(dist) or np.isnan(dist):
                        dist_str = "unk"
                    else:
                        dist_str = f"{dist:.2f}m"
                        closest_dist = dist #for depth publishing

                    #4.Draw bounding box (label+dist optional) 
                    color = (0, 0, 255) #Red box
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)
                    #cv2.putText(cv_image, f"{label} {dist_str}", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                    # 5. Check Bark Logic
                    #Rule 1: Edible?
                    if label in self.edible_items:
                        #Rule 2: Distance < 3.0m?
                        if not np.isinf(dist) and dist < self.bark_dist_thres:
                            #Rule 3: Centered in middle 3/5th?
                            left_limit = width * (1/5)
                            right_limit = width * (4/5)
                            
                            if left_limit < center_x < right_limit:
                                bark_command = "bark"

            #Publish Results
            #Image with boxes
            self.pub_detection_img.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            #Labels
            self.pub_labels.publish(String(data=str(labels_found)))
            #Distances
            self.pub_distance.publish(Float32(data=closest_dist))
            #Speech command
            self.pub_speech.publish(String(data=bark_command))

        except Exception as e:
            self.get_logger().error(f"Inference Error: {e}")

def main(args=None):
    """
    Main function to start the perception node.
    
    Creates the node and starts the ROS 2 spin loop.
    
    Args:
        args: Command line arguments (optional)
    """
    # Initialize ROS 2
    rclpy.init(args=args)
    
    # Create the node
    node = PerceptionNode()
    
    try:
        # Spin the node
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()
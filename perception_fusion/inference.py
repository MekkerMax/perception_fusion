import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo # NEW: Import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import message_filters
import math

class YoloInferenceNode(Node):
    def __init__(self):
        super().__init__('yolo_inference_node')
        self.model = YOLO('yolov8n.pt') 
        self.bridge = CvBridge()
        
        rgb_sub = message_filters.Subscriber(self, Image, '/zed/front/rgb/image_rect_color')
        depth_sub = message_filters.Subscriber(self, Image, '/zed/front/depth/depth_registered')
        info_sub = message_filters.Subscriber(self, CameraInfo, '/zed/front/rgb/camera_info') 
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)
        
        self.marker_pub = self.create_publisher(MarkerArray, '/yolo/detected_objects_3d', 10)


    def sync_callback(self, rgb_msg, depth_msg, info_msg): 
        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1') 
            
            fx = info_msg.k[0]
            cx = info_msg.k[2]
            fy = info_msg.k[4]
            cy = info_msg.k[5]
            
            results = self.model(cv_rgb, verbose=False)
            annotated_frame = results[0].plot()
            marker_array = MarkerArray()
            
            delete_marker = Marker()
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)
            
            for i, box in enumerate(results[0].boxes):
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                
                distance_m = float(cv_depth[center_y, center_x])
                
                if not math.isnan(distance_m) and distance_m > 0:
                    cv2.circle(annotated_frame, (center_x, center_y), 5, (0, 255, 0), -1)
                    cv2.putText(annotated_frame, f"{distance_m:.2f} m", (x1, y2 + 25), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                    pos_x = (center_x - cx) * distance_m / fx
                    pos_y = (center_y - cy) * distance_m / fy
                    pos_z = distance_m
                    
                    marker = Marker()
                    marker.header.frame_id = 'ZED_X_FRONT_optical' 
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = 'yolo_objects'
                    marker.id = i
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose.position.x = pos_x
                    marker.pose.position.y = pos_y
                    marker.pose.position.z = pos_z
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = 0.6
                    marker.scale.y = 1.7
                    marker.scale.z = 0.6
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 0.6 
                    marker_array.markers.append(marker)
            
            self.marker_pub.publish(marker_array)
            cv2.imshow("3D ", annotated_frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
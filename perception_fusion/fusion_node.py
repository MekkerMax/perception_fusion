import rclpy
import os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped 
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import message_filters
import math
import tf2_ros
import tf2_geometry_msgs 

class ObjectTracker:
    def __init__(self, max_distance=1.5, smoothing_alpha=0.2):
        self.tracks = {} 
        self.next_id = 0
        self.max_distance = max_distance 
        self.alpha = smoothing_alpha     

    def update(self, new_detections):
        updated_tracks = {}
        used_detections = set()

        # 1. Match existing tracks to new detections
        for track_id, track_data in self.tracks.items():
            best_dist = self.max_distance
            best_idx = -1
            current_pos = track_data['pos']
            
            for i, det in enumerate(new_detections):
                if i in used_detections:
                    continue
                
                dist = math.sqrt(sum((c - d) ** 2 for c, d in zip(current_pos, det)))
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i
            
            if best_idx != -1:
                new_pos = new_detections[best_idx]
                # Low Pass Filter (Smoothing)
                smoothed_x = current_pos[0] * (1 - self.alpha) + new_pos[0] * self.alpha
                smoothed_y = current_pos[1] * (1 - self.alpha) + new_pos[1] * self.alpha
                smoothed_z = current_pos[2] * (1 - self.alpha) + new_pos[2] * self.alpha
                
                updated_tracks[track_id] = {'pos': [smoothed_x, smoothed_y, smoothed_z], 'missed_frames': 0}
                used_detections.add(best_idx)
            else:
                # Ghost persistence (keep object alive for 5 frames if missed)
                track_data['missed_frames'] += 1
                if track_data['missed_frames'] < 5: 
                    updated_tracks[track_id] = track_data

        # 2. Create new tracks
        for i, det in enumerate(new_detections):
            if i not in used_detections:
                updated_tracks[self.next_id] = {'pos': det, 'missed_frames': 0}
                self.next_id += 1

        self.tracks = updated_tracks
        return {tid: t['pos'] for tid, t in self.tracks.items()}

class FusionNode(Node):
    def __init__(self):
        super().__init__('camera_fusion_node')
        super().__init__('camera_fusion_node')
    
        pkg_share = get_package_share_directory('perception_fusion')
        model_path = os.path.join(pkg_share, 'config', 'yolov8s.engine')
        

        if not os.path.exists(model_path):
            self.get_logger().error(f"Model not found at: {model_path}")
        else:
            self.get_logger().info(f"Loading TensorRT Engine: {model_path}")
            self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.tracker = ObjectTracker()
        self.current_detections = [] 
        

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


        self.marker_pub = self.create_publisher(MarkerArray, '/fusion/detected_objects_3d', 10)
        

        self.create_timer(0.033, self.publish_fused_markers)
        
        self.setup_camera('front', '/zed/front')
        self.setup_camera('left', '/zed/left')
        self.setup_camera('right', '/zed/right')
        self.setup_camera('back', '/zed/back') 
        
        self.get_logger().info("Fusion Engine Started. Waiting for data...")

    def setup_camera(self, cam_name, base_topic):
        rgb_sub = message_filters.Subscriber(self, Image, f'{base_topic}/rgb/image_rect_color')
        depth_sub = message_filters.Subscriber(self, Image, f'{base_topic}/depth/depth_registered')
        info_sub = message_filters.Subscriber(self, CameraInfo, f'{base_topic}/rgb/camera_info')
        
        ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 10, 0.1)
        ts.registerCallback(lambda r, d, i: self.process_camera(r, d, i, cam_name))

    def process_camera(self, rgb_msg, depth_msg, info_msg, cam_name):
        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            
            fx, cx = info_msg.k[0], info_msg.k[2]
            fy, cy = info_msg.k[4], info_msg.k[5]

            results = self.model(cv_rgb, verbose=False)
            
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_x, center_y = int((x1+x2)/2), int((y1+y2)/2)
                distance_m = float(cv_depth[center_y, center_x])

                if math.isnan(distance_m) or distance_m <= 0:
                    continue


                pos_x = (center_x - cx) * distance_m / fx
                pos_y = (center_y - cy) * distance_m / fy
                pos_z = distance_m
                
                point_optical = PointStamped()
                point_optical.header.frame_id = f"ZED_X_{cam_name.upper()}_optical"

                point_optical.header.stamp = rclpy.time.Time().to_msg()
                point_optical.point.x = pos_x
                point_optical.point.y = pos_y
                point_optical.point.z = pos_z
                
                try:
                    target_frame = "dreverZed"
                    
                    transform = self.tf_buffer.lookup_transform(
                        target_frame, 
                        point_optical.header.frame_id, 
                        rclpy.time.Time(), 
                        rclpy.duration.Duration(seconds=0.1) 
                    )
                    

                    point_global = tf2_geometry_msgs.do_transform_point(point_optical, transform)
                    

                    self.current_detections.append([point_global.point.x, point_global.point.y, point_global.point.z])

                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                    pass 
                    
        except Exception as e:
            self.get_logger().error(f"Error in {cam_name}: {e}")

    def publish_fused_markers(self):
        """Runs at 30Hz: Smooths detections and publishes ONE clean list."""
        if not self.current_detections:
            return

        tracked_objects = self.tracker.update(self.current_detections)
        self.current_detections = [] 
        

        markers = MarkerArray()
        

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        for track_id, pos in tracked_objects.items():
            marker = Marker()
            marker.header.frame_id = "dreverZed"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "fused_tracking"
            marker.id = track_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2]
            
            marker.scale.x = 0.6; marker.scale.y = 0.6; marker.scale.z = 1.7
            marker.color.a = 1.0; marker.color.r = 0.0; marker.color.g = 1.0 # Green
            
            # Text Label for ID
            text = Marker()
            text.header = marker.header
            text.ns = "ids"
            text.id = track_id + 1000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pos[0]
            text.pose.position.y = pos[1]
            text.pose.position.z = pos[2] + 1.0
            text.scale.z = 0.3
            text.color.a = 1.0; text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0
            text.text = f"ID: {track_id}"
            
            markers.markers.append(marker)
            markers.markers.append(text)
            
        self.marker_pub.publish(markers)

def main():
    rclpy.init()
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
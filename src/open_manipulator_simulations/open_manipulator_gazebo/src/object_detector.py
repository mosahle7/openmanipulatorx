#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector')
        
        # Initialize YOLO with minimum settings
        self.model = YOLO('yolov8n.pt')  # Nano model for speed
        self.bridge = CvBridge()
        
        # ROS subscriber
        self.image_sub = rospy.Subscriber('/d435/color/image_raw', Image, self.image_callback)
        self.grip_pub = rospy.Publisher('/gripping_points', PointStamped, queue_size=10)
        
        # Visualization window
        cv2.namedWindow("Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Detection", 800, 600)

    def image_callback(self, msg):
        try:
            # Convert ROS image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Run detection on every frame
            results = self.model(cv_image, conf=0.5, verbose=False)  # Disable logging
            
            # Process all detected objects
            for result in results:
                for box in result.boxes:
                    # Get bounding box coordinates
                    x_min, y_min, x_max, y_max = map(int, box.xyxy[0])
                    
                    # Draw bounding box (green)
                    cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                    
                    # Calculate and draw center point (red)
                    center_x = (x_min + x_max) // 2
                    center_y = (y_min + y_max) // 2
                    cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)
                    
                    # Publish gripping point
                    grip_point = PointStamped()
                    grip_point.header = msg.header
                    grip_point.point.x = center_x
                    grip_point.point.y = center_y
                    grip_point.point.z = 0
                    self.grip_pub.publish(grip_point)
            
            # Display the frame
            cv2.imshow("Detection", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr(f"Processing error: {str(e)}")

if __name__ == '__main__':
    try:
        detector = ObjectDetector()
        rospy.loginfo("Detection system ready - Showing live feed with boxes")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

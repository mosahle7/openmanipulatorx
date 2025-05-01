#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class MinimalDetector:
    def __init__(self):
        rospy.init_node('minimal_detector')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/d435/color/image_raw', Image, self.callback)
        
        # Create a blank black image as fallback
        self.blank = np.zeros((480,640,3), dtype=np.uint8)
        cv2.putText(self.blank, "WAITING FOR IMAGES...", (50,240), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)
        
        cv2.namedWindow("Detector", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Detector", 800, 600)
        rospy.loginfo("Minimal detector ready")

    def callback(self, msg):
        try:
            # Convert image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # SIMPLE OBJECT DETECTION (replace this with your actual logic)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_blue = np.array([100,50,50])
            upper_blue = np.array([130,255,255])
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt) > 1000:  # Minimum size
                    x,y,w,h = cv2.boundingRect(cnt)
                    cv2.rectangle(cv_image, (x,y), (x+w,y+h), (0,255,0), 2)
                    cv2.circle(cv_image, (x+w//2, y+h//2), 5, (0,0,255), -1)
            
            cv2.imshow("Detector", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr(f"Error: {str(e)}")
            cv2.imshow("Detector", self.blank)
            cv2.waitKey(1)

if __name__ == '__main__':
    try:
        detector = MinimalDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

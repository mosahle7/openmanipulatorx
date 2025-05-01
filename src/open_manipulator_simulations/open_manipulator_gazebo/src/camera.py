#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraTester:
    def __init__(self):
        rospy.init_node('camera_tester')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/d435/color/image_raw', Image, self.callback)
        cv2.namedWindow("Camera Test", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Camera Test", 800, 600)
        rospy.loginfo("Camera tester ready - Checking feed...")

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.putText(cv_image, "RAW CAMERA FEED", (50,50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)
            cv2.imshow("Camera Test", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Camera test failed: {str(e)}")

if __name__ == '__main__':
    try:
        tester = CameraTester()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

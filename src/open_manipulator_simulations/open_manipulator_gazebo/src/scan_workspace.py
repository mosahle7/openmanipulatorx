#!/usr/bin/env python
import rospy
import time
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest

class WorkspaceScanner:
    def __init__(self):
        rospy.wait_for_service('/goal_joint_space_path')
        self.joint_service = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
        
        # Slow scanning parameters
        self.scan_positions = [
            [0.5, -0.4, -0.3, 0.8],  # Left
            [0.0, -0.3, -0.2, 0.7],  # Center
            [-0.5, -0.4, -0.3, 0.8]  # Right
        ]
        self.move_time = 6.0  # Very slow movement (seconds)
        self.pause_duration = 4.0  # Long pause for detection
        self.scan_cycles = 2  # Number of complete scans

    def scan(self):
        rospy.loginfo(f"Starting {self.scan_cycles} slow scan cycles")
        for _ in range(self.scan_cycles):
            for position in self.scan_positions:
                self.move_to_position(position)
                rospy.sleep(self.pause_duration)  # Pause for detection
        rospy.loginfo("Scanning complete")

    def move_to_position(self, position):
        req = SetJointPositionRequest()
        req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        req.joint_position.position = position
        req.path_time = self.move_time
        self.joint_service(req)

if __name__ == '__main__':
    rospy.init_node('workspace_scanner')
    scanner = WorkspaceScanner()
    scanner.scan()

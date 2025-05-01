#!/usr/bin/env python
import rospy
import time
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest

def move_gripper(position, duration=2.0):
    try:
        rospy.wait_for_service('goal_tool_control', timeout=5.0)
        service = rospy.ServiceProxy('goal_tool_control', SetJointPosition)
        
        request = SetJointPositionRequest()
        request.planning_group = "gripper"
        request.joint_position.joint_name = ["gripper"]
        request.joint_position.position = [position]
        request.path_time = duration
        
        response = service(request)
        rospy.loginfo(f"Gripper movement to {position}: {response.is_planned}")
        return response.is_planned
    except Exception as e:
        rospy.logerr(f"Gripper movement failed: {e}")
        return False

def run_gripper_test():
    rospy.init_node('gripper_tester')
    rospy.loginfo("Starting gripper test")
    
    # Test positions from 0.0 gradually moving outward
    test_positions = [
        0.0,    # Neutral
        0.002,  # Slightly open
        -0.002, # Slightly closed
        0.004,
        -0.004,
        0.006,
        -0.006,
        0.008, 
        -0.008
    ]
    
    for pos in test_positions:
        rospy.loginfo(f"Testing gripper position: {pos}")
        if move_gripper(pos):
            rospy.loginfo(f"SUCCESS: Position {pos} works")
        else:
            rospy.logerr(f"FAILURE: Position {pos} failed")
        time.sleep(3.0)  # Wait between tests
    
    # Return to neutral
    move_gripper(0.0)
    rospy.loginfo("Gripper test completed")

if __name__ == "__main__":
    try:
        run_gripper_test()
    except rospy.ROSInterruptException:
        pass

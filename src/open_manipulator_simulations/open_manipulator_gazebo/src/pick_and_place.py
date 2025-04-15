#!/usr/bin/env python
import rospy
import time
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest

def move_joint(joint_name, position, path_time):
    rospy.loginfo("Waiting for service 'goal_joint_space_path' to become available...")
    rospy.wait_for_service('goal_joint_space_path', timeout=60)
    rospy.loginfo("Service 'goal_joint_space_path' is available")

    try:
        joint_position_service = rospy.ServiceProxy('goal_joint_space_path', SetJointPosition)
        request = SetJointPositionRequest()
        request.joint_position.joint_name = joint_name
        request.joint_position.position = position
        request.path_time = path_time
        response = joint_position_service(request)
        if not response.is_planned:
            rospy.logwarn("Joint movement failed")
        return response.is_planned
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False

def move_task_space(pose, path_time):
    rospy.loginfo("Waiting for service 'goal_task_space_path_position_only' to become available...")
    rospy.wait_for_service('goal_task_space_path_position_only', timeout=10)
    rospy.loginfo("Service 'goal_task_space_path_position_only' is available")

    try:
        task_position_service = rospy.ServiceProxy('goal_task_space_path_position_only', SetKinematicsPose)
        request = SetKinematicsPoseRequest()
        request.end_effector_name = "gripper"
        request.kinematics_pose.pose.position.x = pose[0]
        request.kinematics_pose.pose.position.y = pose[1]
        request.kinematics_pose.pose.position.z = pose[2]
        request.path_time = path_time
        response = task_position_service(request)
        if not response.is_planned:
            rospy.logwarn("Task space movement failed")
        return response.is_planned
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False

def pick_and_place():
    rospy.loginfo("Starting pick and place sequence")

    # Move to initial position
    if not move_joint(['joint1', 'joint2', 'joint3', 'joint4'], [0.0, 0.0, 0.0, 0.0], 2.0):
        rospy.logerr("Failed to move to initial position")
        return
    time.sleep(2)

    # Move to pick position
    if not move_joint(['joint1', 'joint2', 'joint3', 'joint4'], [0.0, -0.5, 0.3, 0.7], 2.0):
        rospy.logerr("Failed to move to pick position")
        return
    time.sleep(2)

    # Move to above the object
    if not move_task_space([0.2, 0, 0.2], 2.0):
        rospy.logerr("Failed to move above the object")
        return
    time.sleep(2)

    # Move to pick the object
    if not move_task_space([0.2, 0, 0.1], 2.0):
        rospy.logerr("Failed to move to pick the object")
        return
    time.sleep(2)

    # Close gripper
    if not move_joint(['gripper'], [0.01], 2.0):
        rospy.logerr("Failed to close the gripper")
        return
    time.sleep(2)

    # Move up with the object
    if not move_task_space([0.2, 0, 0.2], 2.0):
        rospy.logerr("Failed to move up with the object")
        return
    time.sleep(2)

    # Move to place position
    if not move_task_space([0.2, 0.2, 0.2], 2.0):
        rospy.logerr("Failed to move to place position")
        return
    time.sleep(2)

    # Open gripper
    if not move_joint(['gripper'], [0.0], 2.0):
        rospy.logerr("Failed to open the gripper")
        return
    time.sleep(2)

    rospy.loginfo("Pick and place sequence completed successfully")

if __name__ == "__main__":
    rospy.init_node('open_manipulator_python')
    pick_and_place()

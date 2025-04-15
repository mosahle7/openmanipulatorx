#!/usr/bin/env python
import rospy
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest

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

        rospy.loginfo("Joint names: %s", joint_name)
        rospy.loginfo("Joint positions: %s", position)

        response = joint_position_service(request)
        if not response.is_planned:
            rospy.logwarn("Joint movement failed")
        return response.is_planned
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False

if __name__ == "__main__":
    rospy.init_node('open_manipulator_python')

    # Test a simple joint movement
    joint_name = ['joint1']
    position = [0.0]
    path_time = 2.0

    rospy.loginfo("Joint names: %s", joint_name)
    rospy.loginfo("Joint positions: %s", position)

    if not move_joint(joint_name, position, path_time):
        rospy.logerr("Failed to move joint1")
    else:
        rospy.loginfo("Joint movement successful")

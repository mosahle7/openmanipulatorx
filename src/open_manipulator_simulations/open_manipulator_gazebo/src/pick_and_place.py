#!/usr/bin/env python
import rospy
import time
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from gazebo_msgs.msg import ModelStates

class OpenManipulatorPickPlace:
    def __init__(self):
        rospy.init_node('open_manipulator_pick_place')
        rospy.loginfo("Initializing OpenManipulator Pick and Place Controller")
        
        # Object parameters
        self.object_name = "cube"
        self.object_position = None
        self.object_size = 0.03
        
        # Subscribe to Gazebo model states
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_cb)
        
        # Wait for first model states update
        while self.object_position is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for object position...")
            rospy.sleep(0.5)
        
        # Arm parameters
        self.gripper_closed = 0.008    # Safe closing position
        self.gripper_open = -0.008     # Safe opening position
        self.lift_height = 0.15
        self.approach_height = 0.04
        self.z_offset = 0.002
        
        # Movement times
        self.fast_move_time = 2.0
        self.precise_move_time = 3.0
        self.gripper_move_time = 2.0

    def model_states_cb(self, msg):
        try:
            idx = msg.name.index(self.object_name)
            self.object_position = [
                msg.pose[idx].position.x,
                msg.pose[idx].position.y,
                msg.pose[idx].position.z + self.object_size/2
            ]
        except ValueError:
            rospy.logwarn(f"Object {self.object_name} not found")

    def move_joint(self, joint_angles, path_time):
        try:
            rospy.wait_for_service('goal_joint_space_path', timeout=5.0)
            service = rospy.ServiceProxy('goal_joint_space_path', SetJointPosition)
            
            request = SetJointPositionRequest()
            request.planning_group = "manipulator"
            request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
            request.joint_position.position = joint_angles
            request.path_time = path_time
            
            response = service(request)
            return response.is_planned
        except Exception as e:
            rospy.logerr(f"Joint movement failed: {e}")
            return False

    def move_gripper(self, position):
        try:
            safe_position = max(-0.01, min(0.01, position))
            
            rospy.wait_for_service('goal_tool_control', timeout=5.0)
            service = rospy.ServiceProxy('goal_tool_control', SetJointPosition)
            
            request = SetJointPositionRequest()
            request.planning_group = "gripper"
            request.joint_position.joint_name = ["gripper"]
            request.joint_position.position = [safe_position]
            request.path_time = self.gripper_move_time
            
            response = service(request)
            return response.is_planned
        except Exception as e:
            rospy.logerr(f"Gripper movement failed: {e}")
            return False

    def move_to_position(self, position, path_time):
        try:
            rospy.wait_for_service('goal_task_space_path_position_only', timeout=5.0)
            service = rospy.ServiceProxy('goal_task_space_path_position_only', SetKinematicsPose)
            
            request = SetKinematicsPoseRequest()
            request.planning_group = "manipulator"
            request.end_effector_name = "gripper"
            request.kinematics_pose.pose.position.x = position[0]
            request.kinematics_pose.pose.position.y = position[1]
            request.kinematics_pose.pose.position.z = position[2]
            request.path_time = path_time
            
            response = service(request)
            return response.is_planned
        except Exception as e:
            rospy.logerr(f"Position movement failed: {e}")
            return False

    def pick_object(self):
        if self.object_position is None:
            rospy.logerr("No object position available!")
            return False
            
        # Initial lift
        if not self.move_joint([0.0, -0.1, 0.05, 0.2], 2.0):
            return False
        time.sleep(2.0)
        
        # Approach
        approach_pos = [
            self.object_position[0],
            self.object_position[1],
            self.object_position[2] + self.approach_height
        ]
        if not self.move_to_position(approach_pos, 2.0):
            return False
        time.sleep(2.0)
        
        # Descend
        grasp_pos = [
            self.object_position[0],
            self.object_position[1],
            self.object_position[2] + self.z_offset
        ]
        if not self.move_to_position(grasp_pos, 1.5):
            return False
        time.sleep(1.5)
        
        # Close gripper
        if not self.move_gripper(self.gripper_closed):
            return False
        time.sleep(2.5)
        
        # Lift
        lift_angles = [0.0, -0.2, 0.1, 0.3]
        if not self.move_joint(lift_angles, 2.0):
            return False
        time.sleep(2.0)
        
        return True

    def place_object(self):
        if self.object_position is None:
            rospy.logerr("No object position available!")
            return False
            
        # Move to place
        place_pos = [
            self.object_position[0],
            self.object_position[1] + 0.15,
            self.lift_height
        ]
        if not self.move_to_position(place_pos, 2.0):
            return False
        time.sleep(2.0)
        
        # Lower
        release_pos = [
            place_pos[0],
            place_pos[1],
            self.object_position[2] + 0.01
        ]
        if not self.move_to_position(release_pos, 1.5):
            return False
        time.sleep(1.5)
        
        # Open gripper
        if not self.move_gripper(self.gripper_open):
            return False
        time.sleep(1.5)
        
        # Lift
        if not self.move_to_position(place_pos, 2.0):
            return False
        time.sleep(2.0)
        
        return True

    def run(self):
        rospy.loginfo("Starting sequence")
        
        # Home
        if not self.move_joint([0.0, 0.0, 0.0, 0.0], 2.0):
            return
        time.sleep(2.0)
        
        # Pick
        rospy.loginfo("Picking object")
        if not self.pick_object():
            rospy.logerr("Pick failed")
            return
        
        # Place
        rospy.loginfo("Placing object")
        if not self.place_object():
            rospy.logerr("Place failed")
            return
        
        # Home
        if not self.move_joint([0.0, 0.0, 0.0, 0.0], 2.0):
            return
        time.sleep(2.0)
        
        rospy.loginfo("Sequence completed")

if __name__ == "__main__":
    try:
        controller = OpenManipulatorPickPlace()
        controller.run()
    except rospy.ROSInterruptException:
        pass

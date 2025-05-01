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
        
        # Arm parameters - Very conservative gripper limits
        self.gripper_closed = -0.008   # Very conservative closed position
        self.gripper_open = 0.008      # Very conservative open position
        self.lift_height = 0.15
        self.approach_height = 0.04
        self.z_offset = 0.005          # Increased for better cube contact
        
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
            rospy.loginfo(f"Object position updated: {self.object_position}")
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
            rospy.loginfo(f"Joint movement to {joint_angles} completed: {response.is_planned}")
            return response.is_planned
        except Exception as e:
            rospy.logerr(f"Joint movement failed: {e}")
            return False

    def move_gripper(self, position):
        try:
            # Even more conservative limits
            safe_position = max(-0.008, min(0.008, position))
            
            rospy.wait_for_service('goal_tool_control', timeout=5.0)
            service = rospy.ServiceProxy('goal_tool_control', SetJointPosition)
            
            request = SetJointPositionRequest()
            request.planning_group = "gripper"
            request.joint_position.joint_name = ["gripper"]
            request.joint_position.position = [safe_position]
            request.path_time = self.gripper_move_time
            
            response = service(request)
            rospy.loginfo(f"Gripper movement to {safe_position} completed: {response.is_planned}")
            return response.is_planned
        except Exception as e:
            rospy.logerr(f"Gripper movement failed: {e}")
            return False

    def test_gripper_limits(self):
        """Test different gripper positions to find what works"""
        rospy.loginfo("Testing gripper limits")
        
        # Try conservative values
        test_positions = [0.0, 0.005, -0.005, 0.008, -0.008]
        
        for pos in test_positions:
            rospy.loginfo(f"Testing gripper position: {pos}")
            if self.move_gripper(pos):
                rospy.loginfo(f"Position {pos} WORKS")
            else:
                rospy.loginfo(f"Position {pos} FAILS")
            time.sleep(2.0)
        
        return True

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
            rospy.loginfo(f"Position movement to {position} completed: {response.is_planned}")
            return response.is_planned
        except Exception as e:
            rospy.logerr(f"Position movement failed: {e}")
            return False

    def pick_object(self):
        if self.object_position is None:
            rospy.logerr("No object position available!")
            return False
            
        # Initial lift
        rospy.loginfo("Moving to pre-pick position")
        if not self.move_joint([0.0, -0.1, 0.05, 0.2], 2.0):
            return False
        time.sleep(2.0)
        
        # Approach
        approach_pos = [
            self.object_position[0],
            self.object_position[1],
            self.object_position[2] + self.approach_height
        ]
        rospy.loginfo(f"Approaching object at: {approach_pos}")
        if not self.move_to_position(approach_pos, 2.0):
            return False
        time.sleep(2.0)
        
        # Open gripper before descending
        rospy.loginfo("Opening gripper")
        if not self.move_gripper(self.gripper_open):
            return False
        time.sleep(2.0)
        
        # Descend
        grasp_pos = [
            self.object_position[0],
            self.object_position[1],
            self.object_position[2] + self.z_offset
        ]
        rospy.loginfo(f"Descending to grasp position: {grasp_pos}")
        if not self.move_to_position(grasp_pos, 1.5):
            return False
        time.sleep(1.5)
        
        # Close gripper
        rospy.loginfo(f"Closing gripper to: {self.gripper_closed}")
        if not self.move_gripper(self.gripper_closed):
            return False
        time.sleep(3.0)  # Longer wait to ensure grip is secure
        
        # Lift slightly first to ensure grip
        slight_lift_pos = [
            grasp_pos[0],
            grasp_pos[1],
            grasp_pos[2] + 0.03  # Small lift first
        ]
        rospy.loginfo(f"Slightly lifting to test grip: {slight_lift_pos}")
        if not self.move_to_position(slight_lift_pos, 1.0):
            return False
        time.sleep(1.5)
        
        # Lift fully
        rospy.loginfo("Lifting object")
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
        rospy.loginfo(f"Moving to place position: {place_pos}")
        if not self.move_to_position(place_pos, 2.0):
            return False
        time.sleep(2.0)
        
        # Lower
        release_pos = [
            place_pos[0],
            place_pos[1],
            self.object_position[2] + 0.01
        ]
        rospy.loginfo(f"Lowering to release position: {release_pos}")
        if not self.move_to_position(release_pos, 1.5):
            return False
        time.sleep(1.5)
        
        # Open gripper
        rospy.loginfo(f"Opening gripper to: {self.gripper_open}")
        if not self.move_gripper(self.gripper_open):
            return False
        time.sleep(2.0)  # Longer wait for object to stabilize
        
        # Lift
        rospy.loginfo("Lifting after release")
        if not self.move_to_position(place_pos, 2.0):
            return False
        time.sleep(2.0)
        
        return True

    def run(self):
        rospy.loginfo("Starting pick and place sequence")
        
        # Home
        rospy.loginfo("Moving to home position")
        if not self.move_joint([0.0, 0.0, 0.0, 0.0], 2.0):
            return
        time.sleep(2.0)
        
        # Test gripper limits first
        self.test_gripper_limits()
        
        # Open gripper at start with known good position
        rospy.loginfo("Opening gripper with safe value")
        if not self.move_gripper(0.0):  # Try neutral position
            rospy.logerr("Even neutral gripper position failed!")
            return
        time.sleep(2.0)
        
        # Pick
        rospy.loginfo("Starting pick operation")
        if not self.pick_object():
            rospy.logerr("Pick operation failed")
            return
        
        # Place
        rospy.loginfo("Starting place operation")
        if not self.place_object():
            rospy.logerr("Place operation failed")
            return
        
        # Home
        rospy.loginfo("Returning to home position")
        if not self.move_joint([0.0, 0.0, 0.0, 0.0], 2.0):
            return
        time.sleep(2.0)
        
        rospy.loginfo("Pick and place sequence completed successfully")

if __name__ == "__main__":
    try:
        controller = OpenManipulatorPickPlace()
        controller.run()
    except rospy.ROSInterruptException:
        pass

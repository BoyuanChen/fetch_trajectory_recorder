#!/usr/bin/env python

# Note: fetch_moveit_config move_group.launch must be running
import rospy
# import threading
from numpy import array, sign, pi, dot
from numpy.linalg import norm
import moveit_commander
import actionlib
import tf
from control_msgs.msg import (FollowJointTrajectoryGoal,
                             FollowJointTrajectoryAction,
                             FollowJointTrajectoryResult)
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

from actionlib import SimpleActionClient
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped
from robot_controllers_msgs.msg import (QueryControllerStatesAction,
                                       QueryControllerStatesGoal,
                                       ControllerState)

from fetch_arm_control.msg import GripperState

from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

class FetchTrajectoryRecorderNode:
    def __init__(self, node_name='fetch_trajectory_recorder_node'):
        rospy.init_node(node_name)

        # Create move group interface for a fetch robot
        self.move_group = MoveGroupInterface("arm_with_torso", "base_link")

        self.gripper_names = ['l_gripper_finger_joint', 'r_gripper_finger_joint']
        self.ee_name = 'wrist_roll_link'
        self.joint_names = ['shoulder_pan_joint',
                            'shoulder_lift_joint',
                            'upperarm_roll_joint',
                            'elbow_flex_joint',
                            'forearm_roll_joint',
                            'wrist_flex_joint',
                            'wrist_roll_joint']
        self.tf_listener = tf.TransformListener()
        #set home as the initial state
        self.home_gripper_values = [0.08, 0.08]
        self.home_joint_values = [-1.54, -0.036, -1.055, -1.53, -0.535, -1.42, 2.398]
        self.gripper_values = []
        self.joint_values = []
        self.received_joint_poses = []
        self.temp_joint_values = home_joint_values
        self.temp_gripper_values = home_gripper_values

        self.gripper_values.append(home_gripper_values)
        self.joint_values.append(home_joint_values)

        rospy.Subscriber('joint_states', JointState, self.arm_trajectory_record_cb)
        #TODO: change this topic name as gripper related topics
        # rospy.Subscriber('joint_states', JointState, self.gripper_record_cb)
        # Define ground plane
        # This creates objects in the planning scene that mimic the ground
        # If these were not in place gripper could hit the ground
        self.planning_scene = PlanningSceneInterface("base_link")
        self.planning_scene.removeCollisionObject("my_front_ground")
        self.planning_scene.removeCollisionObject("my_back_ground")
        self.planning_scene.removeCollisionObject("my_right_ground")
        self.planning_scene.removeCollisionObject("my_left_ground")
        self.planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
        self.planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
        self.planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
        self.planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

    #record the arm joint values
    def arm_trajectory_record_cb(self, msg):
        import IPython
        IPython.embed()

        for name, position in zip(msg.name, msg.position):
            if name in self.joint_names:
                #if the new position is different from the previous one
                if position != temp_joint_values:
                    self.joint_values.append(position)
                    self.temp_joint_values = position
        #go back to initial position as the last state
        self.joint_values.append(home_joint_values)

        # for i, joint_name in enumerate(msg.name):
        #     if joint_name in self.joint_names:
        #         idx = self.joint_names.index(joint_name)
        #         self.received_joint_poses[idx] = msg.position[i]
        #     else:
        #         self.joint_names.append(joint_name)
        #         self.received_joint_poses.append(msg.position[i])

    #record the gripper values
    def gripper_record_cb(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in self.gripper_names:
                if position != temp_gripper_values:
                    self.gripper_values.append(position)
                    self.temp_gripper_values = position
        #go back to initial position as the last state
        self.gripper_values.append(home_gripper_values)

    #play back the trajetory just recorded
    #TODO: write a gui to trigger this function to play back
    #Otherwise, we will keep recording the joint values and do not move
    def play_back(self, joint_values, gripper_values):
        for joint in joint_values:
            if rospy.is_shutdown():
                break
            # Plans the joints in joint_names to angles in pose
            self.move_group.moveToJointPosition(self.joint_names, joint, wait=False)

            # Since we passed in wait=False above we need to wait here
            move_group.get_move_action().wait_for_result()
            result = move_group.get_move_action().get_result()

            if result:
                # Checking the MoveItErrorCode
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("Play Back ...")
                else:
                    # If you get to this point please search for:
                    # moveit_msgs/MoveItErrorCodes.msg
                    rospy.logerr("Arm goal in state: %s",
                                 move_group.get_move_action().get_state())
            else:
                rospy.logerr("MoveIt! failure no result returned.")




if __name__ == '__main__':
    try: 
         trajectory_rd = FetchTrajectoryRecorderNode()

    except rospy.ROSInterruptException: 
         pass 







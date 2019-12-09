#!/usr/bin/env python
# coding: utf-8
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


class moveit_test_fails:
    """ Ros node that ...
    """

    def __init__(self):
        self.config_ros()
        self.init_moveit()

        self.waypoints = []
        

    def config_ros(self):
        rospy.loginfo("ROS initialization...")
        rospy.init_node('moveit_test_fails', anonymous=True)
        rospy.loginfo("ROS initialization done")

    def init_moveit(self):
        rospy.loginfo("MoveIt initialization...")

        # Initialize moveit_commander:
        moveit_commander.roscpp_initialize(sys.argv)

        ## Instantiate a RobotCommander object:
        self.robot = moveit_commander.RobotCommander()

        ## Instantiate a PlanningSceneInterface object:
        self.scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a MoveGroupCommander object:
        self.group = moveit_commander.MoveGroupCommander("scara_cpe_group")

        ## We create this DisplayTrajectory publisher which is used below to publish
        ## trajectories for RVIZ to visualize.
        self.display_trajectory_publisher = rospy.Publisher(
                                        '/move_group/display_planned_path',
                                        moveit_msgs.msg.DisplayTrajectory)
        
        # Waits for RVIZ:
        # rospy.loginfo("Waiting for RVIZ...")
        # rospy.sleep(10)

        # Done:
        rospy.loginfo("MoveIt initialization done")

    def close(self):
        ## When finished shut down moveit_commander.
        moveit_commander.roscpp_shutdown()

    def move_to_pose(self, x, y):
        # Créer la position:
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = 1.0
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = 0.01

        # Définie la position:
        self.group.set_joint_value_target(pose_target, True)

        # Génère la trajectoire:
        plan = self.group.plan()

        # Déplace le bras:
        self.group.go(wait=True)

    def to_target(self, target):
        mv.group.set_named_target(target)
        mv.group.go(wait=True)


if __name__ == '__main__':
    mv = moveit_test_fails()

    # Etape 1:
    rospy.loginfo("Etape 1")
    rospy.loginfo("Pos: " + "'straight'")
    mv.to_target("straight")
    pos = [0, 0.13]
    rospy.loginfo("Pos: " + str(pos))
    mv.move_to_pose(pos[0], pos[1])

    # Etape 2:
    rospy.loginfo("Etape 2")
    pos = [0.11, 0.05]
    rospy.loginfo("Pos: " + str(pos))
    mv.move_to_pose(pos[0], pos[1])
    rospy.sleep(2)

    # Etape 3:
    rospy.loginfo("Etape 3")
    rospy.loginfo("Pos: " + "'right'")
    mv.to_target("right")
    pos = [-0.073000, 0.080000]
    rospy.loginfo("Pos: " + str(pos))
    mv.move_to_pose(pos[0], pos[1])

    rospy.sleep(3)
#!/usr/bin/env python
# coding: utf-8
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


class moveit_test_named_target:

    def __init__(self):
        self.config_ros()
        self.init_moveit()
        

    def config_ros(self):
        rospy.loginfo("ROS initialization...")
        rospy.init_node('moveit_test_named_target', anonymous=True)
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


    def to_target(self, target):
        # Défini la target où doit se déplacer le bras:
        self.group.set_named_target(target)
        
        # Génère et affiche la trajectoire:
        plan = self.group.plan()

        # Déplace le bras sur la target:
        self.group.go(wait=True)


if __name__ == '__main__':
    # Initialiation de MoveIt!:
    mv = moveit_test_named_target()

    # Liste des positions:
    named_targets = ["straight", "right", "left"]

    while not rospy.is_shutdown():
        # Génère la trajectoire pour chaque position:
        for target in named_targets:
            rospy.loginfo("Plan target: " + target)
            mv.to_target(target)
            rospy.sleep(5)

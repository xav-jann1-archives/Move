#!/usr/bin/env python
# coding: utf-8

import roslib

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal


class Joint:
    def __init__(self, motor_name):
        # Enregistre le nom:
        self.name = motor_name

        # Crée l'action:
        self.action = actionlib.SimpleActionClient('/' + self.name + '_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
        # Wait action:
        rospy.loginfo('Waiting for joint trajectory action')
        self.action.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')


    def move_joint(self, angles):
        goal = FollowJointTrajectoryGoal()

        # Noms des Joints du bras:
        goal.trajectory.joint_names = ['shoulder_1_joint', 'shoulder_2_joint']
        
        # Défini les angles pour chaque Joint:
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(3)
        goal.trajectory.points.append(point)
        
        # Envoie la commande
        self.action.send_goal_and_wait(goal)


def main():
    # Crée le contrôle du bras:
    arm = Joint('scara_cpe/scara_cpe')

    # Envoie la commande en angle:
    rospy.loginfo('Déplace le bras aux rotations: [1.5, 1.5]')
    arm.move_joint([1.5, 1.5])


if __name__ == '__main__':
    rospy.init_node('joint_position_tester')
    main()
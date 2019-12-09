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

from math import sqrt, cos, sin, acos, atan2, pi, degrees

from scara_cpe_kinematics.srv import GoToXY, GoToXYResponse

class Joint:
    def __init__(self, motor_name):
        # Enregistre le nom:
        self.name = motor_name

        # Crée l'action:
        self.action = actionlib.SimpleActionClient('/'+self.name + '_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
        # Wait action:
        rospy.loginfo('Waiting for joint trajectory action')
        self.action.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')

        # Crée le service:
        self.srv = rospy.Service('gotoxy', GoToXY, self.gotoxy)
        rospy.loginfo('Create service: gotoxy')


    def gotoxy(self, req):
        # Récupère la position de la requète:
        pos = [req.x, req.y]
        rospy.loginfo("Service 'gotoxy' -> commande: {}".format(pos))

        # Trouve les angles:
        thetas = self.XYtoThetas(pos)

        # Quitte si les angles n'ont pas été trouvé:
        if thetas[0] >= pi:
            rospy.logwarn("Erreur lors de la conversion en angle")
            return GoToXYResponse(False)

        # Déplace le bras à la position:
        rospy.loginfo("Déplacement du bras aux angles: {} radians".format(thetas[0], thetas[1]))
        self.move_joint(thetas)

        # Fin du déplacement:
        rospy.loginfo("Fin du déplacement")
        return GoToXYResponse(True)


    def move_joint(self, thetas):
        goal = FollowJointTrajectoryGoal()

        # Noms des Joints du bras:
        goal.trajectory.joint_names = ['shoulder_1_joint', 'shoulder_2_joint']
        
        # Défini les angles pour chaque Joint:
        point = JointTrajectoryPoint()
        point.positions = thetas
        point.time_from_start = rospy.Duration(3)
        goal.trajectory.points.append(point)
        
        # Envoie la commande
        self.action.send_goal_and_wait(goal)

    def XYtoThetas(self, posXY):
        x, y = posXY
        l1, l2 = 0.08, 0.047

        # Correction de la position dans le repère du bras:
        y -= l2

        # Espace accessible:
        d = sqrt(x*x + y*y)
        if d > l1 + l2 or d < l1-l2:
            rospy.logwarn("Consigne hors de l'espace atteignable")
            return (pi, pi)

        # Calcul theta2:
        t = (x*x + y*y - l1*l1 - l2*l2) / (2 * l1 * l2)
        if abs(t) > 1:
            rospy.logwarn("Impossible de calculer theta2")
            return (pi, pi)

        theta2 = acos(t)

        # Vérification theta2:
        max_theta2 = 170
        if degrees(theta2) < -max_theta2 or degrees(theta2) > max_theta2:
            rospy.logwarn("L'angle theta2 ne se trouve pas dans le domaine articulaire: -170°<{}<170".format(degrees(theta2)))
            return (pi, pi)

        ## Calcul theta1:

        # Solution coude bas:
        c2 = cos(theta2)
        s2 = sin(theta2)

        c1 = (x*l1 + x*l2*c2 + y*l2*s2) / (d*d)
        s1 = (y*l1 + y*l2*c2 - x*l2*s2) / (d*d)

        theta1 = atan2(s1,c1)

        rx = l1 * cos(theta1) + l2 * cos(theta1 + theta2)
        ry = l1 * cos(theta1) + l2 * cos(theta1 + theta2) 

        return (theta1 - pi / 2, -theta2)



from time import sleep

def main():
    # Crée le contrôle du bras:
    arm = Joint('scara_cpe/scara_cpe')

    ## Test de positions:

    # # Position de chaque point de Gazebo:
    # points = [  [-0.073000, 0.080000],  # Orange
    #             [-0.035000, 0.120000],  # Jaune
    #             [0.035000, 0.140000],   # Vert
    #             [0.044000, 0.080000]]   # Bleu

    # # Place le bras sur chaque point:
    # for p in points:
    #    arm.move_joint(p)


if __name__ == '__main__':
    rospy.init_node('joint_position_tester')
    main()

    rospy.spin()
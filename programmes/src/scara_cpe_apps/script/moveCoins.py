#!/usr/bin/env python
# coding: utf-8
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Bool


class moveitCoins:
    """ Noeud ROS qui déplace une pièce d'une position A à une position B
    """

    def __init__(self, A, B):
        self.config_ros()
        self.init_moveit()
        self.posA = A
        self.posB = B
        

    def config_ros(self):
        rospy.loginfo("ROS initialization...")
        rospy.init_node('moveCoins', anonymous=True)
        rospy.loginfo("ROS initialization done")

        # Publisher pour contrôler électro-aimant:
        self.emagnet_state_pub = rospy.Publisher("", Bool, queue_size = 10)



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

    def close():
        ## When finished shut down moveit_commander.
        moveit_commander.roscpp_shutdown()


    def move_to_pose(self, pos):
        x, y = pos

        # Créer la position
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = 0
        pose_target.orientation.w = 1.0

        # Définie la position:
        self.group.set_joint_value_target(pose_target, True)

        # Génère la trajectoire:
        plan = self.group.plan()

        # Déplace le bras:
        self.group.go(wait=True)

    def take_coin(self):
        """ Active électro-aimant
        """
        self.emagnet_state_pub(True)

    def leave_coin(self):
        """ Désactive électro-aimant
        """
        self.emagnet_state_pub(False)

    def go(self):
        """ Prend et dépose une pièce du point A au point B
        """
        # Déplace le bras sur la position A:
        self.move_to_pose(self.posA)

        # Prend la pièce:
        self.take_coin()

        # Attend un peu:
        rospy.sleep(2)

        # Déplace le bras sur la position B:
        self.move_to_pose(self.posB)

        # Dépose la pièce:
        self.leave_coin()

    

if __name__ == '__main__':
    # Positions A et B:
    A = [-0.073, 0.08]
    B = [ 0.035, 0.14]

    # Créer l'objet pour déplacer la pièce:
    mv = moveitCoins(A, B)

    # Défini la frame des positions:
    mv.group.set_pose_reference_frame("base_link")

    # Prend et dépose une pièce du point A au point B:
    mv.go()

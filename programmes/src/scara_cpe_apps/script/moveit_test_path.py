#!/usr/bin/env python
# coding: utf-8
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


class moveit_test_path:
    """ Ros node that ...
    """

    def __init__(self):
        self.config_ros()
        self.init_moveit()

        self.waypoints = []
        

    def config_ros(self):
        rospy.loginfo("ROS initialization...")
        rospy.init_node('moveit_test_path', anonymous=True)
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
        rospy.loginfo("Waiting for RVIZ...")
        #rospy.sleep(10)

        # Done:
        rospy.loginfo("MoveIt initialization done")

    def close():
        ## When finished shut down moveit_commander.
        moveit_commander.roscpp_shutdown()


    def to_target(self, target):
        mv.group.set_named_target(target)
        self.group.go(wait=True)

    def add_waypoint(self, x, y, z):
        # Créer Pose:
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = z
        wpose.orientation.w = 1.0

        # Ajoute Pose à la trajectoire:
        self.add_waypoint_pose(wpose)


    def add_waypoint_pose(self, wpose):
        self.waypoints.append(copy.deepcopy(wpose))        

    def plan_waypoints(self):
        (self.plan, fraction) = self.group.compute_cartesian_path(
                            self.waypoints,  # waypoints to follow
                            0.001,            # eef_step
                            0.0)             # jump_threshold

        self.show_trajectory()

    def go(self):
        self.group.execute(self.plan, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()

        # It is always good to clear your targets after planning with poses.
        self.group.clear_pose_targets()

    def reset_waypoints(self):
        self.waypoints = []


    def show_trajectory(self):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(self.plan)
        self.display_trajectory_publisher.publish(display_trajectory)



if __name__ == '__main__':
    mv = moveit_test_path()

    # Positions:
    poses = [  [-0.073000, 0.080000],  # Orange
               [-0.035000, 0.120000],  # Jaune
               [0.035000, 0.140000],   # Vert
               [0.044000, 0.080000]    # Bleu
            ]

    # Défini la frame des positions:
    mv.group.set_pose_reference_frame("base_link")

    # Déplace une première fois le bras:
    mv.to_target("left")

    while not rospy.is_shutdown():
        # Ajoute les points de la trajectoire pour chaque position:
        mv.reset_waypoints()
        for i in range(1, len(poses)):
            pose = poses[i]
            rospy.loginfo("Pose: " + str(pose))
            mv.add_waypoint(pose[0], pose[1], 0.01)

        # Affiche la trajectoire:
        rospy.loginfo("Affiche la trajectoire: " + str(len(poses)))
        mv.plan_waypoints()
        rospy.sleep(5)

        # Déplace le bras:
        rospy.loginfo("Déplace le bras: " + str(len(poses)))
        mv.go()

        # Inverse l'ordre des points:
        poses.reverse()
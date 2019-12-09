#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState as JointStateMoveIt
from dynamixel_msgs.msg import JointState as JointStateDynamixel
from std_msgs.msg import Int16, Float64

SHOULDER_1_INDEX = 0
SHOULDER_2_INDEX = 1

class JointController():

    def __init__(self):
        rospy.init_node('state_publisher')

        rate = 15 # Hz
        r = rospy.Rate(rate)


        self.name = ['','']
        self.current_pos = [0.0, 0.0]
        self.velocity = [0.0, 0.0]
        self.load = [0.0, 0.0]        


        # Start controller state subscribers
        rospy.Subscriber('scara_cpe2/scara_cpe_controller/shoulder_1_joint', Float64, self.publish_joints_controller)
        rospy.Subscriber('scara_cpe2/scara_cpe_controller/shoulder_2_joint', Float64, self.publish_joints_controller)
	
        # Start publisher
        self.joint_controller_1_pub = rospy.Publisher('/shoulder_1_controller/command', Float64, queue_size=10)
        self.joint_controller_2_pub = rospy.Publisher('/shoulder_2_controller/command', Float64, queue_size=10)


    def publish_joint_1_controller(self, msg):
        # Send joint states:
        self.joint_controller_1_pub.publish(msg.data)


    def publish_joint_1_controller(self, msg):
        # Send joint states:
        self.joint_controller_2_pub.publish(msg.data)


if __name__ == '__main__':
    try:
        JointController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

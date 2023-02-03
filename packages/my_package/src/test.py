#!/usr/bin/env python3
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32

class OdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")
                
        # Get static parameters
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)

        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_data_left, queue_size=1)
        self.sub_encoder_ticks_right = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_data_right, queue_size=1)
        self.sub_executed_commands = rospy.Subscriber(f'/{self.veh_name}/wheels_driver_node/wheels_cmd_executed', WheelsCmdStamped, self.cb_executed_commands, queue_size=1)

        # Publishers
        self.pub_integrated_distance_left = rospy.Publisher(f'/{self.veh_name}/my_odometry_node/encoder_data_left', Float32, queue_size=1)
        self.pub_integrated_distance_right = rospy.Publisher(f'/{self.veh_name}/my_odometry_node/encoder_data_right', Float32, queue_size=1)

        self.log("Initialized")

    def cb_encoder_data_left(self, msg):
        """ Update encoder distance information from ticks.
        """
        data = msg.data
        res = msg.resolution
        dist = 2*3.14159*self._radius*data/res

        self.pub_integrated_distance_left.publish(dist)


    def cb_encoder_data_right(self, msg):
        """ Update encoder distance information from ticks.
        """
        data = msg.data
        res = msg.resolution
        dist = 2*3.14159*self._radius*data/res

        self.pub_integrated_distance_right.publish(dist)



if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")

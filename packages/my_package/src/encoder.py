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
        #self.veh_name = rospy.get_namespace().strip("/")
        self.veh_name = os.environ["VEHICLE_NAME"]
        # Get static parameters
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)

	# ticks and offsets
        self.left_wheel_ticks = 0
        self.right_wheel_ticks = 0
        self.prevoius_left_wheel_ticks = 0
        self.previous_right_wheel_ticks = 0
        
        self.initial_left_tick = True
        self.initial_right_tick = True
        self.left_dist = 0
        self.right_dist = 0
        
        # Variables to calculate wheel radius every 1m
        self.last_dist_rad_left = 0.0
        self.last_ticks_rad_left = 0.0
        self.last_dist_rad_right = 0.0
        self.last_ticks_rad_right = 0.0

        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick', WheelsCmdStamped, callback=self.cb_encoder_data, callback_args = 'left')
        self.sub_encoder_ticks_right = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick', WheelsCmdStamped, callback=self.cb_encoder_data, callback_args = 'right')
        
        self.sub_executed_commands = rospy.Subscriber(f'/{self.veh_name}/wheels_driver_node/wheels_cmd_executed', WheelsCmdStamped, self.cb_executed_commands)


        # Publishers
        self.pub_integrated_distance_left = rospy.Publisher(f'{self.veh_name}/left_wheel_distance_traveled',Float32,queue_size=1)
        self.pub_integrated_distance_right = rospy.Publisher(f'{self.veh_name}/right_wheel_distance_traveled',Float32,queue_size=1)
        
        #self.pub_motor_commands = rospy.Publisher(f'/{self.veh_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        ## Publish what wheel commands have been executed
        #self.pub_executed_commands = rospy.Publisher(f'/{self.veh_name}/wheels_driver_node/wheels_cmd_executed', String, queue_size = 1)
        ## Publish the distance traveled by each wheel
        #self.pub_wheel_dist_traveled = rospy.Publisher(f'/motor_control_node/wheel_dist_traveled', Float64MultiArray, queue_size = 1)

        self.log("Initialized")

    	
    	
    def cb_encoder_data(self, msg, wheel):
        """ Update encoder distance information from ticks.
        """
        rospy.loginfo(f"Recieved message from {wheel} wheel\n{msg}")
        
        ticks = msg.data
        N_total = 135
        # check if it is the first reciecved messafe and store initial encoder value
        if wheel == 'left':
            if self.initial_left_tick:
            	self.previous_left_wheel_ticks = ticks
            	self.initial_left_tick = False
            	
            left_dist = (ticks - self.prevoius_left_wheel_ticks) * self._radius * 2 * math.pi / N_total
            self.left_dist = self.left_dist + left_dist
            self.previous_left_wheel_ticks = ticks
            print("Left distance: " + self.left_dist)

        elif wheel == 'right':
            if self.initial_right_tick:
            	self.previous_right_wheel_ticks = ticks
            	self.initial_right_tick = False

            right_dist = (ticks - self.prevoius_right_wheel_ticks) * self._radius * 2 * math.pi / N_total
            self.right_dist = self.right_dist + self.right_dist
            self.previous_right_wheel_ticks = ticks
            print("Right distance: " + self.left_dist)


    def cb_executed_commands(self, msg):
        """ Use the executed commands to determine the direction of travel of each wheel.
        """
        print("Excuted command: ", msg)
    
    
    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():

            self.pub_integrated_distance_left.publish(self.left_dist)
            self.pub_integrated_distance_right.publish(self.right_dist)
            rospy.loginfo("_travelledLeft: %f" %self.left_dist)
            rospy.loginfo("_travelledRight: %f" %self.right_dist)

            rate.sleep()
        

if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    # Keep it spinning to keep the node alive
    node.run()
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")

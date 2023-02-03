#!/usr/bin/env python3
print("python running ...")
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32
from std_srvs.srv import Trigger, TriggerResponse

class OdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")
        rospy.loginfo(f"Using vehicle name {self.veh_name}")

        # Get/set static parameters
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)
        self.N_total = 135
        self.initialised_ticks_left = False
        self.initialised_ticks_right = False
        self.wheel_distance_left = 0
        self.wheel_distance_right = 0

        self.calibration_distance = 1.0
        

        # Subscribing to the wheel encoders (executed commands not needed since
        # negative ticks update)
        self.sub_encoder_ticks_left = rospy.Subscriber(
            f'/{self.veh_name}/left_wheel_encoder_node/tick',\
            WheelEncoderStamped, self.cb_encoder_data, callback_args="left"
            )
        self.sub_encoder_ticks_right = rospy.Subscriber(
            f'/{self.veh_name}/right_wheel_encoder_node/tick',\
            WheelEncoderStamped, self.cb_encoder_data, callback_args="right"
            )
        # self.sub_executed_commands = rospy.Subscriber(f'/{self.veh_name}/wheels_driver_node/wheels_cmd_executed', WheelsCmdStamped, self.cb_executed_commands)

        # Publish the calculated wheel displacement distances
        self.pub_wheel_distance_left = rospy.Publisher(f'/{self.veh_name}/wheel_distance_left', Float32, queue_size=10)
        self.pub_wheel_distance_right = rospy.Publisher(f'/{self.veh_name}/wheel_distance_right', Float32, queue_size=10)

        # Service to find wheel radius
        #self.serv_wheel_calibrate = rospy.Service(f'/{self.veh_name}/wheel_calibration', Trigger, self.calibrate)

        self.log("Initialized")


    def calibrate(self, data):
        """ Find the wheel radius based on the calibration distance.
        """

        self.wheel_radius_left = self.N_total / (self.total_ticks_left-self.initial_ticks_left) * self.calibration_distance / (2.0 * np.pi)
        self.wheel_radius_right = self.N_total / (self.total_ticks_right-self.initial_ticks_right) * self.calibration_distance / (2.0 * np.pi)
        self.wheel_radius_average = (self.wheel_radius_left + self.wheel_radius_right) / 2

        #rospy.loginfo(f'Left wheel radius: {self.wheel_radius_left}')
        #rospy.loginfo(f'Right wheel radius: {self.wheel_radius_right}')
        #rospy.loginfo(f'Wheel radius average was calculated as {self.wheel_radius_average}m, assuming the calibration distance was {self.calibration_distance}m.')

        return TriggerResponse(
            success = True,
            message = f'For a calibration distance of {self.calibration_distance}: The left wheel radius is {self.wheel_radius_left}, the right wheel radius is {self.wheel_radius_right} the average wheel radius is {self.wheel_radius_average}m'
        )
        

    def cb_encoder_data(self, msg, wheel):
        """ Update encoder distance information from ticks.
        """

        rospy.loginfo(f"Recieved message from {wheel} wheel\n{msg}")

        if (wheel == "left"):
            # use flag for if ticks have not been recorded yet
            if (not self.initialised_ticks_left):
                self.total_ticks_left = msg.data
                self.initial_ticks_left = msg.data
                self.initialised_ticks_left = True

            self.dN_ticks_left = msg.data - self.total_ticks_left
            self.wheel_distance_left += 2 * np.pi * self._radius \
                * self.dN_ticks_left / self.N_total

            self.total_ticks_left = msg.data

        elif (wheel == "right"):
            # use flag for if ticks have not been recorded yet
            if (not self.initialised_ticks_right):
                self.total_ticks_right = msg.data
                self.initial_ticks_right = msg.data
                self.initialised_ticks_right = True

            self.dN_ticks_right = msg.data - self.total_ticks_right
            self.wheel_distance_right += 2 * np.pi * self._radius \
                * self.dN_ticks_right / self.N_total

            self.total_ticks_right = msg.data

        else:
            raise NameError("wheel name not found")


    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.pub_wheel_distance_left.publish(self.wheel_distance_left)
            self.pub_wheel_distance_right.publish(self.wheel_distance_right)
            rospy.loginfo("Left distance: %f" %self.wheel_distance_left)
            rospy.loginfo("Right distance: %f" %self.wheel_distance_right)

            rate.sleep()

if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    rospy.loginfo("wheel_encoder_node is up and running...")
    node.run()
    # Keep it spinning to keep the node alive
    rospy.spin()
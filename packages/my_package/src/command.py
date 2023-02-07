#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped, LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern
from std_msgs.msg import String
from time import sleep
from std_msgs.msg import Header, Float32

class CommandNode(DTROS):


    def __init__(self,node_name):
        super(CommandNode,self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self.wheelpub = rospy.Publisher(
            '/csc22932/wheels_driver_node/wheels_cmd',
            WheelsCmdStamped,
            queue_size=1)
        self.speed = 0.3


    def forward(self,time):
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = self.speed
        msg.vel_right = self.speed
        self.wheelpub.publish(msg)

        self.set_leds("forward")
        rospy.sleep(time)

    def left_turn(self,time):
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = self.speed
        msg.vel_right = self.speed + 0.1
        self.wheelpub.publish(msg)
        
        self.set_leds("left_turn")
        rospy.sleep(time)

    def right_turn(self,time):
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = self.speed + 0.1
        msg.vel_right = self.speed
        self.wheelpub.publish(msg)
        
        self.set_leds("right_turn")
        rospy.sleep(time)

    def rotate(self,time):
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = self.speed
        msg.vel_right = -self.speed
        self.wheelpub.publish(msg)
        rospy.sleep(time)


    def stop(self):
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.vel_left = 0.0
        msg.vel_right = 0.0
        self.wheelpub.publish(msg)
        self.set_leds("stop")

    def set_leds(self,mode):
        rospy.wait_for_service('/csc22932/led_emitter_node/set_custom_pattern')
        try:
            service = rospy.ServiceProxy('/csc22932/led_emitter_node/set_custom_pattern',
                                         SetCustomLEDPattern)
            msg = LEDPattern()
            msg.color_list = ['green', 'green', 'green', 'blue', 'blue']
            msg.color_mask = [1, 1, 1, 1, 1]
            msg.frequency = 1.0
            msg.frequency_mask = [0, 0, 0, 0, 1]
            
            if mode == "stop":
                msg.color_list[1] = 'red'
                msg.color_list[3] = 'red'
            elif mode == "left_turn":
                msg.color_list[1] = 'yellow'
                msg.frequency = 1.0
                msg.frequency_mask[1] = 1
            elif mode == "right_turn":
                msg.frequency = 1.0
                msg.color_list[3] = 'yellow'
                msg.frequency_mask[3] = 1
            else:
                pass
            
            response = service(msg)
            rospy.loginfo(response)
            
        except rospy.ServiceException:
            print("Service call failed")


    def run(self):
        rospy.sleep(2)
        self.forward(2)
        self.stop()
        self.left_turn(2)
        # self.right_turn(2)
        #self.forward(2)
        self.stop()




if __name__ == '__main__':
    node = CommandNode(node_name="my_command_node")
    node.run()
    node.spin()
#!/usr/bin/env python3
import numpy as np
import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32

class OdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(EncoderNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)

        # Subscribing to the wheel encoders
        #the fourth argument is the callback argument for the cb_encoder_data method
        #MAY NEED TO SAY callback_args = "left"/"right" for the fourth arg in the below two lines
        self.sub_encoder_ticks_left = rospy.Subscriber("/" +  os.environ['VEHICLE_NAME'] + "/left_wheel_encoder_node/tick",WheelEncoderStamped,self.cb_encoder_data,callback_args="left")
        self.sub_encoder_ticks_right = rospy.Subscriber("/" + os.environ['VEHICLE_NAME'] + "/right_wheel_encoder_node/tick",WheelEncoderStamped,self.cb_encoder_data,callback_args="right")
        #self.sub_executed_commands = rospy.Subscriber("/" +  os.environ['VEHICLE_NAME'] + "/wheels_driver_node/wheels_cmd_executed",WheelsCmdStamped,self.cb_executed_commands)

        # Publishers
        #self.pub_integrated_distance_left = rospy.Publisher(...)
        #self.pub_integrated_distance_right = rospy.Publisher(...)
        self.pub_giveVelocity = rospy.Publisher("wheels_driver_node/wheels_cmd", WheelsCmdStamped)

        self.log("Initialized")




    def publishVelocity(self):
        #get the duckie bot name
        duckiebotname = os.environ['VEHICLE_NAME']
        # publish message every 1 second
        rate = rospy.Rate(10) # 1Hz
        while not rospy.is_shutdown():
            
            self.pub_giveVelocity.publish()
            
            rate.sleep()



    def cb_encoder_data(self, msg, wheels):
        """ 
        Update encoder distance information from ticks.
        Formula for ticks: (deltaX = 2 * pi * radius * ticks) / total

        deltaX = distance travelled for one wheel
        radius = 0.0318m
        ticks = number of ticks measured from each wheel
        total = the number of ticks in one full revolution (135)

        length of axel on duckiebot = 0.05m
        """
        radius = 0.0318
        total = 135
        deltaX = 2 * math.pi * radius * msg.data * total

        if wheels == "left":
            #something
            print()
        elif wheels == "right":
            #something
            print()


    


    def cb_executed_commands(self, msg):
        """ Use the executed commands to determine the direction of travel of each wheel.
        """

if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    # Keep it spinning to keep the node alive
    node.publishVelocity()
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")
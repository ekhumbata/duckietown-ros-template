#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String

#found this by: running gui tools -> cmd 'rostopic list' -> find topic you want -> 'rostopic info [topic name]' 
#then import the files wanted
from sensor_msgs.msg import CameraInfo

class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher

        subscriberString = "/" +  os.environ['VEHICLE_NAME'] + "/camera_node/camera_info"

        self.sub = rospy.Subscriber(subscriberString, CameraInfo, self.callback)
        print("got here!")
        #self.sub = rospy.Subscriber('chatter', String, self.callback)

    def callback(self, data):
        #print("in the callback")
        rospy.loginfo(f"""Width of image {data.width}, Height of image {data.height}""")



if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    # keep spinning
    rospy.spin()
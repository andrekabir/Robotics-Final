#!/usr/bin/env python3

import rospy
from geometry_msgs import Twist
from robotics_final_project import PerceivedLocation, AmbulanceHonk

class MasterCar(object):
    def __init__(self):
        rospy.init_node("master_car_node")

        # Names of ROS topics we listen to
        self.motion_topic = 'car/motion_planner'
        self.perception_topic = 'car/perception'
        self.honk_topic = 'ambulance/honk'
        
        # Publisher for velocity commands
        self.vel_publisher = rospy.Publisher('car/cmd_vel')

        # Subscribe to motion planner
        rospy.Subscriber(self.motion_topic, Twist, self.__callback_motion_planner__)
        rospy.Subscriber(self.perception_topic, PerceivedLocation, self.__callback_perception__)
        rospy.Subscriber(self.honk_topic, AmbulanceHonk, self.__callback_honk__)

        pass

    def __callback_motion_planner__(self, cmd: Twist):
        pass

    def __callback_perception__(self, msg: PerceivedLocation):
        pass

    def __callback_honk__(self, msg: AmbulanceHonk):
        pass

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = MasterCar()
    node.run()
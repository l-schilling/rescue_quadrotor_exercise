#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import tf
import numpy as np
import threading

class controller():
    def __init__(self):
        self.pose_subscriber = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, self.update_pose)
        #define velocity publisher here

    def update_pose(self, data):
        #update current pose of the robot
        self.pose_stamped = data
        self.roll, self.pitch, self.yaw=self.quaternion_to_euler(data.pose)

    def quaternion_to_euler(self, robot_pose):
        #transforms robot position from quaternion to roll, pitch, yaw
        quaternion = (
            robot_pose.orientation.x,
            robot_pose.orientation.y,
            robot_pose.orientation.z,
            robot_pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        return roll, pitch, yaw

    def get_init_goal(self):
        self.goal_x = input("Set your x goal:")
        self.goal_y = input("Set your y goal:")
        self.goal_z = input("Set your z goal:")
        self.goal_angle = input("Set your angle goal:")
        print("goal selected")

    def get_goal(self):
        while not rospy.is_shutdown():
            self.goal_x = input("Set your x goal:")
            self.goal_y = input("Set your y goal:")
            self.goal_z = input("Set your z goal:")
            self.goal_angle = input("Set your angle goal:")

    def control_law(self):
        #define pcontroller for x,y,z position and the yaw angle
        #use the self.goal_ variables, self.pose_stamped for current x,y,z and self.yaw for the current angle






        return ux,uy,uz,uw

    def move2goal(self):
        while not rospy.is_shutdown():
            ux,uy,uz,uw = self.control_law()
            #define and publish Twist Message here


            rospy.sleep(0.01)




if __name__ == '__main__':	
    try:
        rospy.init_node('p_controller')
        p_controller = controller()
        rospy.sleep(1) # waiting for the initial pose update
        p_controller.get_init_goal()
        t1 = threading.Thread(name='thread1', target=p_controller.get_goal)
        t2 = threading.Thread(name='thread2', target=p_controller.move2goal)
        t1.start()
        t2.start()
    except rospy.ROSInterruptException: pass

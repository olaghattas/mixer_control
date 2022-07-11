#!/usr/bin/env python

#import the dependencies
 
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
# from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
import transformation_utilities as tu
import numpy as np
# import tf2_ros
# import tf.transformations 
# import tf2_geometry_msgs
# import os


class Input_Sparse:
    def __init__(self,K_gains):
        self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.sub_img_detec =  rospy.Subscriber("/tag_detections", control, self.feedback_control_callback)
        self.vel = Twist()

###TODO this function
    def feedback_control_callback(self):


    def compute_input_sparse(self):
        u = self.compute_u(rot_w_at)

        alpha = 0.5
        beta = 0.1

        ux = alpha/np.linalg.norm(u)
        ux = ux@np.array([[np.cos(self.orien)],[np.sin(self.orien)]]).T
        ux = ux@U

        wz = beta/np.linalg.norm(u)
        wz = wz@np.array([[0],[0],[1]]).T
        cross = np.cross(np.array([[np.cos(self.orien)],[np.sin(self.orien)],[0]]),np.array([[u],[0]]))
        wz = wz@cross

        # adjust the velocity message
		self.vel.angular.z = wz
		self.vel.linear.x = ux
		#publish it
		self.pub.publish(self.vel)

## CHECK IF THIS ORIENTATION TO BE USED
## Get the orientation from different apriltags first from one apriltag 
    def robot_pose(self,at_id):

        # Finally, robot pose wrt. the world: WTR = WTAT & (RTAT)-1
        self.pose = self.landmarks[at_id].tf.dot(np.linalg.inv(apriltag_to_robot))

        # Update the orientation vector of the robot from the seen AprilTag
        # because we only want the X vector

        ## how is this the orientation
        orientation = self.pose[:3,0].flatten()
        orientation[2] = 0
        # Finally, update the orientation vector
        self.orientation = orientation
        self.orientation /= np.linalg.norm(self.orientation)

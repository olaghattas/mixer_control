#!/usr/bin/env python

#import the dependencies
 
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
import transformation_utilities as tu
import numpy as np
import tf2_ros
import tf.transformations 
import tf2_geometry_msgs
import os

class Jackal:
    def __init__(self,K_gains,K_added): 
        self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.sub_img_detec =  rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.apriltag_callback)
        self.vel = Twist()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.apriltags_list = list()
        self.K_gains = K_gains
        self.K_addded = K_added
        self.position_landmark_inworld = {1:[-8.24439, -5.8941, 0.5, 0, -0, 0,],
                                          2:[-8.27337, -1.68115, 0.5, 0, -0, 0],
                                          3:[-8.23126, 1.44051, 0.5, 0, -0, 0],
                                          4:[-5.65321, 3.03627, 0.5, 0, 0, -1.59951],
                                          5:[-2.52459, 3.22243, 0.5, 0, 0, -1.54972],
                                          6:[1.31312, -2.15597, 0.5, 0, 0, -3.09205],
                                          7:[1.45869, -6.20846, 0.5, 0, -0, 3.13477],
                                          8:[-2.46765, -8.80024, 0.5, 0, -0, 1.56763],
                                          9:[-5.70343, -8.65199, 0.5, 0, -0, 1.52896],
                                          11:[-4.49279, 0.886215, 0.5, 0, -0, 0],
                                          12:[-4.10288, -2.02095, 0.5, 0, 0, -3.12059],
                                          13:[-1.98548, -2.07486, 0.5, 0, -0, 0],
                                          14:[-1.93215, -4.18616, 0.5, 0, -0, 0],
                                          15:[1.22551, 1.92208, 0.5, 0, -0, 3.11799]
                                         }

    def apriltag_callback(self,msg):
        apriltags_list = list()

        if msg.detections:
            # '''If there's an AprilTag in the image'''
            # min_distance = np.inf
            # selected_id = 0
            # selected_apriltag = None
            for at in msg.detections:
            #     dist = np.linalg.norm([tmp_point.x, tmp_point.y, tmp_point.z])
            #     if dist < min_distance:
            #         min_distance = dist
            #         selected_id = at.id[0]
                selected_apriltag = at.pose.pose
                #change frame from camera to baselink
                source_frame = "front_realsense_gazebo"
                transform = self.tfBuffer.lookup_transform("base_link", source_frame, rospy.Time(0), rospy.Duration(1.0))
                pose_transformed = tf2_geometry_msgs.do_transform_pose(selected_apriltag, transform)
                '''Now add the apriltag to seen apriltags_list'''
                transformation_at = tu.msg_to_se3(pose_transformed.pose)
                apriltags_list.append([at.id[0],transformation_at])

            self.apriltags_list = apriltags_list
            # robot coordinates in se(3)
            # at_to_robot = tu.msg_to_se3(selected_apriltag)
            self.fiducial_flag = True
            self.op_flow_flag = False
        else:
            self.fiducial_flag = False
            self.op_flow_flag = True
            rospy.logwarn("Can't find an AprilTag in the image!")
            self.apriltags_list = list()
    
    def euler_to_rotmatrix(self,euler_angle):
        #########
        #   The full rotation matrix for the elemental rotation order yaw-pitch-roll 
        #   (u, v, w) are the three Euler angles (roll, pitch, yaw), corresponding to rotations around the x, y and z axes 
        #   c and s are shorthand for cosine and sine
        #########
        euler_angle = np.array(euler_angle).T
        c_u = np.cos(angles[0])
        s_u = np.sin(angles[0])
        c_v = np.cos(angles[1])
        s_v = np.sin(angles[1])
        c_w = np.cos(angles[2])
        s_w = np.sin(angles[2])
        matrix = np.array([[c_v*c_w, s_u*s_v*c_w-c_u*s_w, s_u*s_w+c_u*s_v*c_w],\
        [c_v*s_w, c_u*c_w+s_u*s_v*s_w, c_u*s_v*s_w-s_u*c_w],\
        [-s_v, s_u*c_v, c_u*c_v]])
        return matrix


    def dist_robot_landmark(self): # rot_inthisframe_thisframe ## add to function rot_w_at
        y = []
        at_ids = []
        for at in self.apriltags_list: # at is tag.pose.pose.pose
            # measured displacement of aptag in robot coordinates 
            displacement_robcoor= at[1][:3,3]
            # measured orientation of apriltag wrt robot 
            orientation_aptag_wrt_rob = at[1][:3,:3]
            #orientation of aptag wrt world frame
            orientation_aptag_wrt_world=self.euler_to_rotmatrix(self.position_landmark_inworld[at[0]])
            # disp is the measured displacement of the apriltag wrt robot in world coordinates
            disp = np.dot(orientation_aptag_wrt_world,orientation_aptag_wrt_rob.T)
            disp = np.dot(y,displacement_robcoor)
            at_ids.append(at[0])

            y.append(disp[1,:2].T) # should be of size 2,1
            print("y",y)
        return y, at_ids

    def compute_u(self, rot_w_at, at_ids): # n = num of aptags seen K=[2n,?] y = [2n,1]
        y = self.dist_robot_landmark(rot_w_at)
        K_seen = None
        
        for id in at_ids:
        if K_seen is None:
            K_seen = self.K_gains[:,id-1:id+1]
        else:
            K_see = np.hstack((K_seen,self.K_gains[:,id-1:id+1]))
        
        u = np.dot(K_seen,y.reshape(-1,1)) + self.K_addded
        return u

    def to_tf(self,pos,ori):
        return np.block([[np.array(ori),pos.reshape((-1,1))],[0,0,0,1]])

def read_matrix(csv_dir):
    with open(csv_dir,'r') as f:
        return np.genfromtxt(f,delimiter=',')

home_dir = os.environ["HOME"]

if __name__ == "__main__":

    rospy.init_node("jackal_iros_node")
    
    shared_path = os.environ["HOME"]+"/catkin_ws/src/output_feedback_controller/csv/"
    K_gains_path= shared_path + "K_gains.csv"
    K_gains = read_matrix(K_gains_path)

    K_added_path= shared_path + "K_added.csv"
    K_added = read_matrix(K_added_path)


# ### TO DOO ARRAY WITH LANDMARK POSITION WRT WORLD
    #  jackal = Jackal(K_gains)
    jackal = Jackal(K_gains,K_added)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        jackal.dist_robot_landmark()
	r.sleep()   


	# while not rospy.is_shutdown():
    #     if len(jackal.apriltags_list):
	# 	    jackal.input_sparse()
    #     else:
    #         print(no apriltags)
	# 	r.sleep()

 



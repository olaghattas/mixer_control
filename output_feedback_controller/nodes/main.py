#!/usr/bin/env python

#import the dependencies
 
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
import numpy as np
import transformation_utilities as tu
import so3_utilities as so
import transformations as tr
import tf2_ros
import tf.transformations 
import tf2_geometry_msgs
from vision_based_nav import vision_based_nav
from optical_fiducial_combined.msg import Control
import numpy as np


class Main():
    def __init__(self) -> None:
       
       
        self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
		self.sub_img_detec =  rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.detection_callback)
		# Odometry will be (x,y,theta)
        self.odometry = None
        self.detected = None
        self.start_point = [0,0]
        self.apriltags_list = list()
        self.of_linear = None
        self.of_angular = None
       
        # if not SIMULATION:
        #     self.odom_sub = rospy.Subscriber("/odom",Odometry,self.odometry_callback)       
    
    


class Jackal:
	def __init__(self, K_gains):
        self.robot_publisher = "/cmd_vel"
		self.pub=rospy.Publisher(self.robot_publisher ,Twist,queue_size=1)
		self.sub_img_detec =  rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.detection_callback)
		self.vel = Twist()
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)
		self.saved_time = rospy.Time.now()
		self.point = None
        self.apriltags_list = list()
        self.control_values = rospy.Subscriber("controller", Control, self.callback)
    
    def callback(self, data): 
        self.of_linear = data.linear 
        self.of_angular = data.angular 
       

    def apriltag_callback(self,msg):
        apriltags_list = list()
        
        if msg.detections:
            '''If there's an AprilTag in the image'''
            min_distance = np.inf
            selected_id = 0
            selected_apriltag = None

            for at in msg.detections:
                #change frame from camera to baselink
                source_frame = "front_realsense_gazebo"
                transform = self.tfBuffer.lookup_transform("base_link", source_frame, rospy.Time(0), rospy.Duration(1.0))
                pose_transformed = tf2_geometry_msgs.do_transform_pose(selected_apriltag, transform)
                '''Now add the apriltag to seen apriltags_list'''
                apriltags_list.append([at.id[0],pose_transfromed.pose.pose.pose])

            self.apriltags_list = apriltags_list
            # robot coordinates in se(3)
            # at_to_robot = tu.msg_to_se3(selected_apriltag)
            
        else:
            rospy.logwarn("Can't find an AprilTag in the image!")
            self.apriltags_list = list()

    
    def dist_robot_landmark(self, rot_w_at): # rot_thisframe_inthisframe
        y = []
        for at in self.apriltags_list: # at is tag.pose.pose.pose
            at_position = at.position
            # distance from apriltag to robot 
            dist = np.linalg.norm([at_position.x,at_position.y, at_position.z])
            # rotation matrix of apriltag and robot 
            rot_rob_at = tu.msg_to_se3(at)[:3,:3]
            y = rot_w_at@(rot_rob_at).T
            y.append(y[:2,3]) # should be of size 2,1
        return y

    def compute_u(self,rot_w_at): # n = num of aptags seen K=[2n,?] y = [2n,1]
        # y = self.dist_robot_landmark(rot_w_at)
        K = 0
        k = 1
        u = K@y.reshape(-1,1) + k
        return u

    def to_tf(self,pos,ori):
        return np.block([[np.array(ori),pos.reshape((-1,1))],[0,0,0,1]])

### To DO: get orientation

    def input_sparse(self):
        u = self.compute_u(rot_w_at)

        alpha = 0.5
        beta = 0.1

        ux = alpha/np.linalg.norm(u)
        ux = ux@[[np.cos(self.orien)],[np.sin(self.orien)]].T
        ux = ux@U

        wz = beta/np.linalg.norm(u)
        wz = wz@[[0],[0],[1]].T
        cross = np.cross([[np.cos(self.orien)],[np.sin(self.orien)],[0]],[[u],[0]])
        wz = wz@cross

        # adjust the velocity message
		self.vel.angular.z = wz
		self.vel.linear.x = ux
		#publish it
		self.pub.publish(self.vel)

    def optical_flow_controller(self):
        self.vel.angular.z = self.of_angular 
        self.vel.linear.x = self.of_linear
        self.pub.publish(self.vel)

home_dir = os.environ["HOME"]

if __name__ == "__main__":

    rospy.init_node("jackal_iros_node")
    
    shared_path = os.environ["HOME"]+"/jackal_ws/src/optical_fiducial_combined/csv/"
    K_dir = shared_path + "K_gains.csv"
    K_gains = read_matrix(K_dir)

### TO DOO ARRAY WITH LANDMARK POSITION WRT WORLD
    jackal = Jackal(K_gains)

    r = rospy.Rate(10)
	while not rospy.is_shutdown():
        if len(jackal.apriltags_list):
		    jackal.input_sparse()
        else:
            print(no apriltags)
            jackal.optical_flow_controller
		r.sleep()

        
#!/usr/bin/env python

import rospy
import tf
import PyKDL
import numpy as num
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from tf_conversions import posemath


from std_srvs.srv import Empty, EmptyResponse

from camera_robot_calibration_module import camera_robot_calibration

def save_pose_to_file(f,P):
    f = open(f.name, 'w') #so that sleepy people can compute whatever they want
    f.write(str(P.position.x)+'\t')
    f.write(str(P.position.y)+'\t')
    f.write(str(P.position.z)+'\t')
    f.write(str(P.orientation.x)+'\t')
    f.write(str(P.orientation.y)+'\t')
    f.write(str(P.orientation.z)+'\t')
    f.write(str(P.orientation.w)+'\n')
    f.close()


class camera_robot_calibration_ros():
    def __init__(self):
        #read values from properties
        self.base_frame_name=rospy.get_param('base_frame_name', 'world') #'/base'
        self.camera_frame_name=rospy.get_param('camera_frame_name', 'kinect2_link') #'/kinect2_1_rgb_optical_frame'
        self.robot_ee_frame_name=rospy.get_param('robot_ee_frame_name', 'gripper_r_base') #'/right_gripper_base'
        self.marker_frame_name=rospy.get_param('marker_frame_name', 'calibration_tag') #'/ros_groovy'
        self.sensor_name = rospy.get_param('sensor_name', 'kinect2')
        self.file_path = rospy.get_param('file_path', '.')
        self.publish_tf = False
        #self.save=rospy.get_param('auto_save_to_file', True)

        #rospy.sleep(20.0)

        #nominal positions of camera w.r.t world and marker mounted in the robot
        #this two frames are published
        unity_frame=Pose()
        unity_frame.orientation.w=1;
        unity_frame.position.z=0.2;
        # marker in ee
        self.ee_P_m=rospy.get_param('robot_ee_marker', unity_frame);
        # camera base in world
        R=PyKDL.Rotation(PyKDL.Vector( -0.220699,     0.66163,   -0.716615),
                         PyKDL.Vector(0.13597,    0.748429,    0.649128),
                         PyKDL.Vector(0.965818,   0.0458236,   -0.255139))
        #PyKDL.Rotation.RPY(0,0,0.7)
        init_camera_pose=PyKDL.Frame((R),
                                     PyKDL.Vector(  0.126191,  0.00936311,    -1.21054))
                                     
        self.w_P_c=rospy.get_param('nominal_pose_camera', posemath.toMsg(init_camera_pose));

        #setup TF LISTENER AND BROADCASTER
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        #vectors of saved data
        self.crc=camera_robot_calibration()

        #create services
        self.s1 = rospy.Service('read_tfs', Empty, self.read_tfs)
        self.s2 = rospy.Service('compute_frames', Empty, self.compute_frames)
        self.s3 = rospy.Service('reset_frames', Empty, self.reset_frames)
        rospy.loginfo('servers created')
        #save to file
        self.f= open(self.file_path+"/"+self.sensor_name+".txt", 'w')

    def reset_frames(self,req):
        """empty vectors to reset algorithm"""
        self.crc.reset_frames()
        return EmptyResponse()


    def current_pose(self, target_frame, origin_frame):
        if self.listener == None:
            rospy.loginfo("No transform listener available. Constructing new one.")
            self.listener = tf.TransformListener()

        try:
            print "From: " + str(origin_frame) + "   to: " + str(target_frame)
            now = rospy.Time(0)
            self.listener.waitForTransform(target_frame, origin_frame, now, rospy.Duration(0.3))

            (trans, rot) = self.listener.lookupTransform(origin_frame,target_frame, now)

            pose = Pose()
            pose.position.x = trans[0]
            pose.position.y = trans[1]
            pose.position.z = trans[2]
            pose.orientation.x = rot[0]
            pose.orientation.y = rot[1]
            pose.orientation.z = rot[2]
            pose.orientation.w = rot[3]
            return pose
        except (tf.LookupException, tf.ConnectivityException):
            print "Service call failed: %s" % e
            return 0




    def compute_frames(self,req):
            #read nominal poses, and set as initial positions

            self.crc.set_intial_frames(posemath.fromMsg( self.w_P_c),
                                        posemath.fromMsg(self.ee_P_m))


            #do several iteration of estimation

            n_comp=6
            residue_max=[]
            residue_mod=[]
            for i in range(n_comp):
                print '\ncurrent position'
                print self.crc.w_T_c.p
                residue= self.crc.compute_frames();
                r2=residue.transpose()*residue
                residue_mod.append( num.sqrt (r2[0,0]))
                residue_max.append(num.max(num.abs(residue)))
            print '\nresidue_mod'
            print residue_mod
            print '\nresidue_max'
            print residue_max
            #put result back in parameter
            print '\nee_T_m'
            print self.crc.ee_T_m
            print '\nw_T_c'
            print self.crc.w_T_c
            self.ee_P_m = posemath.toMsg(self.crc.ee_T_m)
            self.w_P_c=posemath.toMsg(self.crc.w_T_c)

            print()

            save_pose_to_file(self.f, self.w_P_c)

            print ("calibration saved in " + self.f.name)
            print ("tf is being published, you can check calibration result.")
            self.publish_tf = True

            return EmptyResponse();

    def read_tfs(self,req):
        #marker w.r.t. camera\print

        ok=True

        #read target w.r.t. camera 

        w_P_ee=self.current_pose(self.robot_ee_frame_name,self.base_frame_name)
        if(w_P_ee==0):
            ok=False
        c_P_m=self.current_pose(self.marker_frame_name,self.camera_frame_name)
        if(c_P_m==0):
            ok=False

        #ee w.r.t. base


        if ok:
            print self.base_frame_name+" -> "+self.robot_ee_frame_name
            print w_P_ee
            print self.camera_frame_name + " -> " + self.marker_frame_name
            print c_P_m
            #save data
            #save_pose_to_file(self.f,w_P_ee)
            #save_pose_to_file(self.f,c_P_m)
            self.crc.store_frames(posemath.fromMsg( w_P_ee),posemath.fromMsg(c_P_m))
            print "saved so far"
            print len(self.crc._w_T_ee)
        else:
            print "error in retrieving a frame"

        return EmptyResponse();


    def publish_tfs(self):
        #publish the estimated poses of marker and camera, in tf

        if not self.publish_tf:
            return

        self.br.sendTransform((self.w_P_c.position.x,self.w_P_c.position.y,self.w_P_c.position.z),
                         (self.w_P_c.orientation.x,self.w_P_c.orientation.y,self.w_P_c.orientation.z,self.w_P_c.orientation.w),
                         rospy.Time.now(),
                         self.camera_frame_name,
                         self.base_frame_name)

        self.br.sendTransform((self.ee_P_m.position.x,self.ee_P_m.position.y,self.ee_P_m.position.z),
                         (self.ee_P_m.orientation.x,self.ee_P_m.orientation.y,self.ee_P_m.orientation.z,self.ee_P_m.orientation.w),
                         rospy.Time.now(),
                         self.marker_frame_name+"_nominal",
                         self.robot_ee_frame_name)





            #

if __name__ == '__main__':


    rospy.init_node('camera_robot_calibration')
    est=camera_robot_calibration_ros()

    while not rospy.is_shutdown():
      est.publish_tfs()
      rospy.sleep(0.01)

    rospy.spin()

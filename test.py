#!/usr/bin/env python
import rospy
from jsk_recognition_msgs.msg import PeoplePoseArray
import numpy as np

global m
m = None

def callback(msg): 
    global m
    m = msg
    poses_list = msg.poses
    n_person = len(poses_list)
    for i in range(n_person):
        poses_person = poses_list[i]
        c_mass = estimate_pos_human(poses_person)
        print c_mass

def estimate_pos_human(msg_people_pose):
    poses = msg_people_pose.poses
    n_part = len(poses)
    c_mass = np.zeros(3)

    for i in range(n_part):
        pose = poses[i]
        pos = pose.position
        c_mass[0] += pos.x
        c_mass[1] += pos.y
        c_mass[2] += pos.z

    c_mass /= n_part
    return c_mass

if __name__=='__main__':
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('/edgetpu_human_pose_estimator/output/poses', PeoplePoseArray, callback)
    rospy.spin()



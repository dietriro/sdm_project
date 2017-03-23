#!/usr/bin/env python

# ROS
import rospy
from geometry_msgs.msg import Pose, Pose2D, PoseStamped, PoseArray
from tf import transformations
from sdm_project.srv import *
# Local
from ARCThetaStar_new import ARCThetaStar

import numpy as np


def pose_to_pose2d(pose_stamped):
    # Convert quaternion to euler angles
    quaternion = (
        pose_stamped.pose.orientation.x,
        pose_stamped.pose.orientation.y,
        pose_stamped.pose.orientation.z,
        pose_stamped.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)

    # Create 2D-Pose
    pose2d = Pose2D()
    pose2d.x = pose_stamped.pose.position.x
    pose2d.y = pose_stamped.pose.position.y
    pose2d.theta = euler[2]

    return pose2d


def pose2d_to_pose(pose2d):
    # Convert euler angles to quaternion
    quaternion = transformations.quaternion_from_euler(0, 0, pose2d.theta)
    
    print(quaternion)
    
    # Create Pose
    pose = Pose()
    pose.position.x = pose2d.x
    pose.position.y = pose2d.y
    pose.position.z = 0
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    return pose


def handle_get_path(req):
    print('Service called, path planner is executed.')
    
    # Initialize np map
    map = np.zeros((len(req.map.dim1), len(req.map.dim1[0].dim2)), int)
    x = 0
    
    # Convert 2d-array map to np map
    for row in req.map.dim1:
        map[x, :] = row.dim2
        x += 1
        
    # TODO: Set correct threshold
    # map[map < 200] = 0
    map[map == 255] = 0

        
    # Create path planner node
    path_planner = ARCThetaStar(map, r_max=500, t_factor=0)
    
    # Calculate path for given start and goal
    path = path_planner.calculate_path(pose_to_pose2d(req.start),
                                       pose_to_pose2d(req.goal))
    
    if path is None:
        return PoseArray()
    
    print(path)
    
    pose_path = PoseArray()
    pose_path.header.stamp = rospy.get_rostime()
    pose_path.header.frame_id = req.start.header.frame_id
    pose_path.header.seq = 1
    for pose2d in path:
        # Create new PoseStamped from Pose2D
        new_pose = Pose()

        new_pose = pose2d_to_pose(pose2d)
        new_pose.position.x *= 0.05
        new_pose.position.y *= 0.05
        
        print(new_pose.position)

        # Append it to the pose path
        pose_path.poses.append(new_pose)
        
    print('Service successfully completed.')
    
    return pose_path


if __name__ == '__main__':
    ##### ROS #####
    # Initialize this node
    rospy.init_node('sdm_path_planner', anonymous=True)
    
    # Service
    path_planner_srv = rospy.Service('arc_theta_star_global_planner', arc_theta_star_get_plan, handle_get_path)
    
    # Subscriber
    # sub_sensor_data = rospy.Subscriber('/robot0/laser_0', LaserScan, cb_sensor_data)
    
    # Publisher
    # pub_pose_est = rospy.Publisher('/rt_local_dnn/robot0/pose', PoseStamped, queue_size=10)
    
    # Register function that is called before shutdown
    # rospy.on_shutdown(sess.close)
    
    print('Service established and waiting for calls.')
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
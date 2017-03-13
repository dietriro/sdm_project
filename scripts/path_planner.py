import rospy
from sdm_project.msg import Array2D, Array1D
from sdm_project.srv import *
from ARCThetaStar_new import ARCThetaStar
import numpy as np


def handle_get_path(req):
    print('Service successfully called.')
    
    map = np.zeros((len(req.map.dim1), len(req.map.dim2)), int)
    
    x = 0
    for row in req.map.dim1:
        map[x, :] = row
        x += 1
    # Create path planner node
    path_planner = ARCThetaStar(map)
    # Calculate path for given start and goal
    path = path_planner.calculate_path(req.start, req.goal)
    


if __name__ == '__main__':
    ##### DATA #####
    
    
    ##### ROS #####
    # Initialize this node
    rospy.init_node('sdm_path_planner', anonymous=True)
    
    # Subscriber
    # sub_sensor_data = rospy.Subscriber('/robot0/laser_0', LaserScan, cb_sensor_data)
    
    # Publisher
    # pub_pose_est = rospy.Publisher('/rt_local_dnn/robot0/pose', PoseStamped, queue_size=10)
    
    # Register function that is called before shutdown
    # rospy.on_shutdown(sess.close)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
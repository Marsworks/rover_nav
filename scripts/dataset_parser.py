#! /usr/bin/env python

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf import TransformBroadcaster, transformations

from pathlib import Path
import numpy as np

dataset_path = Path("/home/mk_99/Downloads/3D_maps_space_datasets/a100_dome")
tf_pub = TransformBroadcaster()

header = Header()
header.frame_id = "pcl_frame"

fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1)
          ]
          
pcl_pub = rospy.Publisher('/d400/depth/color/points/', PointCloud2, queue_size=10)
rospy.init_node('pcl_publisher', anonymous=True)
rate = rospy.Rate(10)

cntr = 0
for sub_dir in dataset_path.iterdir(): 
    rospy.loginfo(sub_dir)
    
    # File containing the ground truth transform of the robot
    gt_file_path = list(sub_dir.glob("*.gt"))[0]

    gt_file_content = gt_file_path.open()
    
    data = gt_file_content.readline().split()
    x_rot = data
    x = float(data[3])

    data = gt_file_content.readline().split()
    y_rot = data
    y = float(data[3])

    data = gt_file_content.readline().split()
    z_rot = data
    z = float(data[3])

    data = gt_file_content.readline().split()

    rot_matrix = np.array([x_rot, y_rot, z_rot, data], np.double)

    tf_pub.sendTransform((x,y, z),
                         transformations.quaternion_from_matrix(rot_matrix),
                         rospy.Time.now(),
                         "pcl_frame",
                         "t265_odom_frame")

    # File containing x y z point coordinate of the point cloud
    xyz_file_path = list(sub_dir.glob("*.xyz"))[0]
    
    points = []
    with xyz_file_path.open() as file:
        for line in file:
            coordinate = line.split()
            points.append([float(coordinate[0]), float(coordinate[1]), float(coordinate[2])])
            
    pcl_msg = point_cloud2.create_cloud(header, fields, points)
    pcl_pub.publish(pcl_msg) 
    # cntr +=1
    if rospy.is_shutdown() or cntr==10:
        break
    # rate.sleep()
   
    
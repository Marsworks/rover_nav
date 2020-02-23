#!/usr/bin/env python
import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid

map_pub = rospy.Publisher('projected_map_2D', OccupancyGrid, queue_size=10)
map_res = 0.1

map_message = OccupancyGrid()

map_message.header.frame_id = "t265_odom_frame" #"map"
map_message.info.resolution = map_res


def map_publisher_callbask(data):
    '''
    Why am I using data.markers[16] you may ask, I have no idea!
    for some reason that was the only marker getting filled with
    significant amounts of usable data. Need to understand why
    this is the case.
    '''
    map_message.header.stamp = rospy.get_rostime()
    x=[]; y=[]; z=[]

    for marker in data.markers[16].points:
        x.append(marker.x)
        y.append(marker.y)
        z.append(marker.z)

    max_x, min_x = max(x), min(x)
    max_y, min_y = max(y), min(y)
    min_z = min(z)
    mean_z = sum(z) /len(z)
    
    map_message.info.origin.position.x = min_x
    map_message.info.origin.position.y = min_y

    map_message.info.width = abs(int(round((max_x-min_x) / map_res)))
    map_message.info.height = abs(int(round((max_y-min_y) / map_res)))

    # map_message.info.width = 100/map_res
    # map_message.info.height = 100/map_res
    # rospy.loginfo(str(map_message.info.width) + " " + str(map_message.info.height))

    array = np.zeros(shape=(map_message.info.height, map_message.info.width))
    array.fill(0)
    # rospy.loginfo(data.markers[16].points)
    for marker in data.markers[16].points:
        
        x = int(round(((marker.x - min_x) / map_res)))
        x = min(x, map_message.info.width-1)

        y = int(round(((marker.y - min_y) / map_res)))
        y = min(y, map_message.info.height-1)

        fixed_min_thresh = marker.z > (min_z+0.6) # decent results with 0.4
        avg_thresh = abs(marker.z - mean_z) > 0.6

        if avg_thresh:
            array[y][x] = 100
        else:
            array[y][x] = 0

    map_message.data = array.astype(np.int8).reshape(-1)
    map_pub.publish(map_message)
    
    rospy.loginfo("Published map " + str(max_x) +  " " + str(max_y) + " " + str(min_z))

def main():
    rospy.init_node('map_broadcaster', anonymous=True)
    rospy.Subscriber("occupied_cells_vis_array", MarkerArray, map_publisher_callbask)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except:
        pass

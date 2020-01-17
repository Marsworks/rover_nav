#!/usr/bin/env python
import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid, MapMetaData

map_pub = rospy.Publisher('maro_map', OccupancyGrid, queue_size=10)
map_res = 0.05

map_message = OccupancyGrid()

map_message.header.frame_id = "map"
map_message.info.resolution = map_res

def callback(data):
    '''
    Why am I using data.markers[16] you may ask, I have no idea!
    for some readon that was the only marker getting filled with
    significant amounts of usable data. Need to undersatnd why
    this is the case.
    '''
    map_message.header.stamp = rospy.get_rostime()
    x=[]
    y=[]
    z=[]
    for marker in data.markers[16].points:
        x.append(marker.x)
        y.append(marker.y)
        z.append(marker.z)

    max_x, min_x = max(x), min(x)
    max_y, min_y = max(y), min(y)
    min_z = min(z)

    map_message.info.width = int(round((max_x-min_x) / map_res))
    map_message.info.height = int(round((max_y-min_y) / map_res))
    # rospy.loginfo(str(map_message.info.width) + " " + str(map_message.info.height))

    array = np.zeros(shape=(map_message.info.height, map_message.info.width))
    array.fill(0)

    # rospy.loginfo(str(max_x) +  " " + str(max_y) + " " + str(min_z))

    for marker in data.markers[16].points:
        x = int(round(((marker.x - min_x) / map_res)))
        x = min(x, map_message.info.width-1)

        y = int(round(((marker.y - min_y) / map_res)))
        y = min(y, map_message.info.height-1)

        if marker.z > (min_z+0.1): # decent results wiht 0.4
            array[y][x] = 100
        else:
            array[y][x] = 0

    map_message.data = array.astype(np.int8).reshape(-1)
    rospy.loginfo("Published map " + str(max_x) +  " " + str(max_y) + " " + str(min_z))
    map_pub.publish(map_message)

def listener():
    rospy.init_node('map_creator', anonymous=True)

    rospy.Subscriber("occupied_cells_vis_array", MarkerArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

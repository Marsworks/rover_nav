#!/usr/bin/env python
import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid

map_pub = rospy.Publisher('projected_map_2D', OccupancyGrid, queue_size=10)

map_res = 0.1 # Resolution of the incoming ocotomap

# Occupancy message to publish 
# Message format http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
map_message = OccupancyGrid() 

map_message.header.frame_id = "t265_odom_frame"
map_message.info.resolution = map_res


def map_publisher_callback(data):
    '''
    This is function is called whenever a new ocotomap
    is published.
    '''
    
    x=[]; y=[]; z=[] # This reperesent positions on their respective axis (meters) 

    # Creating lists of the x, y & z coordinates of the points  
    for marker in data.markers[16].points:
        x.append(marker.x)
        y.append(marker.y)
        z.append(marker.z)

    max_x, min_x = max(x), min(x) # getting the max and min x values
    max_y, min_y = max(y), min(y) # getting the max and min y values
    min_z = min(z) # Getting the min z height
    mean_z = sum(z) /len(z) # getting mean of all the heights
    
    
    map_message.header.stamp = rospy.get_rostime()
    map_message.info.origin.position.x = min_x
    map_message.info.origin.position.y = min_y

    map_message.info.width = abs(int(round((max_x-min_x) / map_res)))
    map_message.info.height = abs(int(round((max_y-min_y) / map_res)))

    # rospy.loginfo(str(map_message.info.width) + " " + str(map_message.info.height))

    # Creating a 2D array size map_message.info.height x map_message.info.width
    array = np.zeros(shape=(map_message.info.height, map_message.info.width))
    # Filling the array with zero, to initialize the map as fully empty
    array.fill(0)
    
    '''
    Why am I using data.markers[16] you may ask, I have no idea!
    for some reason that was the only marker getting filled with
    significant amounts of usable data. Need to understand why
    this is the case.
    '''
    # Looping through all the points
    for marker in data.markers[16].points:
        
        # Converting the x position (meters) to an index in the array
        x = int(round(((marker.x - min_x) / map_res)))
        # Making sure the index is within the array size  
        x = min(x, map_message.info.width-1)

        # Converting the y position (meters) to an index in the array
        y = int(round(((marker.y - min_y) / map_res)))
        # Making sure the index is within the array size
        y = min(y, map_message.info.height-1)

        fixed_min_thresh = marker.z > (min_z+0.6) # decent results with 0.4
        avg_thresh = abs(marker.z - mean_z) > 0.6
        
        '''
        Add your decision logic below then make sure you change the 
        the if condition below.
        '''

        # Desciding whether a cell should be marked as occupied or free
        if avg_thresh:
            array[y][x] = 100 # occupied
        else:
            array[y][x] = 0 # free

    # Flattening the array and converting it int8
    map_message.data = array.astype(np.int8).reshape(-1)
    
    # Publishing the map (Occupancy grid)
    map_pub.publish(map_message)
    
    rospy.loginfo("Published map " + str(max_x) +  " " + str(max_y) + " " + str(min_z))

def main():
    rospy.init_node('map_broadcaster', anonymous=True)
    rospy.Subscriber("occupied_cells_vis_array", MarkerArray, map_publisher_callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except:
        pass

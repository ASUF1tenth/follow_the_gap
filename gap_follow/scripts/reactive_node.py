import rclpy
from rclpy.node import Node
import os
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math


log_path = "/sim_ws/src/f1tenth_gym_ros/readings.csv"


class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """

    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        self.car_width = 0.28
        self.safety_margin = 0.2 + self.car_width/2
        self.gap_distance = 2
        self.range_offset = 180
        self.radians_per_elem = 0.00436332312998582
        self.PREPROCESS_CONV_SIZE = 3
        self.MAX_LIDAR_DIST = 6
        self.disparity = 1
        self.minimum_distance = 0.2
        self.center_index = 540


        # TODO: Subscribe to LIDAR
        self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        # TODO: Publish to drive
        self.marker_pub = self.create_publisher(MarkerArray, 'safety_bubbles_markers', 10)


    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        

        # # Extract front 180° FOV: combine front-right (270-360°) and front-left (0-90°)
        proc_ranges = np.array(ranges[self.range_offset:-self.range_offset])
        # # Replace inf and nan values with a large number (e.g., 10.0)
        proc_ranges[np.isinf(proc_ranges)] = self.MAX_LIDAR_DIST
        proc_ranges[np.isnan(proc_ranges)] = self.MAX_LIDAR_DIST
        # # Apply moving average filter for smoothing (window size of 5)
        window_size = 5
        kernel = np.ones(window_size) / window_size
        proc_ranges = np.convolve(proc_ranges, kernel, mode='valid')
        # # Cap maximum distance
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        # self.get_logger().info(f"Processed ranges: {proc_ranges}")
        self.center_index = len(proc_ranges) // 2
        # Lidar Right to left, Print Left to right
        return proc_ranges[::-1]

    def find_max_gap(self, free_space_ranges) -> tuple :
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        # threshold for gap distance = self.gap_distance
        # threshold for gap size >= self.safety_margin

        
        
        return None
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """

        return None

    def safety_bubble(self,proc_ranges):
        output_ranges = proc_ranges.copy()
        bubble_coords = []
        for i in range(len(proc_ranges)):
            # Edge case for the first element
            if i == 0:
                last = proc_ranges[i]
                continue

            if (abs(proc_ranges[i] - last)) > self.disparity:
                 # Check if the edge is Left->Right , or Right->Left, it'll matter where we place the bubble.
                if proc_ranges[i] < last:
                    center_of_bubble = i
                    distance_to_point = max(self.minimum_distance,proc_ranges[i])
                else:
                    center_of_bubble = i - 1
                    distance_to_point = max(self.minimum_distance,last)
                # Get bubble radius in terms of number of LiDAR points to zero out
                bubble_coords.append((center_of_bubble, distance_to_point))
                bubble_radius = int(
                        (self.safety_margin / distance_to_point) / self.radians_per_elem
                    )
                # Ensure start_idx does not go below 0
                start_idx = max(0, center_of_bubble - bubble_radius)
                # Ensure end_idx does not exceed the array bounds
                end_idx = min(len(proc_ranges) - 1, center_of_bubble + bubble_radius)
                # Zero out the points within the bubble
                output_ranges[start_idx:end_idx + 1] = 0
                # Pass bubble coords to RVIZ function
            last = proc_ranges[i]
        self.publish_bubbles(bubble_coords)
        return output_ranges

    def publish_bubbles(self, bubble_locations):
        """
        bubble_locations: List of tuples (index, distance) where a bubble was placed
        """
        marker_array = MarkerArray()
        
        # Clear previous markers (prevents "ghost" markers in RViz)
        clear_marker = Marker()
        # clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for i, (idx, dist) in enumerate(bubble_locations):
            marker = Marker()
            marker.header.frame_id = "ego_racecar/laser"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "bubbles"
            marker.id = i
            marker.type = Marker.CYLINDER # Cylinder looks better for 2D safety zones
            marker.action = Marker.ADD
            
            # Math to place the marker in 3D space
            angle = (idx - self.center_index) * self.radians_per_elem
            marker.pose.position.x = dist * math.cos(angle)
            marker.pose.position.y = dist * math.sin(angle)
            marker.pose.position.z = 0.0 # On the ground
            
            # Scale: diameter of the bubble
            marker.scale.x = self.safety_margin * 2.0
            marker.scale.y = self.safety_margin * 2.0
            marker.scale.z = 0.05 # Flat disk
            
            # Color: Semi-transparent Red
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.4 # Transparency
            
            marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        proc_ranges = self.safety_bubble(proc_ranges)

        
        # TODO:
        #Find closest point to LiDAR

        #Eliminate all points inside 'bubble' (set them to zero), Done

        #Find max length gap 

        #Find the best point in the gap 

        #Publish Drive message


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
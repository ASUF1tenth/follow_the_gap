import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

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

        # TODO: Subscribe to LIDAR
        self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        # TODO: Publish to drive


    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        
        # Convert to numpy array for easier processing
        proc_ranges = np.array(ranges)
        # Extract front 180° FOV: combine front-right (270-360°) and front-left (0-90°)
        proc_ranges = np.array(ranges[self.range_offset:-self.range_offset])
        # Replace inf and nan values with a large number (e.g., 10.0)
        proc_ranges[np.isinf(proc_ranges)] = 10.0
        proc_ranges[np.isnan(proc_ranges)] = 10.0
        
        # Cap maximum distance at 3.0 meters
        proc_ranges = np.clip(proc_ranges, 0, 3.0)
        
        # Apply moving average filter for smoothing (window size of 5)
        window_size = 5
        kernel = np.ones(window_size) / window_size
        proc_ranges = np.convolve(proc_ranges, kernel, mode='same')
        
        return proc_ranges

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

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        
        # TODO:
        #Find closest point to LiDAR

        #Eliminate all points inside 'bubble' (set them to zero) 

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
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
        self.disparity = 0.5
        self.minimum_distance = 0.005
        self.center_index = 360
        self.printed = 0
        self.iscornering = 0
        self.steering_Kp = 1.0
        self.last_steering_angle = 0.0

        # TODO: Subscribe to LIDAR
        self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        # TODO: Publish to drive
        self.marker_pub = self.create_publisher(MarkerArray, 'safety_bubbles_markers', 10)
        self.gap_pub = self.create_publisher(MarkerArray, 'gap_marker', 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped,drive_topic,10)


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
        kernel = np.ones(self.PREPROCESS_CONV_SIZE) / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.convolve(proc_ranges, kernel, mode='valid')
        # # Cap maximum distance
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        # self.get_logger().info(f"Processed ranges: {proc_ranges}")
        self.center_index = len(proc_ranges) // 2
        # Lidar Right to left, Print Left to right
        return proc_ranges[::-1]
    
    def cornering_check(self,ranges):
        ranges = np.array(ranges)
        right_side = ranges[:self.range_offset]
        left_side = ranges[-self.range_offset:]
        corner_threshold = 0.45
        if np.any(right_side < corner_threshold) or np.any(left_side < corner_threshold):
            self.iscornering = 1
            self.get_logger().info("Cornering")
        else:
            self.iscornering = 0
        return None

    def find_best_gap(self, ranges) -> tuple :
        """ Return the start index & end index of the best gap in ranges
        """

        best_global_index = None
        best_score = -np.inf

        i = 0
        n = len(ranges)

        while i < n:

            if ranges[i] <= self.gap_distance:
                i += 1
                continue

            start_i = i

            while i < n and ranges[i] > self.gap_distance:
                i += 1

            end_i = i - 1

            gap_length = end_i - start_i + 1

            mid_index = (start_i + end_i) // 2
            mid_distance = ranges[mid_index]

            gap_width = gap_length * mid_distance * self.radians_per_elem

            # ===== Safety Check =====
            if gap_width < self.safety_margin:
                continue



            # ===== Best point inside gap =====
            gap_slice = ranges[start_i:end_i + 1]
            max_dist = np.max(gap_slice)
            best_indices = np.where(gap_slice >= (max_dist - 0.1))[0]
            local_best = start_i + int(np.median(best_indices))
            best_distance = ranges[local_best]

            # ===== Score Function =====
            center_penalty = abs(local_best - self.center_index)
            score = best_distance - 0.01 * center_penalty

            if score > best_score:
                best_score = score
                best_global_index = local_best

        if best_global_index is None:
            best_global_index = self.center_index
        distance_to_index = ranges[best_global_index]
        self.publish_bubble_gap(best_global_index,distance_to_index)
        return best_global_index  

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
                    distance_to_point = max(self.minimum_distance,proc_ranges[i-1])

                # Get bubble radius in terms of number of LiDAR points to zero out
                bubble_radius = int(
                        (2.0 * self.safety_margin / distance_to_point) / self.radians_per_elem
                    )
                # self.get_logger().info(f"{bubble_radius}")

                bubble_coords.append((center_of_bubble, distance_to_point))


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
        # 1. Properly clear old markers
        clear_marker = Marker()
        clear_marker.header.frame_id = self.frame_id # MUST MATCH
        clear_marker.ns = "bubbles"
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        half_width = (self.center_index) * self.radians_per_elem
        for i, (idx, dist) in enumerate(bubble_locations):
            marker = Marker()
            marker.header.frame_id = self.frame_id # MUST MATCH
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "bubbles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Math to place the marker in 3D space
            angle = half_width - (idx * self.radians_per_elem)
            marker.pose.position.x = dist * math.cos(angle)
            marker.pose.position.y = dist * math.sin(angle)
            marker.pose.position.z = 0.0 # On the ground
            
            # Scale: diameter of the bubble
            marker.scale.x = 0.34*2.0
            marker.scale.y = 0.34*2.0
            marker.scale.z = 0.05 # Flat disk
            
            # Color: Semi-transparent Red
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.4 # Transparency
            
            marker.lifetime = rclpy.duration.Duration(seconds=0, nanoseconds=100000000).to_msg()
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def publish_bubble_gap(self, idx , dist):
 
        marker_array = MarkerArray()

        # 1. Properly clear old markers
        clear_marker = Marker()
        clear_marker.header.frame_id = self.frame_id # MUST MATCH
        clear_marker.ns = "bubbles"
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        half_width = (self.center_index) * self.radians_per_elem
        marker = Marker()
        marker.header.frame_id = self.frame_id # MUST MATCH
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "bubbles"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Math to place the marker in 3D space
        angle = half_width - (idx * self.radians_per_elem)
        marker.pose.position.x = dist * math.cos(angle)
        marker.pose.position.y = dist * math.sin(angle)
        marker.pose.position.z = 0.0 # On the ground
        
        # Scale: diameter of the bubble
        marker.scale.x = 0.34
        marker.scale.y = 0.34
        marker.scale.z = 0.05 # Flat disk
        
        # Color: Semi-transparent Red
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.4 # Transparency
        
        marker.lifetime = rclpy.duration.Duration(seconds=0, nanoseconds=100000000).to_msg()
        marker_array.markers.append(marker)

        self.gap_pub.publish(marker_array)

    def get_steering_angle(self,best_idx):
        # If best_idx > center_index, the target is to the RIGHT.
        # In ROS, Right is NEGATIVE.

        # If the car turns too slow, increase Kp to 1.2 , If the car wobbles, decrease to 0.8.
        
        angle_error = (self.center_index - best_idx) * self.radians_per_elem
        
        if self.iscornering == 0:
            steering_angle = angle_error * self.steering_Kp
        else:
            steering_angle = 0

        # Clamp to physical limits (~24 degrees)
        return np.clip(steering_angle, -0.4189, 0.4189)

    def get_velocity(self, steering_angle):
        abs_angle = abs(steering_angle)
        
        v = 154.2982* math.exp(1.3078*abs_angle) - 150.2948 * math.exp(1.3526*abs_angle)

        return v

    def publish_drive(self,steering_angle,velocity):
        drive_msg = AckermannDriveStamped()
        if abs(self.last_steering_angle - steering_angle) > np.radians(10):
            if self.last_steering_angle > steering_angle:
                steering_msg = self.last_steering_angle - 0.1 * steering_angle
            else:
                steering_msg = self.last_steering_angle + 0.1 * steering_angle
        else:
            steering_msg = steering_angle   
        drive_msg.drive.steering_angle = steering_msg
        drive_msg.drive.speed = velocity
        self.last_steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        self.frame_id = data.header.frame_id
        #Process LiDAR data, returned array is from Left to Right of the car
        proc_ranges = self.preprocess_lidar(ranges)

        #Check for cornering (Car not done with corner yet)
        self.cornering_check(ranges)

        #Implement a Safety bubble across disparities
        pro_proc_ranges = self.safety_bubble(proc_ranges)

        #Finding the best possible gap (The one with deepest path available to move through)
        indx = self.find_best_gap(pro_proc_ranges)

        #Get steering angle for the drive message
        steering_angle = self.get_steering_angle(indx)

        #Dynamic velocity(Maybe?)
        target_velocity = self.get_velocity(steering_angle)

        #Publish a drive message for the car
        self.publish_drive(steering_angle,target_velocity)

        # For inspecting the arrays and functions outputs
        if self.printed == 0 :
            self.get_logger().info(f"{ranges}")
            self.get_logger().info(f"{proc_ranges}")
            self.get_logger().info(f"{pro_proc_ranges}")
            self.get_logger().info(f"{indx}")
            self.printed = 1
        # self.get_logger().info("")


        # TODO:
        #Find closest point to LiDAR
        # ?? we're not doing that

        #Eliminate all points inside 'bubble' (set them to zero)
        #Done

        #Find max length gap
        #Done

        #Find the best point in the gap 
        #Done

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
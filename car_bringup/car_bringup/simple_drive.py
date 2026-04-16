import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SimpleDriveNode(Node):
    def __init__(self):
        super().__init__('simple_drive_node')
        
        self.throttle = 0.1  # (-1.0 to 1.0)
        self.steering = 60.0  # (degrees)
        
        # Publishers (QoS 10)
        self.throttle_pub = self.create_publisher(Float32, '/throttle', 10) 
        self.steering_pub = self.create_publisher(Float32, '/steering_command', 10) 
        
        # Timer to publish commands at 20Hz
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Simple Drive Node started. Publishing {self.throttle} throttle and {self.steering} steering.')

    def timer_callback(self):
        # messages
        throttle_msg = Float32()
        steering_msg = Float32()
        
        # values
        throttle_msg.data = self.throttle  
        steering_msg.data = self.steering  
        
        # Publish
        self.throttle_pub.publish(throttle_msg)
        self.steering_pub.publish(steering_msg)

def main(args=None):
    rclpy.init(args=args)
    
    simple_drive_node = SimpleDriveNode()
    
    try:
        rclpy.spin(simple_drive_node)
    except KeyboardInterrupt:
        # added logss
        simple_drive_node.get_logger().info("KBINT received, STOPPING XX")
    finally:
        # Publish 0 throttle to stop the car before shutting down
        stop_msg = Float32()
        stop_msg.data = 0.0
        simple_drive_node.throttle_pub.publish(stop_msg)
        
        # delya time to physically transmit the message
        # befroe destroying the publisher
        time.sleep(0.5)
        
        simple_drive_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

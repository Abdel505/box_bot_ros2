import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

class ObstacleAvoider(Node):

    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # Publisher for movement commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriber for laser scan data
        # Using BEST_EFFORT reliability to match typical sensor publication QoS (especially in simulation)
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

        self.get_logger().info('Obstacle Avoider Node Started: Autonomous Obstacle Avoidance')

    def listener_callback(self, msg):
        # Initialize velocity command
        cmd = Twist()

        # Analyze LaserScan data
        # msg.ranges contains the distance readings.
        # We assume the middle of the array corresponds to the front of the robot.
        
        mid_index = len(msg.ranges) // 2
        
        # Define a window to check for obstacles in front (e.g., center +/- 20 readings)
        window_size = 20
        start_index = max(0, mid_index - window_size)
        end_index = min(len(msg.ranges), mid_index + window_size)
        
        front_ranges = msg.ranges[start_index:end_index]
        
        # Filter out invalid readings (inf usually means no obstacle in range)
        valid_ranges = [r for r in front_ranges if r < msg.range_max and r > msg.range_min]
        
        min_distance = float('inf')
        if valid_ranges:
            min_distance = min(valid_ranges)

        # Decision logic: "if see wall, turn right"
        safe_distance = 0.3  # meters

        if min_distance < safe_distance:
            # Obstacle detected
            self.get_logger().info(f'Obstacle detected at {min_distance:.2f}m! Turning right.')
            cmd.linear.x = 0.0
            cmd.angular.z = -0.5  # Turn right (negative z)
        else:
            # Path clear
            self.get_logger().info(f'Path clear ({min_distance:.2f}m). Moving forward.', throttle_duration_sec=2.0)
            cmd.linear.x = 0.5   # Move forward
            cmd.angular.z = 0.0

        # Publish the command
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

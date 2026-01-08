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
        
        # State Machine Variables
        self.state = 'FORWARD'
        self.state_start_time = 0.0
        self.backup_duration = 2.0  # seconds
        self.turn_direction = -1.0  # Default turn right (-1.0) or left (1.0)

    def listener_callback(self, msg):
        # Initialize velocity command
        cmd = Twist()
        current_time = self.get_clock().now().nanoseconds / 1e9

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

        # Calculate a wider buffer distance to ensure corners are clear before resuming forward motion
        # Using a wider window (e.g., +/- 60 samples) covers the "shoulders" of the robot
        buffer_size = 60
        start_buffer = max(0, mid_index - buffer_size)
        end_buffer = min(len(msg.ranges), mid_index + buffer_size)
        buffer_ranges = msg.ranges[start_buffer:end_buffer]
        valid_buffer_ranges = [r for r in buffer_ranges if r < msg.range_max and r > msg.range_min]
        min_buffer_distance = float('inf')
        if valid_buffer_ranges:
            min_buffer_distance = min(valid_buffer_ranges)

        # Decision logic: "if see wall, turn right"
        safe_distance = 0.5  # meters

        if self.state == 'FORWARD':
            if min_distance < safe_distance:
                # Obstacle detected, switch to BACKING
                self.get_logger().info(f'Obstacle detected at {min_distance:.2f}m! Backing up.')
                self.state = 'BACKING'
                self.state_start_time = current_time
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                # Path clear
                self.get_logger().info(f'Path clear ({min_distance:.2f}m). Moving forward.', throttle_duration_sec=2.0)
                cmd.linear.x = 0.9  # Move forward
                cmd.angular.z = 0.0

        elif self.state == 'BACKING':
            if (current_time - self.state_start_time) < self.backup_duration:
                cmd.linear.x = -0.2  # Move backward
                cmd.angular.z = 0.0
            else:
                # Decide which side is clearer
                # Right side: ranges[0 : mid_index], Left side: ranges[mid_index : end]
                # We treat 'inf' as range_max to favor open spaces
                left_ranges = [r if r < msg.range_max else msg.range_max for r in msg.ranges[mid_index:]]
                right_ranges = [r if r < msg.range_max else msg.range_max for r in msg.ranges[:mid_index]]

                avg_left = sum(left_ranges) / len(left_ranges) if left_ranges else 0.0
                avg_right = sum(right_ranges) / len(right_ranges) if right_ranges else 0.0

                if avg_left > avg_right:
                    self.turn_direction = 1.0  # Turn Left
                    self.get_logger().info(f'Backing done. Left ({avg_left:.1f}m) > Right ({avg_right:.1f}m). Turning Left.')
                else:
                    self.turn_direction = -1.0 # Turn Right
                    self.get_logger().info(f'Backing done. Right ({avg_right:.1f}m) >= Left ({avg_left:.1f}m). Turning Right.')

                self.state = 'TURNING'
                self.state_start_time = current_time

        elif self.state == 'TURNING':
            # Turn until the path is clear (hysteresis added to prevent flickering)
            # Use min_buffer_distance to ensure we don't scrape the corner as we move forward
            if min_buffer_distance > (safe_distance + 0.2):
                self.get_logger().info(f'Path clear ({min_buffer_distance:.2f}m). Stop turning.')
                self.state = 'FORWARD'
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5 * self.turn_direction

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

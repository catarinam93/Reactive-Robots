import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan  # Corrected to LaserScan
from geometry_msgs.msg import Twist

MAX_RANGE = 0.15  # Maximum range for obstacle detection


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        # Publisher for robot movement
        self.__publisher = self.create_publisher(Twist, 'robot1_cmd_vel', 1)

        # Subscription to the LIDAR topic
        self.create_subscription(LaserScan, 'robot1_lidar', self.__lidar_callback, 1)

    def __lidar_callback(self, message):
        """ Processes LIDAR data and decides movement """
        if not message.ranges:  # Ensure data exists
            self.get_logger().warn("Empty LIDAR data received")
            return

        n_sensor = len(message.ranges)
        middle_sensor = n_sensor // 2

        # Get the minimum distance from LIDAR data
        min_distance = min(message.ranges)
        index_min_sensor = 0
        for i in range(len(message.ranges)):
            if message.ranges[i] == min_distance:
                index_min_sensor = i
                break
        
        angular_vel = 0.0
        if index_min_sensor < middle_sensor:
            angular_vel = 4.0
        else:
            angular_vel = -4.0

        # left_most_sensor_distance = message.ranges[0]
        # right_most_sensor_distance = message.ranges[-1]

        command_message = Twist()
        command_message.linear.x = 0.1

        # Turn if an obstacle is detected
        if min_distance < 0.25 and min_distance > 0.155:
            command_message.angular.z = angular_vel
        if min_distance < 0.15 and index_min_sensor > 15 and index_min_sensor < 85:
            command_message.angular.z = -1 * angular_vel
        

        # Publish the movement command
        self.__publisher.publish(command_message)

        # Log for debugging
        # self.get_logger().info(f"Min LIDAR Distance: {min_distance:.3f}m | Angular: {command_message.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    avoider = ObstacleAvoider()
    rclpy.spin(avoider)
    
    # Cleanup
    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
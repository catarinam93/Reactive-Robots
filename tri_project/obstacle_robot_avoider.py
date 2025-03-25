# # import rclpy
# # from rclpy.node import Node
# # from sensor_msgs.msg import Range
# # from geometry_msgs.msg import Twist


# # MAX_RANGE = 0.15


# # class ObstacleAvoider(Node):
# #     def __init__(self):
# #         super().__init__('obstacle_robot_avoider')

# #         self.__publisher = self.create_publisher(Twist, 'robot2_cmd_vel', 1)

# #         self.create_subscription(Range, 'robot2_lidar', self.__lidar, 1)
    
# #     def __lidar(self, message):
# #         self.__lidar_value = message.ranges

# #         command_message = Twist()

# #         command_message.linear.x = 10.0
# #         command_message.angular.z = 2.0
        

# #         # if self.__left_sensor_value < 0.9 * MAX_RANGE or self.__right_sensor_value < 0.9 * MAX_RANGE:
# #         #     command_message.angular.z = -2.0

# #         self.__publisher.publish(command_message)


# # def main(args=None):
# #     rclpy.init(args=args)
# #     avoider = ObstacleAvoider()
# #     rclpy.spin(avoider)
# #     # Destroy the node explicitly
# #     # (optional - otherwise it will be done automatically
# #     # when the garbage collector destroys the node object)
# #     avoider.destroy_node()
# #     rclpy.shutdown()


# # if __name__ == '__main__':
# #     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan  # Corrected to LaserScan
from geometry_msgs.msg import Twist

MAX_RANGE = 0.15  # Maximum range for obstacle detection


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_robot_avoider')

        # Publisher for robot movement
        self.__publisher = self.create_publisher(Twist, 'robot2_cmd_vel', 1)

        # Subscription to the LIDAR topic
        self.create_subscription(LaserScan, 'robot2_lidar', self.__lidar_callback, 1)

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


# import math
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist
# import numpy as np

# class ObstacleAvoider(Node):
#     def __init__(self):
#         super().__init__('obstacle_robot_avoider')

#         # Publisher for robot movement
#         self.publisher_ = self.create_publisher(Twist, 'robot2_cmd_vel', 1)

#         # Subscription to LIDAR topic
#         self.create_subscription(LaserScan, 'robot2_lidar', self.lidar_callback, 1)

#     def lidar_callback(self, message):
#         """ Processes LIDAR data and decides movement """

#         if not message.ranges:
#             self.get_logger().warn("Empty LIDAR data received")
#             return
        
#         linear_vel, angular_vel = self.distance_handler(1, message.ranges)

#         # Publish velocity command
#         command_message = Twist()
#         command_message.linear.x = linear_vel
#         command_message.angular.z = angular_vel
#         self.publisher_.publish(command_message)

#         # Debug log
#         self.get_logger().info(f"Linear Vel: {linear_vel:.3f}, Angular Vel: {angular_vel:.3f}")

#     def distance_handler(self, direction: int, dist_values: [float]) -> (float, float):
#         """ Calculate linear and angular velocities based on LIDAR input """

#         max_speed = 0.1
#         distP = 0.3
#         angleP = 1
#         wallDist = 0.1

#         # Remove invalid values (0.0 means no detection)
#         valid_ranges = [r if r > 0 else float('inf') for r in dist_values]

#         # Find the index of the closest obstacle
#         size = len(valid_ranges)
#         min_index = np.argmin(valid_ranges)

#         angle_increment = 2 * math.pi / (size - 1)
#         angleMin = (size // 2 - min_index) * angle_increment
#         distMin = valid_ranges[min_index]
#         distFront = valid_ranges[size // 2]
#         distSide = valid_ranges[size // 4] if (direction == 1) else valid_ranges[3 * size // 4]
#         distBack = valid_ranges[0]

#         print(f"distMin: {distMin:.3f}, angleMin: {math.degrees(angleMin):.2f}")

#         if math.isfinite(distMin):
#             if distFront < 1.25 * wallDist and (distSide < 1.25 * wallDist or distBack < 1.25 * wallDist):
#                 angular_vel = direction * -1
#             else:
#                 angular_vel = (direction * distP * (distMin - wallDist) +
#                                angleP * (angleMin - direction * math.pi / 2))
            
#             if distFront < wallDist:
#                 linear_vel = 0  # Stop and turn
#             elif distFront < 2 * wallDist or distMin < wallDist * 0.75 or distMin > wallDist * 1.25:
#                 linear_vel = 0.5 * max_speed  # Slow down
#             else:
#                 linear_vel = max_speed  # Move at full speed
#         else:
#             angular_vel = np.random.normal(0.0, 1.0)
#             linear_vel = max_speed

#         return linear_vel, angular_vel

# def main(args=None):
#     rclpy.init(args=args)
#     avoider = ObstacleAvoider()
#     rclpy.spin(avoider)
    
#     # Cleanup
#     avoider.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

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
# ---------------------------------------------------------------------------------------------------------------
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan  # Corrected to LaserScan
# from geometry_msgs.msg import Twist

# MAX_RANGE = 0.15  # Maximum range for obstacle detection


# class ObstacleAvoider(Node):
#     def __init__(self):
#         super().__init__('obstacle_robot_avoider')

#         # Publisher for robot movement
#         self.__publisher = self.create_publisher(Twist, 'robot2_cmd_vel', 1)

#         # Subscription to the LIDAR topic
#         self.create_subscription(LaserScan, 'robot2_lidar', self.__lidar_callback, 1)

#     def __lidar_callback(self, message):
#         """ Processes LIDAR data and decides movement """
#         if not message.ranges:  # Ensure data exists
#             self.get_logger().warn("Empty LIDAR data received")
#             return

#         n_sensor = len(message.ranges)
#         middle_sensor = n_sensor // 2

#         # Get the minimum distance from LIDAR data
#         min_distance = min(message.ranges)
#         index_min_sensor = 0
#         for i in range(len(message.ranges)):
#             if message.ranges[i] == min_distance:
#                 index_min_sensor = i
#                 break
        
#         angular_vel = 0.0
#         if index_min_sensor < middle_sensor:
#             angular_vel = 4.0
#         else:
#             angular_vel = -4.0

#         # left_most_sensor_distance = message.ranges[0]
#         # right_most_sensor_distance = message.ranges[-1]

#         command_message = Twist()
#         command_message.linear.x = 0.1

#         # Turn if an obstacle is detected
#         if min_distance < 0.25 and min_distance > 0.155:
#             command_message.angular.z = angular_vel
#         if min_distance < 0.15 and index_min_sensor > 15 and index_min_sensor < 85:
#             command_message.angular.z = -1 * angular_vel
        

#         # Publish the movement command
#         self.__publisher.publish(command_message)

#         # Log for debugging
#         # self.get_logger().info(f"Min LIDAR Distance: {min_distance:.3f}m | Angular: {command_message.angular.z}")

# def main(args=None):
#     rclpy.init(args=args)
#     avoider = ObstacleAvoider()
#     rclpy.spin(avoider)
    
#     # Cleanup
#     avoider.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

# ---------------------------------------------------------------------------------------------------------------
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
#---------------------------------------------------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math

MAX_RANGE = 0.15  # Maximum range for obstacle detection
FOLLOW_DISTANCE = 0.25  # Ideal distance to follow the first robot
ROBOT_RADIUS = 0.035  # 3.5 cm radius (since diameter is 7 cm)
FOLLOW_ANGLE_THRESHOLD = 5  # Degrees range for following in front

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_robot_avoider')

        # Publisher for movement commands
        self.publisher = self.create_publisher(Twist, 'robot2_cmd_vel', 1)

        # Subscribe to LIDAR topic
        self.create_subscription(LaserScan, 'robot2_lidar', self.lidar_callback, 1)

        self.robot_found = False
        self.distance_found = 0.0

    def lidar_callback(self, message):
        """Processes LIDAR data to detect and follow the first robot."""
        if not message.ranges:  # Ensure data exists
            self.get_logger().warn("Empty LIDAR data received")
            return

        n_sensors = len(message.ranges)
        angles = np.linspace(message.angle_min, message.angle_max, n_sensors)

        # Convert LIDAR data to (x, y) points
        ranges_np = np.array(message.ranges)
        x_points = ranges_np * np.cos(angles)
        y_points = ranges_np * np.sin(angles)

        # Filter valid points (distance between 5 cm and 40 cm)
        valid_mask = (ranges_np > 0.05) & (ranges_np < 0.4)
        x_points = x_points[valid_mask]
        y_points = y_points[valid_mask]
        self.get_logger().info(f"Self Robot Found: {self.robot_found}")
                               
        if self.robot_found:
            self.get_logger().info("Entrou no if do robot found")
            best_trio_points = []
            min_circle_error = math.inf
            xc_min, yc_min = None, None

            for i in range(len(x_points) - 4):
                p1 = (x_points[i], y_points[i])
                p2 = (x_points[i + 2], y_points[i + 2])
                p3 = (x_points[i + 4], y_points[i + 4])

                _, xc, yc, r = self.are_points_on_circle(p1, p2, p3, radius=ROBOT_RADIUS)

                # self.get_logger().info(f"xc: {xc}, yc: {yc}, r: {r}")
                if r is not None:
                    self.get_logger().info("entrou no if do r")
                    r_error = abs(r - ROBOT_RADIUS)
                    
                    if r_error < min_circle_error:
                        self.get_logger().info("entrou no if do r_error")
                        min_circle_error = r_error
                        best_trio_points = [p1, p2, p3]
                        xc_min, yc_min = xc, yc
            
            if xc_min is not None:
                self.get_logger().info("entrou no if do xc_min")
                distance = np.sqrt(xc_min ** 2 + yc_min ** 2)

                if abs(distance - self.distance_found) < 0.05:
                    self.get_logger().info("entrou no if do distance")
                    self.distance_found = distance
                    self.follow_robot(xc_min, yc_min)
                    # return
                # else:
                #     self.robot_found = False
            else:
                self.robot_found = False
                

        # Only proceed if we have at least 3 points
        if len(x_points) >= 3:
            # self.get_logger().info(f"Detected {len(x_points)} valid points")

            # Check if any 3 points belong to a circle of radius 3.5 cm
            for i in range(len(x_points) - 4):
                p1 = (x_points[i], y_points[i])
                p2 = (x_points[i + 2], y_points[i + 2])
                p3 = (x_points[i + 4], y_points[i + 4])

                flag, xc, yc, r = self.are_points_on_circle(p1, p2, p3, radius=ROBOT_RADIUS)
                if flag:
                    self.get_logger().info(f"Robot detected")
                    # xc, yc, r = self.fit_circle([p1[0], p2[0], p3[0]], [p1[1], p2[1], p3[1]])
                    # self.get_logger().info(f"Robot detected at ({xc:.2f}, {yc:.2f}) | Radius: {r:.2f}")

                    self.robot_found = True
                    self.distance_found = np.sqrt(xc ** 2 + yc ** 2)
                    self.follow_robot(xc, yc)
                    return  # Stop further processing once the robot is detected

        # Default behavior: avoid obstacles
        middle_sensor = n_sensors // 2
        self.avoid_walls(message.ranges, middle_sensor)

    def are_points_on_circle(self, p1, p2, p3, radius=0.035, tol=0.1):
        """
        Checks if three points belong to a circle with a given radius.

        Parameters:
            p1, p2, p3: Tuples (x, y) representing the points
            radius: Expected radius of the circle (default 3.5 cm)
            tol: Tolerance for floating-point precision

        Returns:
            True if the points belong to the same circle with the given radius, False otherwise
        """
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3

        # Construct matrices
        A = np.array([
            [x1, y1, 1],
            [x2, y2, 1],
            [x3, y3, 1]
        ])
        
        B = np.array([
            [x1**2 + y1**2, y1, 1],
            [x2**2 + y2**2, y2, 1],
            [x3**2 + y3**2, y3, 1]
        ])

        C = np.array([
            [x1**2 + y1**2, x1, 1],
            [x2**2 + y2**2, x2, 1],
            [x3**2 + y3**2, x3, 1]
        ])

        D = np.array([
            [x1**2 + y1**2, x1, y1],
            [x2**2 + y2**2, x2, y2],
            [x3**2 + y3**2, x3, y3]
        ])

        # Compute determinants
        detA = np.linalg.det(A)
        detB = np.linalg.det(B)
        detC = np.linalg.det(C)
        detD = np.linalg.det(D)
        # self.get_logger().info(f"DetA: {abs(detA):.10f}")
        if abs(detA) < 0.001:
            return False, None, None, None  # Points are collinear, no valid circle

        # Compute center and radius
        xc = 0.5 * (detB / detA)
        yc = -0.5 * (detC / detA)
        calculated_radius = np.sqrt(xc**2 + yc**2 + (detD / detA))
        # self.get_logger().info(f"Center: ({xc:.2f}, {yc:.2f}) | Radius: {calculated_radius:.2f}")
        # self.get_logger().info(f"Diff: {abs(calculated_radius - radius):.3f}")
        return abs(calculated_radius - radius) < tol, xc, yc, calculated_radius

    def follow_robot(self, xc, yc):
        """ Adjust movement to follow detected robot """
        command = Twist()
        self.get_logger().info(f"Following robot at ({xc:.2f}, {yc:.2f})")
        # Calculate distance to the detected robot
        distance = np.sqrt(xc ** 2 + yc ** 2)

        if distance > FOLLOW_DISTANCE:
            command.linear.x = 0.1  # Move forward
        else:
            command.linear.x = 0.0  # Stop if close enough

        # # Adjust direction based on robot position
        angle_to_robot = np.arctan2(yc, xc) * 180 / np.pi  # Convert to degrees
        # if abs(angle_to_robot) > FOLLOW_ANGLE_THRESHOLD:
        #     self.get_logger().info(f"Angle to robot: {angle_to_robot:.2f}")
        #     command.angular.z = -0.5 if angle_to_robot > 0 else 0.5  # Turn to face robot
        K_p = 0.1  # Proportional gain factor
        max_turn_speed = 2.0 # Limit max turning speed

        if abs(angle_to_robot) > FOLLOW_ANGLE_THRESHOLD:
            self.get_logger().info(f"Angle to robot: {angle_to_robot:.2f}")
            command.angular.z = np.clip(K_p * angle_to_robot, -max_turn_speed, max_turn_speed)


        self.publisher.publish(command)
        self.get_logger().info(f"Following robot at ({xc:.2f}, {yc:.2f}) | Distance: {distance:.2f}")

    def avoid_walls(self, ranges, middle_sensor):
        """ Default wall avoidance logic """
        min_distance = min(ranges)
        index_min_sensor = np.argmin(ranges)

        angular_vel = 4.0 if index_min_sensor < middle_sensor else -4.0
        command = Twist()
        command.linear.x = 0.1  # Default forward speed

        if min_distance < 0.20 and min_distance > 0.155:
            command.angular.z = angular_vel  # Turn to avoid obstacle
        if min_distance < 0.15 and 45 < index_min_sensor < 255:
            command.angular.z = -angular_vel  # Turn away if very close

        self.publisher.publish(command)

def main(args=None):
    rclpy.init(args=args)
    follower = ObstacleAvoider()
    rclpy.spin(follower)
    
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

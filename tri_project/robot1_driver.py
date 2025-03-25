# import rclpy
# from ament_index_python.packages import get_package_share_directory
# from geometry_msgs.msg import Twist, Point, TransformStamped
# from tf2_ros import StaticTransformBroadcaster
# import os
# from visualization_msgs.msg import Marker

# HALF_DISTANCE_BETWEEN_WHEELS = 0.045
# WHEEL_RADIUS = 0.025

# class Robot1Driver:
#     def init(self, webots_node, properties):
#         self.__robot = webots_node.robot
#         self.__left_motor = self.__robot.getDevice('left wheel motor')
#         self.__right_motor = self.__robot.getDevice('right wheel motor')
#         self.__left_motor.setPosition(float('inf'))
#         self.__right_motor.setPosition(float('inf'))

#         # ROS setup
#         rclpy.init(args=None)
#         self.__node = rclpy.create_node('robot1_driver')
#         self.__clock = self.__node.get_clock()

#         # Subscribe to velocity command topic and publish position
#         self.__node.create_subscription(Twist, 'robot1_cmd_vel', self.__cmd_vel_callback, 1)

#         # Initialize target twist values
#         self.__target_twist = Twist()

#         # Setup sound 
#         package_dir = get_package_share_directory('tri_project')
#         sound_path = os.path.join(package_dir, 'music', 'sound_mono_16bit.wav')
#         self.__speaker = self.__robot.getDevice('speaker')
#         self.__speaker.playSound(self.__speaker, self.__speaker, sound_path, 1.0, 1.0, 0, False)

#         # GPS setup for position tracking
#         self.__gps = self.__robot.getDevice('gps')
#         self.__node.get_logger().info(f"GPS enabled: {self.__gps}")
#         self.__gps.enable(10)  # Enable GPS with a 10ms update interval

#         # Path marker setup
#         self.path_marker = Marker()
#         self.path_marker.header.frame_id = 'world'
#         self.path_marker.header.stamp = self.__clock.now().to_msg()  # Use node's clock to get timestamp
#         self.path_marker.ns = 'path'
#         self.path_marker.id = 0
#         self.path_marker.type = Marker.LINE_STRIP
#         self.path_marker.action = Marker.ADD
#         self.path_marker.scale.x = 0.01  # Set line width
#         self.path_marker.color.b = 1.0  # Red color
#         self.path_marker.color.a = 1.0  # Full opacity
#         self.path_marker.points = []  # List to store points

#         # Path marker publisher
#         self.path_pub = self.__node.create_publisher(Marker, 'robot1_path', 10)

#     def __cmd_vel_callback(self, twist):
#         """ Callback function to update target twist values """
#         self.__target_twist = twist

#     def step(self):
#         """ Called at each Webots step to update motor speeds """
#         rclpy.spin_once(self.__node, timeout_sec=0)  # Process any incoming messages

#         AXLE_LENGTH: float = 0.057  # Calibrated axle length
#         WHEEL_RADIUS: float = 0.0205

#         linear_vel = self.__target_twist.linear.x
#         angular_vel = self.__target_twist.angular.z

#         # Compute wheel velocities
#         r_omega = (linear_vel + angular_vel * AXLE_LENGTH / 2) / WHEEL_RADIUS
#         l_omega = (linear_vel - angular_vel * AXLE_LENGTH / 2) / WHEEL_RADIUS

#         # Apply velocities to motors
#         self.__left_motor.setVelocity(l_omega)
#         self.__right_motor.setVelocity(r_omega)

#         # Publish robot position using GPS
#         position = self.__gps.getValues()

#         # Store position
#         position_msg = Point()
#         position_msg.x = position[0]
#         position_msg.y = position[1]
#         position_msg.z = position[2]

#         # Add new position to path marker
#         self.path_marker.points.append(position_msg)

#         # Update timestamp of the path marker to be current time
#         self.path_marker.header.stamp = self.__clock.now().to_msg()

#         # Publish the updated path marker
#         self.path_pub.publish(self.path_marker)

# -------------------------------------------------------

# import rclpy
# from ament_index_python.packages import get_package_share_directory
# from geometry_msgs.msg import Twist, Point
# from visualization_msgs.msg import Marker
# import os
# import csv
# import time

# HALF_DISTANCE_BETWEEN_WHEELS = 0.045
# WHEEL_RADIUS = 0.025

# class Robot1Driver:
#     def init(self, webots_node, properties):
#         """Initialize class variables."""
#         self.__robot = None
#         self.__node = None
#         self.__left_motor = None
#         self.__right_motor = None
#         self.__gps = None
#         self.__speaker = None
#         self.__target_twist = Twist()
#         self.csv_file = None
#         self.csv_writer = None

#         """Initialize the robot's motors, sensors, ROS node, and logging."""
#         self.__robot = webots_node.robot
#         self.__left_motor = self.__robot.getDevice('left wheel motor')
#         self.__right_motor = self.__robot.getDevice('right wheel motor')
#         self.__right_motor.setPosition(float('inf'))
#         self.__left_motor.setPosition(float('inf'))

#         # ROS setup
#         rclpy.init(args=None)
#         self.__node = rclpy.create_node('robot1_driver')
#         self.__clock = self.__node.get_clock()

#         # Subscribe to velocity command topic
#         self.__node.create_subscription(Twist, 'robot1_cmd_vel', self.__cmd_vel_callback, 1)

#         # GPS setup
#         self.__gps = self.__robot.getDevice('gps')
#         self.__gps.enable(10)

#         # Path marker setup
#         self.path_marker = Marker()
#         self.path_marker.header.frame_id = 'world'
#         self.path_marker.header.stamp = self.__clock.now().to_msg()
#         self.path_marker.ns = 'path'
#         self.path_marker.id = 0
#         self.path_marker.type = Marker.LINE_STRIP
#         self.path_marker.action = Marker.ADD
#         self.path_marker.scale.x = 0.01
#         self.path_marker.color.b = 1.0
#         self.path_marker.color.a = 1.0
#         self.path_marker.points = []

#         # Path marker publisher
#         self.path_pub = self.__node.create_publisher(Marker, 'robot1_path', 10)

#         # Open CSV file for logging
#         log_path = '/path/to/log_data.csv'  # Update path
#         self.csv_file = open(log_path, 'w', newline='')
#         self.csv_writer = csv.writer(self.csv_file)
#         self.csv_writer.writerow(['Timestamp', 'Linear Velocity', 'Angular Velocity', 'Right Wheel Vel', 'Left Wheel Vel'])

#         # Register shutdown handler
#         self.__node.destroy_node = self.shutdown_callback

#     def __cmd_vel_callback(self, twist):
#         """Callback function to update target twist values."""
#         self.__target_twist = twist

#     def step(self):
#         """Called at each Webots step to update motor speeds and log data."""
#         rclpy.spin_once(self.__node, timeout_sec=0)

#         AXLE_LENGTH = 0.057
#         WHEEL_RADIUS = 0.0205

#         linear_vel = self.__target_twist.linear.x
#         angular_vel = self.__target_twist.angular.z

#         # Compute wheel velocities
#         r_omega = (linear_vel + angular_vel * AXLE_LENGTH / 2) / WHEEL_RADIUS
#         l_omega = (linear_vel - angular_vel * AXLE_LENGTH / 2) / WHEEL_RADIUS

#         # Apply velocities to motors
#         self.__left_motor.setVelocity(l_omega)
#         self.__right_motor.setVelocity(r_omega)

#         # # Get timestamp
#         current_time = time.time()

#         # Write data to CSV (Ensure file is open)
#         if self.csv_file is not None and not self.csv_file.closed:
#             # timestamp = self.path_marker.header.stamp.sec + self.path_marker.header.stamp.nanosec * 1e-9
#             # self.csv_writer.writerow([timestamp, linear_vel, angular_vel, r_omega, l_omega])

#             self.csv_writer.writerow([current_time, linear_vel, angular_vel, r_omega, l_omega])
#         else:
#             self.__node.get_logger().warn("CSV file was unexpectedly closed.")

#         # Publish robot position using GPS
#         position = self.__gps.getValues()
#         position_msg = Point()
#         position_msg.x = position[0]
#         position_msg.y = position[1]
#         position_msg.z = position[2]

#         # Add new position to path marker
#         self.path_marker.points.append(position_msg)

#         # Update timestamp of the path marker
#         self.path_marker.header.stamp = self.__clock.now().to_msg()

#         # Publish the updated path marker
#         self.path_pub.publish(self.path_marker)

#     def shutdown_callback(self):
#         """Close CSV file when shutting down the node."""
#         if self.csv_file and not self.csv_file.closed:
#             self.csv_file.close()
#             self.__node.get_logger().info("CSV file closed safely.")

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
import os
import csv
import time

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class Robot1Driver:
    def init(self, webots_node, properties):
        """Initialize class variables."""
        self.__robot = None
        self.__node = None
        self.__left_motor = None
        self.__right_motor = None
        self.__gps = None
        self.__speaker = None
        self.__target_twist = Twist()
        self.csv_file = None
        self.csv_writer = None
        self.csv_file_path = os.path.join(os.path.expanduser("~"), "ros2_ws/src/tri_project/tri_project/robot1_log_data.csv")

    
        """Initialize the robot's motors, sensors, ROS node, and logging."""
        self.__robot = webots_node.robot
        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')
        self.__left_motor.setPosition(float('inf'))
        self.__right_motor.setPosition(float('inf'))

        # ROS setup
        rclpy.init(args=None)
        self.__node = rclpy.create_node('robot1_driver')
        self.__clock = self.__node.get_clock()

        # Subscribe to velocity command topic
        self.__node.create_subscription(Twist, 'robot1_cmd_vel', self.__cmd_vel_callback, 1)

        # GPS setup
        self.__gps = self.__robot.getDevice('gps')
        self.__gps.enable(10)

        # Path marker setup
        self.path_marker = Marker()
        self.path_marker.header.frame_id = 'world'
        self.path_marker.header.stamp = self.__clock.now().to_msg()
        self.path_marker.ns = 'path'
        self.path_marker.id = 0
        self.path_marker.type = Marker.LINE_STRIP
        self.path_marker.action = Marker.ADD
        self.path_marker.scale.x = 0.01
        self.path_marker.color.b = 1.0
        self.path_marker.color.a = 1.0
        self.path_marker.points = []

        # Path marker publisher
        self.path_pub = self.__node.create_publisher(Marker, 'robot1_path', 10)

        # Open CSV file in append mode
        self.open_csv_file()

        # Register shutdown handler
        self.__node.destroy_node = self.shutdown_callback

    def open_csv_file(self):
        """Open the CSV file for writing data."""
        try:
            self.csv_file = open(self.csv_file_path, 'a', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            if self.csv_file.tell() == 0:  # If file is empty, write headers
                self.csv_writer.writerow(['Timestamp', 'Linear Velocity', 'Angular Velocity', 'Right Wheel Vel', 'Left Wheel Vel'])
            self.__node.get_logger().info(f"CSV file opened at {self.csv_file_path}")
        except Exception as e:
            self.__node.get_logger().error(f"Failed to open CSV file: {e}")

    def __cmd_vel_callback(self, twist):
        """Callback function to update target twist values."""
        self.__target_twist = twist

    def step(self):
        """Called at each Webots step to update motor speeds and log data."""
        rclpy.spin_once(self.__node, timeout_sec=0)

        AXLE_LENGTH = 0.057
        WHEEL_RADIUS = 0.0205

        linear_vel = self.__target_twist.linear.x
        angular_vel = self.__target_twist.angular.z

        # Compute wheel velocities
        r_omega = (linear_vel + angular_vel * AXLE_LENGTH / 2) / WHEEL_RADIUS
        l_omega = (linear_vel - angular_vel * AXLE_LENGTH / 2) / WHEEL_RADIUS

        # Apply velocities to motors
        self.__left_motor.setVelocity(l_omega)
        self.__right_motor.setVelocity(r_omega)

        # Throttle data logging by only writing every N seconds
        LOG_INTERVAL = 0.0 
        
        current_time = time.time()

        if hasattr(self, 'last_log_time') and (current_time - self.last_log_time) < LOG_INTERVAL:
            pass
        else:
            # Write data to CSV
            try:
                self.csv_writer.writerow([current_time, linear_vel, angular_vel, r_omega, l_omega])
                self.csv_file.flush()  # Ensure data is written to disk
            except Exception as e:
                self.__node.get_logger().error(f"Error writing to CSV file: {e}")

        self.last_log_time = current_time  # Update last log time

        # Publish robot position using GPS
        position = self.__gps.getValues()
        position_msg = Point()
        position_msg.x = position[0]
        position_msg.y = position[1]
        position_msg.z = position[2]

        # Add new position to path marker
        self.path_marker.points.append(position_msg)

        # Update timestamp of the path marker
        self.path_marker.header.stamp = self.__clock.now().to_msg()

        # Publish the updated path marker
        self.path_pub.publish(self.path_marker)


    def shutdown_callback(self):
        """Close CSV file when shutting down the node."""
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.close()
            self.__node.get_logger().info("CSV file closed safely.")

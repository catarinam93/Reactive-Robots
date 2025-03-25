import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from controller import Supervisor

class RobotPathVisualizer(Node):
    def __init__(self):
        super().__init__('robot_path_visualizer')

        # Create Supervisor instance
        # self.supervisor = Supervisor()
        # self.robot = self.supervisor.getSelf()  # Get the robot instance
        # self.timestep = int(self.robot.getBasicTimeStep())  # Set the timestep for simulation

        # Create Supervisor instance
        self.supervisor = Supervisor()

        # Specify the correct robot
        self.robot = self.supervisor.getFromDef('robot2')  # Ensure 'robot2' exists in the .wbt file

        if self.robot is None:
            self.get_logger().error("Failed to get 'robot2' from Webots. Check your world file!")
            return  # Prevent further execution

        # Create a new Shape node to represent the path
        self.shape = self.create_shape()
        self.__node.create_subscription(Point, 'robot2_position', self.position_callback, 1)

    def create_shape(self):
        """Create a Shape node with IndexedLineSet to visualize the robot's path"""
        root_node = self.supervisor.getRoot()

        # Create the Shape node dynamically
        shape = self.supervisor.nodeCreate('Shape')
        shape.setAppearance(self.supervisor.nodeCreate('Appearance'))
        shape.getAppearance().setMaterial(self.supervisor.nodeCreate('Material'))
        shape.getAppearance().getMaterial().setDiffuseColor(1.0, 0.0, 0.0)  # Red color for the path

        # Create the IndexedLineSet node
        indexed_line_set = self.supervisor.nodeCreate('IndexedLineSet')
        indexed_line_set.setCoord(self.supervisor.nodeCreate('Coordinate'))

        # Set initial coordinates to an empty list
        indexed_line_set.getCoord().setPoint([])

        # Attach the IndexedLineSet to the Shape node
        shape.addChild(indexed_line_set)

        # Attach the Shape node to the root node
        root_node.addChild(shape)

        return indexed_line_set

    def position_callback(self, position_msg: Point):
        """Callback to update the path with the robot's current position"""
        position = [position_msg.x, position_msg.y, position_msg.z]
        self.get_logger().info(f"Received position: {position}")

        # Add the current position to the path (coordinates)
        coords = self.shape.getCoord().getPoint()
        coords.append([position[0], position[1], position[2]])  # Add the new point to the list
        self.shape.getCoord().setPoint(coords)  # Update the points of the path

    # def run(self):
    #     """Main loop to run the supervisor and update the path"""
    #     while self.supervisor.step(self.timestep) != -1:
    #         rclpy.spin_once(self)  # Process ROS 2 callbacks

def main(args=None):
    rclpy.init(args=args)
    visualizer = RobotPathVisualizer()
    rclpy.spin(visualizer)
    
    # Cleanup
    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
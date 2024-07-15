import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib.pyplot as plt
import numpy as np

class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')

        self.map_data = None
        self.path = []
        self.trajectory = []
        self.current_pose = None

        qos_profile = rclpy.qos.QoSProfile(depth=1)
        qos_profile.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        qos_profile.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE

        qos_profile2 = rclpy.qos.QoSProfile(depth=5)
        qos_profile2.durability = rclpy.qos.DurabilityPolicy.VOLATILE
        qos_profile2.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile)
        self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        self.timer = self.create_timer(1.0, self.plot_data)
        self.fig, self.ax = plt.subplots(figsize=(10, 10))

    def map_callback(self, msg):
        self.get_logger().info('Map received')
        width = msg.info.width
        height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.map_data = np.array(msg.data).reshape((height, width))

    def path_callback(self, msg):
        self.get_logger().info('Path received')
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def pose_callback(self, msg):
        print("pose girdi")
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        self.trajectory.append((x, y))
        self.current_pose = (x, y)  # Update current_pose with the latest position

    def plot_data(self):
        if self.map_data is None:
            return
        
        self.ax.clear()  # Clear previous plot

        # Plot the map
        self.ax.imshow(self.map_data, cmap='gray', origin='lower',
                   extent=(self.origin_x, self.origin_x + self.map_data.shape[1] * self.resolution,
                           self.origin_y, self.origin_y + self.map_data.shape[0] * self.resolution))

        # Plot the path
        if self.path:
            path_x, path_y = zip(*self.path)
            self.ax.plot(path_x, path_y, 'go:', label='Planned Path', markersize=25)

        # Plot the trajectory
        if self.trajectory:
            traj_x, traj_y = zip(*self.trajectory)
            self.ax.plot(traj_x, traj_y, 'ro-', label='Robot Trajectory', markersize=10)


        if self.current_pose:
            zoom_level = 3  # Define the zoom level in meters
            x_center, y_center = self.current_pose
            self.ax.set_xlim(x_center - zoom_level, x_center + zoom_level)
            self.ax.set_ylim(y_center - zoom_level, y_center + zoom_level)

        self.ax.legend()
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Path and Trajectory')
        self.ax.grid(True)
        
        plt.draw()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    visualization_node = VisualizationNode()
    rclpy.spin(visualization_node)
    visualization_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
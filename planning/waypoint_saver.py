import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

import csv

class WaypointSaver(Node):

    def __init__(self):
        super().__init__('waypoint_saver')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        self.waypoints = []
        self.csv_file = 'waypoints.csv'
        self.initialize_csv()

    def initialize_csv(self):
        # Create the CSV file and write the header if it does not exist
        with open(self.csv_file, 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['x', 'y', 'z', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w'])

    def pose_callback(self, msg):
        pose = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        self.waypoints.append(pose)
        self.get_logger().info('Waypoint saved: %s' % pose)
        self.save_waypoints_to_csv()

    def save_waypoints_to_csv(self):
        with open(self.csv_file, 'a', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(self.waypoints[-1])
            self.get_logger().info('Waypoint saved to %s' % self.csv_file)

def main(args=None):
    rclpy.init(args=args)
    waypoint_saver = WaypointSaver()
    rclpy.spin(waypoint_saver)
    waypoint_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

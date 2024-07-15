import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
import math
import tf_transformations

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('goal_tolerance', 0.1)  # Define goal tolerance

        qos_profile = rclpy.qos.QoSProfile(depth=1)
        qos_profile.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        qos_profile.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE

        qos_profile2 = rclpy.qos.QoSProfile(depth=5)
        qos_profile2.durability = rclpy.qos.DurabilityPolicy.VOLATILE
        qos_profile2.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE

        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value

        self.path_subscriber = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, qos_profile)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(PoseStamped, '/goal_pose', self.goalPoseCallback, qos_profile2)

        self.path = []
        self.current_pose = None

    def path_callback(self, msg):
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.get_logger().info(f'Path received with {len(self.path)} points')
        if self.current_pose is not None:
            self.pure_pursuit_control()

    def goalPoseCallback(self, msg):
        self.get_logger().info("Goal received")
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

    def pose_callback(self, msg):
        pose = msg.pose.pose
        self.current_pose = (pose.position.x, pose.position.y, self.get_yaw_from_quaternion(pose.orientation))
        self.get_logger().info(f'Received pose: {self.current_pose}')
        if self.path:
            self.pure_pursuit_control()

    def get_yaw_from_quaternion(self, orientation):
        _, _, yaw = tf_transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return yaw

    def pure_pursuit_control(self):
        self.get_logger().info("Entered pure_pursuit_control")
        if not self.path or not self.current_pose:
            self.get_logger().info("Path or current pose not available.")
            return

        x, y, yaw = self.current_pose
        self.get_logger().info(f'Current Pose: x={x}, y={y}, yaw={yaw}')
        goal_x, goal_y = self.find_lookahead_point(x, y)

        if goal_x is None or goal_y is None:
            self.get_logger().info("No valid lookahead point found.")
            cmd_msg = Twist()
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self.cmd_publisher.publish(cmd_msg)
            return

        alpha = math.atan2(goal_y - y, goal_x - x) - yaw
        Ld = math.sqrt((goal_x - x)**2 + (goal_y - y)**2)

        self.get_logger().info(f'alpha={alpha}, Ld={Ld}')

        # Check if the robot is within goal tolerance
        if self.is_goal_reached(x, y):
            self.get_logger().info("Reached goal within tolerance, stopping.")
            cmd_msg = Twist()
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self.cmd_publisher.publish(cmd_msg)
            return

        cmd_msg = Twist()
        cmd_msg.linear.x = 0.05  # Adjust this value based on your robot's capability
        cmd_msg.angular.z = -1 * cmd_msg.linear.x * math.sin(alpha) / Ld

        self.get_logger().info(f'Publishing cmd_vel: linear.x={cmd_msg.linear.x}, angular.z={cmd_msg.angular.z}')
        self.cmd_publisher.publish(cmd_msg)

    def find_lookahead_point(self, x, y):
        for point in self.path:
            dist = math.sqrt((point[0] - x)**2 + (point[1] - y)**2)
            if dist >= self.lookahead_distance:
                self.get_logger().info(f'Lookahead point found: x={point[0]}, y={point[1]}')
                return point
        self.get_logger().info("No lookahead point found within lookahead distance.")
        return None, None

    def is_goal_reached(self, x, y):
        dist_to_goal = math.sqrt((self.goal_x - x)**2 + (self.goal_y - y)**2)
        self.get_logger().info(f'Distance to goal: {dist_to_goal}')
        return dist_to_goal <= self.goal_tolerance

def main(args=None):
    rclpy.init(args=args)
    pure_pursuit = PurePursuit()
    rclpy.spin(pure_pursuit)
    pure_pursuit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

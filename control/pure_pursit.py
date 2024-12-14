import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import numpy as np

class MPPIController(Node):
    def __init__(self):
        super().__init__('mppi_controller')
        self.get_logger().info('Initializing MPPI Controller Node')
        qos_profile = rclpy.qos.QoSProfile(depth=1)
        qos_profile.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        qos_profile.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE
        # Parameters
        self.dt = 0.1  # Time step
        self.horizon = 30 # Horizon length
        self.num_samples = 100  # Number of sampled trajectories
        self.control_dim = 2  # [v, omega]
        self.gamma = 0.5 #mperature parameter for weighting

        # Robot state
        self.robot_pose = np.zeros(3)  # [x, y, theta]
        self.global_path = []

        # Subscribers
        self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, qos_profile)

        # Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Subscriptions and Publishers initialized')

    def path_callback(self, msg):
        self.get_logger().info('Path received with %d poses' % len(msg.poses))
        self.global_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.get_logger().info('Global path updated: %s' % str(self.global_path))
        self.follow_path()  # Trigger follow_path after updating the path


    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.get_logger().info('Pose updated: x=%.2f, y=%.2f, theta=%.2f' % (
        self.robot_pose[0], self.robot_pose[1], self.robot_pose[2]
    ))

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.robot_pose = 0.9 * self.robot_pose + 0.1 * np.array([
            position.x,
            position.y,
            self.quaternion_to_yaw(orientation)
        ])
        self.get_logger().info('Updated pose: %s' % str(self.robot_pose))
        
        self.follow_path()

    def follow_path(self):
        if not self.global_path:
            self.get_logger().warn('No global path available')
            return
        
        # Hedef noktaya olan mesafeyi hesapla
        goal = np.array(self.global_path[-1])  # Son hedef
        distance_to_goal = np.linalg.norm(self.robot_pose[:2] - goal)

        # Hedefe çok yaklaşıldığında durma
        if distance_to_goal < 0.2:  # Bu mesafeyi ihtiyaca göre ayarlayabilirsiniz
            self.get_logger().info('Goal reached, stopping the robot')
            self.stop_robot()
            return

        # Robotun mevcut yönelimine doğru dönüş yap
        heading_error = self.get_heading_error(goal)
        control = self.mppi_control(heading_error)

        # Hedefe doğru yumuşak bir şekilde yönel
        if control is not None:
            self.publish_velocity(control)

    def get_heading_error(self, goal):
        """Hedefin robotun mevcut yönelimine göre hata açısını hesaplar."""
        goal_angle = np.arctan2(goal[1] - self.robot_pose[1], goal[0] - self.robot_pose[0])
        heading_error = goal_angle - self.robot_pose[2]
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi  # -pi ile pi arasında tut
        return heading_error

    def mppi_control(self, heading_error):
        if len(self.global_path) < 2:
            return None

        # Başlangıç durumu
        x0 = self.robot_pose
        controls = np.zeros((self.horizon, self.control_dim))  # [v, omega] kontrol dizisi

        # Örneklenen kontrol yolları
        sampled_trajectories = np.random.randn(self.num_samples, self.horizon, self.control_dim) * 0.1
        costs = np.zeros(self.num_samples)

        for i, trajectory in enumerate(sampled_trajectories):
            state = np.copy(x0)
            for t in range(self.horizon):
                control = trajectory[t]
                state = self.simulate(state, control)
                costs[i] += self.cost(state, t)

        # Ağırlıklı kontrol
        weights = np.exp(-costs / self.gamma)
        weights /= np.sum(weights)

        # Ağırlıklı kontrol toplamı
        for i, trajectory in enumerate(sampled_trajectories):
            controls += weights[i] * trajectory

        # Başlangıçta hedefe doğru düzgün bir dönüş yapacak şekilde kontrolü ayarla
        if abs(heading_error) < 0.1:  # Küçük bir hata olduğunda dönüşü durdur
            controls[:, 1] = 0  # Dönüş hızını sıfırlayarak doğrusal hızda ilerlemesini sağla

        return controls[0]  # İlk kontrol aksiyonunu geri döndür

    

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    def simulate(self, state, control):
        """Simulate the robot's motion for one timestep."""
        x, y, theta = state
        v, omega = control
        new_x = x + v * np.cos(theta) * self.dt
        new_y = y + v * np.sin(theta) * self.dt
        new_theta = theta + omega * self.dt
        return np.array([new_x, new_y, new_theta])

    def cost(self, state, t):
        """Cost function combining path following and obstacle avoidance."""
        # Path following cost
        if t < len(self.global_path):
            goal = np.array(self.global_path[t])
            path_cost = np.linalg.norm(state[:2] - goal)
        else:
            path_cost = 0.0

        # Control effort cost
        control_effort_cost = 0.01 * np.sum(np.square(state))

        reversing_penalty = 100.0 if state[2] < 0 else 0.0  # Penalize negative linear velocity


        return path_cost + control_effort_cost + reversing_penalty

    def publish_velocity(self, control):
        """Publish the velocity command."""
        twist = Twist()
        twist.linear.x = min(control[0], 1.0)  # Cap maximum linear velocity
        twist.angular.z = control[1]
        self.cmd_vel_publisher.publish(twist)

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle."""
        return np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


def main(args=None):
    rclpy.init(args=args)
    mppi_controller = MPPIController()
    rclpy.spin(mppi_controller)
    mppi_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

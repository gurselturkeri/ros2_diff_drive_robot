import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.command = Twist()
        self.step = 0
        self.start_time = time.time()

    def timer_callback(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        if self.step == 0:
            # Move forward
            self.command.linear.x = 0.2
            self.command.angular.z = 0.0
            if elapsed_time > 5:  # Move forward for 5 seconds
                self.step = 1
                self.start_time = time.time()

        elif self.step == 1:
            # Turn right
            self.command.linear.x = 0.0
            self.command.angular.z = 1.0
            if elapsed_time > 5:  # Turn right for 2 seconds
                self.step = 2
                self.start_time = time.time()

        elif self.step == 2:
            # Move forward again
            self.command.linear.x = 0.2
            self.command.angular.z = 0.0
            if elapsed_time > 8:  # Move forward for another 5 seconds
                self.step = 3
                self.start_time = time.time()

        else:
            # Stop the robot
            self.command.linear.x = 0.0
            self.command.angular.z = 0.0

        self.publisher_.publish(self.command)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

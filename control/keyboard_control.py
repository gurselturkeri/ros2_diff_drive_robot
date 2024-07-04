#! /usr/bin/env python3

"""
    Author: Gaurav Gupta
    Email: gaurav@blackcoffeerobotics.com
    Ref: https://stackoverflow.com/questions/510357/how-to-read-a-single-character-from-the-user
"""


import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class _Getch:
    """
    Gets a single character from standard input. 
    Does not echo to the screen.
    """

    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self):
        return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty
        import sys

    def __call__(self):
        import sys
        import tty
        import termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


class TeleopPublisher(Node):

    def __init__(self):
        super().__init__('bcr_teleop_node')
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        timer_period = 0.05
        self.timer = self.create_timer(
            timer_period, self.velocity_publish_event)
        self.cmd_vel_msg = Twist()

    def set_vel(self, v, w):
        self.get_logger().info("v: %f, w: %f" % (v, w))
        self.cmd_vel_msg.linear.x = v
        self.cmd_vel_msg.angular.z = w

    def velocity_publish_event(self):
        self.vel_publisher.publish(self.cmd_vel_msg)


def main(args=None):
    rclpy.init(args=args)
    getch = _Getch()
    publish_node = TeleopPublisher()
    thread = threading.Thread(target=rclpy.spin,
                              args=(publish_node, ), daemon=True)
    # Thread for node's timer callback
    thread.start()
    v = 0.0
    w = 0.0
    publish_node.get_logger().info("\n\tw: increment linear velocity by 0.1,\n\
        s: decrement linear velocity by 0.1,\n\
        a: increment angular velocity by 0.1,\n\
        d: decrement angular velocity by 0.1,\n\
        space: zero velocity command,\n\
        q: QUIT")
    try:
        while (rclpy.ok()):
            key_in = getch()
            if key_in == "w":
                v += 0.1
            elif key_in == "s":
                v -= 0.1
            elif key_in == "d":
                w -= 0.1
            elif key_in == "a":
                w += 0.1
            elif key_in == "q":
                break
            else:
                v = 0.0
                w = 0.0
            publish_node.set_vel(v, w)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()

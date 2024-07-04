import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import random
import math

class NodeRRT:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

class RRTStar(Node):
    def __init__(self):
        super().__init__('rrt_star')
        
        qos_profile = rclpy.qos.QoSProfile(depth=1)
        qos_profile.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        qos_profile.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile)
        self.map_data = None

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info('Map data received')
        if self.map_data is not None:
            self.run_rrt_star()

    def run_rrt_star(self):
        start = NodeRRT(-0.307, -2.146)   # Replace with actual start coordinates
        goal = NodeRRT(9.947, 1.424)  # Replace with actual goal coordinates
        max_iter = 1000
        step_size = 0.5
        goal_sample_rate = 0.1

        path = self.rrt_star(start, goal, max_iter, step_size, goal_sample_rate)
        if path:
            self.visualize(path)

    def rrt_star(self, start, goal, max_iter, step_size, goal_sample_rate):
        nodes = [start]

        for i in range(max_iter):
            rand_node = self.get_random_node(goal, goal_sample_rate)
            nearest_node = self.get_nearest_node(nodes, rand_node)
            new_node = self.steer(nearest_node, rand_node, step_size)

            if self.check_collision(new_node):
                continue

            near_nodes = self.get_near_nodes(nodes, new_node, step_size * 2)
            new_node = self.choose_parent(near_nodes, new_node)
            nodes.append(new_node)
            self.rewire(nodes, near_nodes, new_node)

            if self.distance(new_node, goal) < step_size:
                final_node = self.steer(new_node, goal, step_size)
                if not self.check_collision(final_node):
                    return self.generate_final_path(final_node)

        return None

    def get_random_node(self, goal, goal_sample_rate):
        if random.random() > goal_sample_rate:
            return NodeRRT(random.uniform(0, self.map_data.info.width), random.uniform(0, self.map_data.info.height))
        return goal

    def get_nearest_node(self, nodes, rand_node):
        return min(nodes, key=lambda node: self.distance(node, rand_node))

    def steer(self, from_node, to_node, step_size):
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_node = NodeRRT(from_node.x + step_size * math.cos(theta), from_node.y + step_size * math.sin(theta))
        new_node.parent = from_node
        return new_node

    def check_collision(self, node):
        x = int(node.x)
        y = int(node.y)
        if x < 0 or y < 0 or x >= self.map_data.info.width or y >= self.map_data.info.height:
            return True
        return self.map_data.data[y * self.map_data.info.width + x] != 0

    def get_near_nodes(self, nodes, new_node, radius):
        return [node for node in nodes if self.distance(node, new_node) < radius]

    def choose_parent(self, near_nodes, new_node):
        if not near_nodes:
            return new_node
        min_cost = float('inf')
        best_node = None
        for node in near_nodes:
            cost = self.calculate_cost(node) + self.distance(node, new_node)
            if cost < min_cost:
                min_cost = cost
                best_node = node
        new_node.parent = best_node
        return new_node

    def rewire(self, nodes, near_nodes, new_node):
        for node in near_nodes:
            new_cost = self.calculate_cost(new_node) + self.distance(new_node, node)
            if new_cost < self.calculate_cost(node):
                node.parent = new_node

    def calculate_cost(self, node):
        cost = 0
        while node.parent:
            cost += self.distance(node, node.parent)
            node = node.parent
        return cost

    def generate_final_path(self, node):
        path = []
        while node:
            path.append([node.x, node.y])
            node = node.parent
        return path[::-1]

    def distance(self, node1, node2):
        return math.hypot(node1.x - node2.x, node1.y - node2.y)

    def visualize(self, path):
        map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
        plt.imshow(map_array, cmap='gray')
        path = np.array(path)
        plt.plot(path[:, 0], path[:, 1], 'r')
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    rrt_star_node = RRTStar()
    rclpy.spin(rrt_star_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

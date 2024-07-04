import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import heapq

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.get_logger().info('Initializing Path Planner Node')

        # Define QoS profile
        qos_profile = rclpy.qos.QoSProfile(depth=1)
        qos_profile.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        qos_profile.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE

        self.subscription = self.create_subscription(OccupancyGrid,'/map',self.map_callback,qos_profile)

        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)

        self.get_logger().info('Subscriptions and Publishers initialized')

    def map_callback(self, msg):
        self.get_logger().info('Map received')
        self.grid = msg.data
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin
        self.create_occupancy_grid()

        # Example start and goal positions (in meters)
        start = (-0.30754125118255615, -2.1467812061309814)  # Replace with actual robot's current position
        goal = (17.745447158813477, 18.52811050415039)  # Replace with actual goal position
        path = self.plan_path(start, goal)
        if path:
            self.get_logger().info(f'Path found: {path}')
            self.send_path_to_robot(path)
        else:
            self.get_logger().warn('No path found')

    def create_occupancy_grid(self):
        self.occupancy_grid = []
        for i in range(self.height):
            row = []
            for j in range(self.width):
                row.append(self.grid[i * self.width + j])
            self.occupancy_grid.append(row)

    def a_star(self, start, goal):
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < self.height and 0 <= neighbor[1] < self.width and self.occupancy_grid[neighbor[0]][neighbor[1]] == 0:
                    tentative_g_score = g_score[current] + 1
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))
        return None

    def plan_path(self, start, goal):
        start_grid = (int((start[1] - self.origin.position.y) / self.resolution),
                      int((start[0] - self.origin.position.x) / self.resolution))
        goal_grid = (int((goal[1] - self.origin.position.y) / self.resolution),
                     int((goal[0] - self.origin.position.x) / self.resolution))
        path = self.a_star(start_grid, goal_grid)
        if path:
            path_coords = [(x * self.resolution + self.origin.position.x,
                            y * self.resolution + self.origin.position.y) for y, x in path]
            return path_coords
        else:
            return None

    def send_path_to_robot(self, path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        for coord in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = coord[0]
            pose.pose.position.y = coord[1]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.get_logger().info('Publishing path')
        self.path_publisher.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    path_planner = PathPlanner()
    rclpy.spin(path_planner)
    path_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

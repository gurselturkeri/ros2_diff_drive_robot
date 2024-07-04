import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt

class MapVisualizer(Node):
    def __init__(self):
        super().__init__('map_visualizer')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        plt.ion()  # Turn on interactive mode
        self.fig, self.ax = plt.subplots()

    def listener_callback(self, msg):
        # Check if data is being received
        self.get_logger().info(f'Received map data: width={msg.info.width}, height={msg.info.height}')

        # Convert the occupancy grid to a numpy array
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))

        # Check the range of data values
        self.get_logger().info(f'Data min: {data.min()}, max: {data.max()}')

        # Normalize the data for better visualization
        data = np.where(data == -1, 50, data)  # Treat unknown (-1) as 50
        data = np.where(data == 100, 255, data)  # Occupied as 255
        data = np.where(data == 0, 0, data)  # Free as 0

        # Check some statistics about the data
        occupied_count = np.sum(data == 255)
        free_count = np.sum(data == 0)
        unknown_count = np.sum(data == 50)
        self.get_logger().info(f'Occupied cells: {occupied_count}, Free cells: {free_count}, Unknown cells: {unknown_count}')

        # Display the map
        self.ax.clear()
        self.ax.imshow(data, cmap='gray', origin='lower')
        self.ax.set_title('Occupancy Grid Map')
        self.fig.colorbar(self.ax.imshow(data, cmap='gray', origin='lower'), ax=self.ax)
        plt.draw()
        plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    map_visualizer = MapVisualizer()
    
    try:
        rclpy.spin(map_visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        map_visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv

class OrbitPlotNode(Node):
    def __init__(self):
        super().__init__("orbit_plot_node")

        # ROS subscription
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/satellite_orbit',
            self.listener_callback,
            10
        )

        # Data storage for plotting
        self.x_data = []
        self.y_data = []
        self.z_data = []

        # Open CSV file once and append data
        self.csv_file = open('data.csv', 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)

    def listener_callback(self, msg):
        """Callback function for ROS 2 subscriber."""
        self.get_logger().info(f"Data received: {msg.data}")

        if len(msg.data) >= 3:
            self.x_data.append(msg.data[0])
            self.y_data.append(msg.data[1])
            self.z_data.append(msg.data[2])

            # Limit stored data to prevent memory overflow (optional)
            if len(self.x_data) > 100:
                self.x_data.pop(0)
                self.y_data.pop(0)
                self.z_data.pop(0)

            # Write to CSV
            self.csv_writer.writerow([msg.data[0], msg.data[1], msg.data[2]])

    def destroy_node(self):
        """Ensure the CSV file is properly closed on shutdown."""
        self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = OrbitPlotNode()

    # Use an executor to manage spinning in a single thread
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Set up 3D plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-10000000, 10000000])
    ax.set_ylim([-10000000, 10000000])
    ax.set_zlim([-10000000, 10000000])
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')

    # We create a line object that we'll update on every iteration
    line, = ax.plot([], [], [], marker='o', linestyle='-')

    # Switch to interactive mode so plt.show() doesn't block
    plt.ion()
    plt.show()

    try:
        while rclpy.ok():
            # Spin node briefly (non-blocking)
            executor.spin_once(timeout_sec=0.1)

            # Update line data from the node's stored data
            if node.x_data:
                line.set_data(node.x_data, node.y_data)
                line.set_3d_properties(node.z_data)

            # Update the plot limits (if desired, or set them once above)
            ax.set_xlim([-10000000, 10000000])
            ax.set_ylim([-10000000, 10000000])
            ax.set_zlim([-10000000, 10000000])

            # Redraw the figure
            plt.draw()
            plt.pause(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy, csv, threading, numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class OrbitPlotNode(Node):
    def __init__(self):
        super().__init__('orbit_plot_node')

        self.create_subscription(Float32MultiArray,
                                 '/satellite_orbit',
                                 self.listener_callback, 10)

        # trajectory data (protected by a lock – we use threads)
        self._lock   = threading.Lock()
        self.x_list, self.y_list, self.z_list = [], [], []

        self.csv_file   = open('data.csv', 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)

    # ───────────────────────────────── ROS callback ────────────────────────────
    def listener_callback(self, msg):
        if len(msg.data) < 3:
            return

        with self._lock:                         # thread‑safe append
            self.x_list.append(float(msg.data[0]))
            self.y_list.append(float(msg.data[1]))
            self.z_list.append(float(msg.data[2]))

        # raw logging (no lock needed, csv handles its own buffering)
        self.csv_writer.writerow(msg.data[:3])

    # snapshot gives the GUI thread a *consistent copy*
    def snapshot(self):
        with self._lock:
            return (np.asarray(self.x_list, dtype=float),
                    np.asarray(self.y_list, dtype=float),
                    np.asarray(self.z_list, dtype=float))

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

# ──────────────────────────────────── main ────────────────────────────────────
def main():
    rclpy.init()
    node      = OrbitPlotNode()
    executor  = MultiThreadedExecutor()
    executor.add_node(node)

    # Matplotlib: one Line3D for the trail, one scatter for the satellite
    fig = plt.figure()
    ax  = fig.add_subplot(111, projection='3d')
    limit = (-1.0e7, 1.0e7)
    ax.set(xlim=limit, ylim=limit, zlim=limit,
           xlabel='x [m]', ylabel='y [m]', zlabel='z [m]')

    trail_line, = ax.plot([], [], [], lw=1.0)              # whole trajectory
    head_scatter = ax.scatter([], [], [], s=30)            # current position

    plt.ion(); plt.show()

    try:
        while rclpy.ok():
            executor.spin_once(0.01)                       # 100 Hz GUI update

            xs, ys, zs = node.snapshot()
            if xs.size == 0:                               # nothing yet
                continue

            # update the trail
            trail_line.set_data(xs, ys)
            trail_line.set_3d_properties(zs)

            # update the head (single latest point)
            head_scatter._offsets3d = (xs[-1:], ys[-1:], zs[-1:])

            plt.draw(); plt.pause(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
import numpy as np


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(MagneticField, '/bluerov/sensors/Compass', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = MagneticField()
        # les valeurs sont usuellement en Tesla
        msg.magnetic_field.x = 0.
        msg.magnetic_field.y = 0.
        msg.magnetic_field.z = 0.

        msg.magnetic_field_covariance = np.zeros(9, dtype = np.float64)

        self.publisher_.publish(msg)
        self.get_logger().info('good')



def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
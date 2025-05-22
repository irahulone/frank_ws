import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16
from geometry_msgs.msg import Twist

class du_controller(Node):

    def __init__(self):
        super().__init__('du_controller')
        self.publisher_ = self.create_publisher(Int16, 'du1_pwr', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int16()
        msg.data = 112
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    du_ctrl = du_controller()

    rclpy.spin(du_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    du_ctrl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
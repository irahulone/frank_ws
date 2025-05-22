import rclpy
import serial
from rclpy.node import Node
import time
from std_msgs.msg import Int16

roboteq_obj = serial.Serial(
port='/dev/ttyACM0',
baudrate=115200,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS,
timeout=1
)

roboteq_obj_1 = serial.Serial(
port='/dev/ttyACM1',
baudrate=115200,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS,
timeout=1
)

class MotorDriver(Node):

    def __init__(self):
        super().__init__('cmd_roboteq')

        # Initialize the 4 motor drivers on the rover.
        self.subscription = self.create_subscription(
            Int16,
            'rov/m1/rpm',
            lambda msg: self.cmd_callback_1(msg, 1, roboteq_obj),
            5)

        self.subscription = self.create_subscription(
            Int16,
            'rov/m2/rpm',
            lambda msg: self.cmd_callback_1(msg, 2, roboteq_obj),
            5)

        self.subscription = self.create_subscription(
            Int16,
            'rov/m3/rpm',
            lambda msg: self.cmd_callback_2(msg, 1, roboteq_obj_1),
            5)

        self.subscription = self.create_subscription(
            Int16,
            'rov/m4/rpm',
            lambda msg: self.cmd_callback_2(msg, 2, roboteq_obj_1),
            5)

        self.subscription  # prevent unused variable warning

    # Constructs a payload for the motor drivers.
    def move_motor(self, val, motor_num, rob_obj):
        payload = "!G "+ str(motor_num) + " " + str(-val) + "_"
        rob_obj.write(str.encode(payload))

    # Passes the callback into the driver nodes.
    def cmd_callback_1(self, msg, motor_num, rob_obj):
        inCmd = msg.data
        self.move_motor(inCmd, motor_num, rob_obj)
 

# Main function for the node.
def main(args=None):
    # Initialize rclpy node.
    rclpy.init(args=args)
    moter_driver_node = MotorDriver()
    time.sleep(2)
    rclpy.spin(moter_driver_node)
    moter_driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import serial
class RoboteQ_MC:

    def __init__(self, port):

        self.ser = serial.Serial(port, 115200, timeout=None)

        self.is_open = self.ser.isOpen()

        if self.is_open:

            print('pydexarm: %s open' % self.ser.name)

        else:

            print('Failed to open serial port')

    def _send_cmd(self, data):

        #self.ser.write(data)

        self.ser.write(data.encode())


    def channel_1(self, x):

        cmd = "!G 1 " + str(x) + "_"

        self._send_cmd(cmd)

    def channel_2(self, x):

        cmd = "!G 2 " + str(x) + "_"

        self._send_cmd(cmd)

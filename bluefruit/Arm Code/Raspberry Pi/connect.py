import serial
from serial.tools import list_ports


class Arduino(object):
    def __init__(self, port=0):
        ser_port = '/dev/ttyACM' + str(port)
        try:
            self.ser = serial.Serial(
                ser_port,
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
        except:
            print("Error: /dev/ttyACM%s doesn't exist (use port arg to select)" % port)
            quit()

    def read(self, *args, **kwargs):
        input = self.ser.read()
        print(input)

    def write(self, string, **kwargs):
        self.ser.write(string.encode())

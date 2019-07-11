import serial
from serial.tools import list_ports
import struct


class Arduino(object):
    def __repr__(self):
        return self.name

    def __init__(self, name='Arduino', port='/dev/ttyACM0', baud=115200):
        self.name = name
        ser_port = port
        try:
            self.ser = serial.Serial(
                ser_port,
                baudrate=baud,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
        except Exception:
            print("Error: /dev/ttyACM%s doesn't exist (use port arg to select)" % port)
            quit()

    def read(self, *args, **kwargs):
        input = self.ser.read()
        print(input)

    def write(self, data, **kwargs):
        try:
            self.ser.write(struct.pack('>B', data))
        except Exception:
            self.ser.write(data.encode())

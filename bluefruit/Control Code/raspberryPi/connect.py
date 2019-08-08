import serial
from serial.tools import list_ports
import struct


class Arduino(object):
    def __repr__(self):
        return self.name

    def __init__(self, name='Arduino', port='/dev/ttyACM0', baud=115200):
        self.name = name
        self.baudrate = baud
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
        except Exception as e:
            print(e)
            quit()

    def disconnect(self):
        try:
            self.ser.close()
        except Exception as e:
            print("failed to disconnect")
            print(e)

    def read(self, *args, **kwargs):
        input = self.ser.read()
        print(input)

    def write(self, data, **kwargs):
        try:
            try:
                self.ser.write(struct.pack('>B', data))
            except Exception:
                self.ser.write(data.encode())
        except Exception as e:
            print(e)
            exit()

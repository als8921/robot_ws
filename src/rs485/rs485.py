import time
import struct
import serial
import serial.rs485
from MD400 import MD400

SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 19200

class RS485Communication:
    def __init__(self):
        self.ser = serial.rs485.RS485(SERIAL_PORT, baudrate=BAUDRATE, timeout=1)
        self.ser.rs485_mode = serial.rs485.RS485Settings()
        self.complete = False

    def read_data(self):
        if self.ser.readable():
            response = bytearray()
            response.extend(self.ser.read())
            if(response and response[0] == md400.TMID):
                response.extend(self.ser.read(4))
                additional_read = response[-1] + 1
                response.extend(self.ser.read(additional_read))
                print("Received:    ", *response)

                if(response[3]==197):
                    print("Position : ", md400.bytes_to_pos())

    def send_data(self, data):
        try:
            self.ser.write(data)
            print("Sent         ", *data)
        except Exception as e:
            print(f"Error sending data: {e}")

    def run(self):
        while True:
            self.read_data()
            if(not self.complete):
                packet = md400.set_alarm_reset()
                # packet = md400.set_pos(100)
                # packet = md400.stop()
                # packet = md400.homing()
                # packet = md400.get_pos()
                self.complete = True
                self.send_data(packet)

    def close(self):
        self.ser.close()


if __name__ == '__main__':
    comm = RS485Communication()
    md400 = MD400()
    md400.RMID = 0xb7
    md400.TMID = 0xb8
    md400.ID = 0x01
    try:
        comm.run()
    finally:
        comm.close()

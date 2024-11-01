import time
import serial
import serial.rs485
from MD400 import MD400
from collections import deque

# SERIAL_PORT = '/dev/ttyACM0'
SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 19200

class RS485Communication:
    def __init__(self):
        self.ser = serial.rs485.RS485(SERIAL_PORT, baudrate=BAUDRATE, timeout=1)
        self.ser.rs485_mode = serial.rs485.RS485Settings()
        self.command_queue = deque()

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
                    print("Position : ", md400.bytes_to_pos(response))

    def send_data(self, data):
        try:
            self.ser.write(data)
            print("Sent         ", *data)
        except Exception as e:
            print(f"Error sending data: {e}")

    def run(self):
        while True:
            self.read_data()
            while(self.command_queue):
                packet = self.command_queue.popleft()
                self.send_data(packet)
                self.read_data()
                time.sleep(0.5)

    def close(self):
        self.ser.close()


if __name__ == '__main__':
    comm = RS485Communication()
    md400 = MD400(rmid = 0xb7, tmid = 0xb8, id = 0x01)

    command = [
        # md400.set_alarm_reset(),
        # md400.set_direction_ccw(),
        # md400.set_direction_cw(),
        # md400.get_pos(),
        # md400.set_pos(0),
        # md400.stop(),
        # md400.homing(),
        # md400.set_rpm(2000),
        # md400.set_current_limit(10)
    ]

    comm.command_queue.extend(command)
    try:
        comm.run()
    finally:
        comm.close()

import time
import struct
import serial
import serial.rs485

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 19200

class RS485Communication:
    def __init__(self):
        self.ser = serial.rs485.RS485(SERIAL_PORT, baudrate=BAUDRATE, timeout=1)
        self.ser.rs485_mode = serial.rs485.RS485Settings()
        self.RMID = 0xb7
        self.TMID = 0xb8
        self.ID = 0x01

    def create_cmd_packet(self, RMID, TMID, ID, PARAMETER_ID, DATA_NUMBER, *DATA):
        check_sum = RMID + TMID + ID + PARAMETER_ID + DATA_NUMBER + sum(DATA)
        check_sum = (256 - (check_sum % 256)) % 256
        fmt = 'B' * (len(DATA) + 6)
        
        packet_obj = struct.pack(fmt, 
            RMID,
            TMID,
            ID,
            PARAMETER_ID,
            DATA_NUMBER,
            *DATA,
            check_sum)
        return packet_obj

    def read_data(self):
        if self.ser.readable():
            response = bytearray()
            response.extend(self.ser.read())
            if(response and response[0] == self.TMID):
                response.extend(self.ser.read(4))
                additional_read = response[-1] + 1
                response.extend(self.ser.read(additional_read))
                print("Received:    ", self.bytes_to_decimal(response))

    def send_data(self, data):
        try:
            self.ser.write(data)
            print("Sent         ", self.bytes_to_decimal(data))
        except Exception as e:
            print(f"Error sending data: {e}")


    def bytes_to_decimal(self, byte_string):
        return [byte for byte in byte_string]

    def run(self):
        while True:
            self.read_data()

            packet = self.create_cmd_packet(self.RMID, self.TMID, self.ID, 10, 3, 8)
            # packet = self.create_cmd_packet(183, 184, 1, 244, 4, 200, 0, 0, 0)
            # packet = self.create_cmd_packet(1, 2, 3, 10, 4, 5, 6, 10, 0)
            self.send_data(packet)

    def close(self):
        self.ser.close()


if __name__ == '__main__':
    comm = RS485Communication()
    try:
        comm.run()
    finally:
        comm.close()

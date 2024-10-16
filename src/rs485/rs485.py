import time
import struct
import serial
import serial.rs485

SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 19200

class RS485Communication:
    def __init__(self):
        self.ser = serial.rs485.RS485(SERIAL_PORT, baudrate=BAUDRATE, timeout=1)
        self.ser.rs485_mode = serial.rs485.RS485Settings()
        self.RMID = 0xb7
        self.TMID = 0xb8
        self.ID = 0x01
        self.fmt = None

    def create_cmd_packet(self, RMID, TMID, ID, PARAMETER_ID, DATA_NUMBER, *DATA):
        check_sum = RMID + TMID + ID + PARAMETER_ID + DATA_NUMBER + sum(DATA)
        check_sum = (256 - (check_sum % 256)) % 256
        print(check_sum)
        fmt = 'B' * (len(DATA) + 6)
        self.fmt = fmt
        
        packet_obj = struct.pack(fmt, 
            RMID,
            TMID,
            ID,
            PARAMETER_ID,
            DATA_NUMBER,
            *DATA,
            check_sum)
        return packet_obj
        
    def unpack_cmd(self, data):
        return struct.unpack(self.fmt, data)




    def read_data(self):
        if self.ser.readable():
            response = self.ser.readline()
            print("Received: ", response)

    def send_data(self, data):
        try:
            self.ser.write(data)
            print(f"Sent: {data}")
        except Exception as e:
            print(f"Error sending data: {e}")

    def run(self):
        # while True:

        for i in range(5):            # 데이터 수신
            self.read_data()

            # 패킷 생성
            # packet = self.create_cmd_packet(self.RMID, self.TMID, self.ID, 10, 1, 8)
            packet = self.create_cmd_packet(183, 184, 1, 244, 4, 200, 0, 0, 0)
            self.send_data(packet)
            time.sleep(3)

    def close(self):
        self.ser.close()


if __name__ == '__main__':
    comm = RS485Communication()
    try:
        comm.run()
    finally:
        comm.close()

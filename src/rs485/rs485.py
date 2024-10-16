import time
import struct
import serial

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 19200

class RS485Communication:
    def __init__(self):
        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        self.RMID = 0xb7
        self.TMID = 0xac
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
        if(self.ser.readable()):
            response = self.ser.readline()
            print("res : ", response)


    def send_data(self, data):
        try:
            self.ser.write(data)
            print(f"Sent: {data}")
        except Exception as e:
            print(f"Error sending data: {e}")

    def run(self):
        while True:
            # 데이터 수신
            rx_data = self.read_data()
            if rx_data:
                print(f"Received: {rx_data}")

            packet = self.create_cmd_packet(self.RMID, self.TMID, self.ID, 10, 1, 8)
            self.send_data(packet)
            time.sleep(0.1)

    def close(self):
        self.ser.close()


if __name__ == '__main__':
    comm = RS485Communication()
    try:
        comm.run()
    finally:
        comm.close()

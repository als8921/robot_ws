import struct

class MD400:
    def __init__(self):
        self.RMID = 0xb7
        self.TMID = 0xb8
        self.ID = 0x01

    def create_packet(self, RMID, TMID, ID, PARAMETER_ID, DATA_NUMBER, *DATA):
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
    
    def set_alarm_reset(self):
        return self.create_packet(self.RMID, self.TMID, self.ID, 10, 1, 8)

    def set_direction_cw(self):
        return self.create_packet(self.RMID, self.TMID, self.ID, 16, 1, 0)

    def set_direction_ccw(self):
        return self.create_packet(self.RMID, self.TMID, self.ID, 16, 1, 1)

    def stop(self):
        return self.create_packet(self.RMID, self.TMID, self.ID, 5, 1, 1)

    def homing(self):
        return self.create_packet(self.RMID, self.TMID, self.ID, 10, 1, 90)

    def get_pos(self):
        """
            Tx를 보내면 Rx로 값을 받아옴
        """
        return self.create_packet(self.RMID, self.TMID, self.ID, 4, 1, 197)
    
    def bytes_to_pos(self, packet):
        """
            Rx로 받아온 패킷으로 현재 위치 값을 계산
        """
        
        hex_data = packet[5:5+packet[4]]
        print(hex_data)

        hex_string = ''.join(format(byte, '02x') for byte in reversed(hex_data))

        decimal_value = int(hex_string, 16)

        return decimal_value

    def pos_to_bytes(self, decimal_value):
        """
            10진수 값을 받아 4바이트로 변환
        """
        hex_string = format(decimal_value, '08x')
        byte_array = bytearray.fromhex(hex_string)
        position_bytes = [byte for byte in reversed(byte_array)]

        return position_bytes

    def set_pos(self, pos):
        
        return self.create_packet(self.RMID, self.TMID, self.ID, 219, *self.pos_to_bytes(pos), 0, 0)

    def set_rpm(self):
        return

    def set_current_limit(self):
        return
    
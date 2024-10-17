import struct

class MD400:
    def __init__(self, rmid, tmid, id):
        self.RMID = rmid
        self.TMID = tmid
        self.ID = id

    def create_packet(self, RMID, TMID, ID, PARAMETER_ID, DATA_NUMBER, *DATA):
        """
            CheckSum 계산 후 패킷 생성 
        """
        check_sum = RMID + TMID + ID + PARAMETER_ID + DATA_NUMBER + sum(DATA)
        check_sum = (256 - (check_sum % 256)) % 256
        fmt = 'B' * (len(DATA) + 6)
        
        packet_obj = struct.pack(fmt, RMID, TMID, ID, PARAMETER_ID, DATA_NUMBER, *DATA, check_sum)
        return packet_obj
    
    ################################################################
    # Tx 명령 전달
    ################################################################
    def set_alarm_reset(self):
        """
            모든 에러 알람을 리셋
        """
        return self.create_packet(self.RMID, self.TMID, self.ID, 10, 1, 8)

    def set_direction_cw(self):
        """
            모터 회전 방향을 정방향으로 설정 (Default : 역방향/ 리프트 상승 방향)
        """
        return self.create_packet(self.RMID, self.TMID, self.ID, 16, 1, 0)

    def set_direction_ccw(self):
        """
            모터 회전 방향을 역방향으로 설정 (Default : 역방향/ 리프트 상승 방향)
        """
        return self.create_packet(self.RMID, self.TMID, self.ID, 16, 1, 1)

    def stop(self):
        """
            모터를 부드럽게 정지
        """
        return self.create_packet(self.RMID, self.TMID, self.ID, 5, 1, 1)

    def homing(self):
        """
            영점을 찾기위해 Calibration 진행
        """
        return self.create_packet(self.RMID, self.TMID, self.ID, 10, 1, 90)

    def set_pos(self, pos):
        """
            지정된 카운트 값 위치로 모터 제어
        """
        # Position control with Target Speed(PID 219)
        # return self.create_packet(self.RMID, self.TMID, self.ID, 219, 6, *self.pos_to_bytes(pos), 0, 0)

        # Position Control with Set Speed(PID 243)
        return self.create_packet(self.RMID, self.TMID, self.ID, 243, 4, *self.pos_to_bytes(pos))

    ################################################################
    # Rx 데이터 요청
    ################################################################
    def get_pos(self):
        """
            현재 위치 값 요청 -> 값을 받아와야함
        """
        return self.create_packet(self.RMID, self.TMID, self.ID, 4, 1, 197)
    
    def get_alarm_log(self):
        return self.create_packet(self.RMID, self.TMID, self.ID, 229, 1, 0)

    

    ################################################################
    # 계산 하는 부분
    ################################################################
    def bytes_to_pos(self, packet):
        """
            Rx로 받아온 패킷으로 현재 위치 값을 계산
        """
        
        hex_data = packet[5:5+packet[4]]

        hex_string = ''.join(format(byte, '02x') for byte in reversed(hex_data))

        decimal_value = int(hex_string, 16)

        return decimal_value

    def pos_to_bytes(self, decimal_value):
        """
            10진수 값을 받아 16진수 4바이트로 변환
        """
        hex_string = format(decimal_value, '08x')
        byte_array = bytearray.fromhex(hex_string)
        position_bytes = [byte for byte in reversed(byte_array)]

        return position_bytes
    ################################################################

    def set_rpm(self):
        return

    def set_current_limit(self):
        return
    
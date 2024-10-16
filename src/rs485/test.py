import struct
def create_cmd_packet(RMID, TMID, ID, PARAMETER_ID, DATA_NUMBER, *DATA):
    check_sum = RMID + TMID + ID + PARAMETER_ID + DATA_NUMBER + sum(DATA)
    check_sum = (256 - (check_sum % 256)) % 256

    fmt = 'B' * (len(DATA) + 6)
    print(check_sum)
    
    packet_obj = struct.pack(fmt, 
        RMID,
        TMID,
        ID,
        PARAMETER_ID,
        DATA_NUMBER,
        *DATA,
        check_sum)
    return packet_obj

# 사용 예시
packet = create_cmd_packet(0xb7, 0xb8, 0x01, 10, 1, 8)
print(packet)

import serial
import time

WRITE_DATA = 0x03
ANGLE_OFSSET_IDX = 0x1F         # 位置補正
MOVING_MODE_IDX = 0x21         # 動作モード(1byte)
POSITION_IDX = 0x2A
TIME_IDX = 0x2C
SPEED_IDX = 0x2E

class Command:
    def __init__(self):
        self.message = []

    def init_message(self):
        self.message = [0] * 6
        self.message[0] = 0xFF;  # ヘッダ
        self.message[1] = 0xFF;  # ヘッダ
        self.message[2] = 0;    # サーボID
        self.message[3] = 0;  # パケットデータ長
        self.message[4] = 0;  # コマンド
        self.message[5] = 0;  # レジスタ先頭番号

    def set_position(self, id, pos):
        self.init_message()
        self.message[4] = WRITE_DATA
        self.message[2] = id
        self.message[3] = 0x09  # データ長(9byte)
        self.message[5] = POSITION_IDX

        self.message += [0]*2  # checksumは別

        self.message[6] = pos & 0xFF
        self.message[7] = (pos >> 8) & 0xFF
        self.checksum()

        
        print(self.message)
        return self.message
    
    def checksum(self):
        cksum = 0
        for i in range(2, len(self.message)):
            cksum += self.message[i]
        cksum = ~cksum & 0xFF
        self.message.append(cksum)  # チェックサム追加

class Bus:
    def __init__(self, serial_port):
        self.serial_port = serial_port
    
    def send_command(self, command):
        self.serial_port.write(command)
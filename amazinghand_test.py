#!/usr/bin/env python3
"""
AmazingHand (TsingSens) 測試腳本
Feetech SCS0009 Serial Bus Servo

使用方式:
    sudo python3 amazinghand_test.py           # 互動模式
    sudo python3 amazinghand_test.py scan      # 掃描舵機
    sudo python3 amazinghand_test.py move 1 512  # 移動舵機
    sudo python3 amazinghand_test.py open      # 張開
    sudo python3 amazinghand_test.py close     # 握緊
"""

import serial
import time
import sys

# 設定
PORT = '/dev/ttyUSB0'
BAUDRATE = 1000000  # 預設 1Mbps，可嘗試: 38400, 115200, 500000

# 位置範圍
POS_MIN = 0
POS_MAX = 1023
POS_OPEN = 200    # 張開位置
POS_CLOSE = 800   # 握緊位置


class FeetechServo:
    """Feetech SCS 協議驅動"""
    
    # 指令碼
    INST_PING = 0x01
    INST_READ = 0x02
    INST_WRITE = 0x03
    INST_REG_WRITE = 0x04
    INST_ACTION = 0x05
    INST_RESET = 0x06
    INST_SYNC_WRITE = 0x83
    
    # 暫存器位址
    ADDR_ID = 0x03
    ADDR_BAUD = 0x06
    ADDR_GOAL_POSITION = 0x2A
    ADDR_RUNNING_SPEED = 0x2C
    ADDR_PRESENT_POSITION = 0x38
    ADDR_PRESENT_SPEED = 0x3C
    ADDR_PRESENT_LOAD = 0x3E
    
    def __init__(self, port=PORT, baudrate=BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
    
    def connect(self):
        """連接串口"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.5)
            print(f"✅ 連線成功: {self.port} @ {self.baudrate} bps")
            return True
        except Exception as e:
            print(f"❌ 連線失敗: {e}")
            return False
    
    def disconnect(self):
        """斷開連接"""
        if self.ser:
            self.ser.close()
            print("已斷開連接")
    
    def _checksum(self, data):
        """計算校驗和"""
        return (~sum(data)) & 0xFF
    
    def _send_packet(self, servo_id, instruction, params=[]):
        """發送封包"""
        length = len(params) + 2  # instruction + checksum
        pkt = [0xFF, 0xFF, servo_id, length, instruction] + list(params)
        pkt.append(self._checksum(pkt[2:]))
        
        self.ser.reset_input_buffer()
        self.ser.write(bytes(pkt))
        self.ser.flush()
        time.sleep(0.02)
        
        resp = self.ser.read(self.ser.in_waiting or 20)
        return resp
    
    def ping(self, servo_id):
        """Ping 舵機"""
        resp = self._send_packet(servo_id, self.INST_PING)
        return len(resp) >= 6 and resp[0] == 0xFF and resp[1] == 0xFF
    
    def read_position(self, servo_id):
        """讀取當前位置"""
        resp = self._send_packet(servo_id, self.INST_READ, [self.ADDR_PRESENT_POSITION, 2])
        if len(resp) >= 8:
            return resp[5] | (resp[6] << 8)
        return None
    
    def write_position(self, servo_id, position, speed=500):
        """寫入目標位置"""
        position = max(POS_MIN, min(POS_MAX, position))
        pos_l = position & 0xFF
        pos_h = (position >> 8) & 0xFF
        spd_l = speed & 0xFF
        spd_h = (speed >> 8) & 0xFF
        
        # 寫入 Goal Position + Speed (連續 4 bytes)
        self._send_packet(servo_id, self.INST_WRITE, 
                         [self.ADDR_GOAL_POSITION, pos_l, pos_h, spd_l, spd_h])
    
    def scan(self, id_range=range(0, 254)):
        """掃描舵機"""
        found = []
        print(f"掃描舵機 ID {id_range.start}-{id_range.stop-1}...")
        
        for sid in id_range:
            if self.ping(sid):
                pos = self.read_position(sid)
                pos_str = f"位置={pos}" if pos is not None else ""
                print(f"  ID {sid:3d}: ✅ {pos_str}")
                found.append(sid)
        
        print(f"\n找到 {len(found)} 個舵機: {found}")
        return found


class AmazingHand:
    """AmazingHand 控制類"""
    
    def __init__(self, port=PORT, baudrate=BAUDRATE):
        self.servo = FeetechServo(port, baudrate)
        self.servo_ids = []
    
    def connect(self):
        """連接並掃描"""
        if not self.servo.connect():
            return False
        
        # 掃描 ID 1-8
        self.servo_ids = self.servo.scan(range(1, 9))
        
        if not self.servo_ids:
            print("\n⚠️ 未找到舵機，嘗試擴大掃描範圍...")
            self.servo_ids = self.servo.scan(range(0, 20))
        
        return len(self.servo_ids) > 0
    
    def disconnect(self):
        self.servo.disconnect()
    
    def move(self, servo_id, position, speed=500):
        """移動單個舵機"""
        print(f"移動 ID {servo_id} 到位置 {position}")
        self.servo.write_position(servo_id, position, speed)
    
    def move_all(self, position, speed=500):
        """移動所有舵機"""
        for sid in self.servo_ids:
            self.servo.write_position(sid, position, speed)
            time.sleep(0.03)
    
    def open_hand(self):
        """張開手掌"""
        print("🖐️ 張開手掌...")
        self.move_all(POS_OPEN)
    
    def close_hand(self):
        """握緊手掌"""
        print("✊ 握緊手掌...")
        self.move_all(POS_CLOSE)
    
    def wave(self):
        """揮手動作"""
        print("👋 揮手...")
        for _ in range(3):
            self.move_all(300, speed=800)
            time.sleep(0.4)
            self.move_all(700, speed=800)
            time.sleep(0.4)
        self.move_all(512)


def interactive_mode(hand):
    """互動模式"""
    print("\n" + "="*50)
    print("AmazingHand 互動控制")
    print("="*50)
    print("指令:")
    print("  1 - 張開 🖐️")
    print("  2 - 握緊 ✊")
    print("  3 - 中間位置")
    print("  4 - 揮手 👋")
    print("  m <ID> <位置> - 移動單個舵機")
    print("  s - 重新掃描")
    print("  q - 離開")
    print("="*50)
    
    while True:
        try:
            cmd = input("\n> ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == '1':
                hand.open_hand()
            elif cmd == '2':
                hand.close_hand()
            elif cmd == '3':
                print("移動到中間位置...")
                hand.move_all(512)
            elif cmd == '4':
                hand.wave()
            elif cmd == 's':
                hand.servo_ids = hand.servo.scan(range(0, 20))
            elif cmd.startswith('m '):
                parts = cmd.split()
                if len(parts) >= 3:
                    sid = int(parts[1])
                    pos = int(parts[2])
                    hand.move(sid, pos)
                else:
                    print("格式: m <ID> <位置>")
            else:
                print("未知指令")
                
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"錯誤: {e}")


def main():
    args = sys.argv[1:]
    
    hand = AmazingHand()
    
    if not hand.connect():
        print("\n❌ 無法連接 AmazingHand")
        print("請檢查:")
        print("  1. USB 連接是否正常")
        print("  2. 是否用 sudo 執行")
        print("  3. /dev/ttyUSB0 是否存在")
        return
    
    try:
        if not args:
            # 互動模式
            interactive_mode(hand)
        
        elif args[0] == 'scan':
            # 已在 connect 中掃描
            pass
        
        elif args[0] == 'open':
            hand.open_hand()
        
        elif args[0] == 'close':
            hand.close_hand()
        
        elif args[0] == 'wave':
            hand.wave()
        
        elif args[0] == 'move' and len(args) >= 3:
            sid = int(args[1])
            pos = int(args[2])
            hand.move(sid, pos)
        
        else:
            print("使用方式:")
            print("  sudo python3 amazinghand_test.py          # 互動模式")
            print("  sudo python3 amazinghand_test.py scan     # 掃描")
            print("  sudo python3 amazinghand_test.py open     # 張開")
            print("  sudo python3 amazinghand_test.py close    # 握緊")
            print("  sudo python3 amazinghand_test.py wave     # 揮手")
            print("  sudo python3 amazinghand_test.py move <ID> <位置>")
    
    finally:
        hand.disconnect()


if __name__ == "__main__":
    main()

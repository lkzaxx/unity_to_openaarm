#!/usr/bin/env python3
"""
平滑移動腳本 - 線性插值
用法: python3 smooth_move.py --start "[-0.13,0.05,...]" --end "[0.26,0.0,...]" --duration 2.0
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import argparse
import json
import time

class SmoothMover(Node):
    def __init__(self):
        super().__init__('smooth_mover')
        self.publisher = self.create_publisher(JointState, '/unity/joint_commands', 10)
        
    def move(self, start, end, duration, rate=50):
        """線性插值移動"""
        joint_names = ['R_J1', 'R_J2', 'R_J3', 'R_J4', 'R_J5', 'R_J6', 'R_J7']
        steps = int(duration * rate)
        dt = 1.0 / rate
        
        self.get_logger().info(f'平滑移動: {steps} 步, {duration}秒')
        
        for i in range(steps + 1):
            t = i / steps  # 0.0 ~ 1.0
            
            # 線性插值
            pos = [start[j] + t * (end[j] - start[j]) for j in range(7)]
            
            msg = JointState()
            msg.name = joint_names
            msg.position = pos
            self.publisher.publish(msg)
            
            time.sleep(dt)
        
        self.get_logger().info('移動完成')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--start', required=True, help='起始位置 JSON array')
    parser.add_argument('--end', required=True, help='目標位置 JSON array')
    parser.add_argument('--duration', type=float, default=2.0, help='移動時間(秒)')
    parser.add_argument('--rate', type=int, default=50, help='發送頻率(Hz)')
    args = parser.parse_args()
    
    start = json.loads(args.start)
    end = json.loads(args.end)
    
    rclpy.init()
    mover = SmoothMover()
    
    # 等待連線
    time.sleep(0.5)
    
    mover.move(start, end, args.duration, args.rate)
    
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

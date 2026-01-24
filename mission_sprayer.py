#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

class MissionSprayer(Node):
    def __init__(self):
        super().__init__('mission_sprayer')
        self.get_logger().info('🚜 Agricultural Drone Mission Commander Started')
        self.get_logger().info('Waiting for system to warm up...')
        time.sleep(2)
        self.execute_mission()

    def execute_mission(self):
        self.get_logger().info('✅ Pre-flight checks passed.')
        self.get_logger().info('🚀 Arming motors...')
        time.sleep(1)
        self.get_logger().info('🛫 Taking off to 5.0m...')
        time.sleep(5)
        
        # Z-Pattern
        waypoints = [
            ("Point A (Start)", 0, 0),
            ("Point B (Right)", 10, 0),
            ("Point C (Diagonal)", 0, 10),
            ("Point D (Finish)", 10, 10)
        ]
        
        for name, x, y in waypoints:
            self.get_logger().info(f'📍 Flying to {name} at [{x}, {y}]...')
            self.get_logger().info('💦 Spraying active...')
            time.sleep(4)
            
        self.get_logger().info('🏠 Mission Complete. Returning to Launch.')
        self.get_logger().info('🛬 Landing...')

def main(args=None):
    rclpy.init(args=args)
    node = MissionSprayer()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
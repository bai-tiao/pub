#!/usr/bin/env python3
"""
发布测试用的虚拟action_path
简短路径，适合小场地测试
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class VirtualPathPublisher(Node):
    def __init__(self):
        super().__init__('virtual_path_publisher')
        self.publisher = self.create_publisher(Path, '/action_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)  # 每秒发布一次
        
        # 声明参数
        self.declare_parameter('path_type', 'straight')  # straight, square, circle
        self.declare_parameter('path_length', 2.0)       # 路径长度(米)
        
        self.get_logger().info('Virtual Path Publisher Started')
        self.get_logger().info('Publishing to /action_path')
        
    def publish_path(self):
        path_type = self.get_parameter('path_type').get_parameter_value().string_value
        length = self.get_parameter('path_length').get_parameter_value().double_value
        
        if path_type == 'straight':
            path = self.create_straight_path(length)
        elif path_type == 'square':
            path = self.create_square_path(length)
        elif path_type == 'circle':
            path = self.create_circle_path(length)
        else:
            path = self.create_straight_path(length)
        
        self.publisher.publish(path)
        self.get_logger().info(f'Published {path_type} path with {len(path.poses)} points')
    
    def create_straight_path(self, length):
        """创建直线路径 - 从(0,0)到(length,0)"""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'odom'
        
        # 每20cm一个点
        num_points = int(length / 0.2) + 1
        for i in range(num_points):
            pose = PoseStamped()
            pose.header.stamp = path.header.stamp
            pose.header.frame_id = 'odom'
            pose.pose.position.x = i * 0.2
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        
        return path
    
    def create_square_path(self, side_length):
        """创建正方形路径 - 边长side_length"""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'odom'
        
        # 四个角点
        corners = [
            (0.0, 0.0),
            (side_length, 0.0),
            (side_length, side_length),
            (0.0, side_length),
            (0.0, 0.0)  # 回到起点
        ]
        
        # 在每条边上插入中间点
        for i in range(len(corners) - 1):
            x1, y1 = corners[i]
            x2, y2 = corners[i + 1]
            
            # 每20cm一个点
            dist = math.hypot(x2 - x1, y2 - y1)
            num_points = int(dist / 0.2) + 1
            
            for j in range(num_points):
                t = j / num_points
                pose = PoseStamped()
                pose.header.stamp = path.header.stamp
                pose.header.frame_id = 'odom'
                pose.pose.position.x = x1 + t * (x2 - x1)
                pose.pose.position.y = y1 + t * (y2 - y1)
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        
        return path
    
    def create_circle_path(self, radius):
        """创建圆形路径 - 半径radius，圆心在(radius, 0)"""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'odom'
        
        # 圆周上均匀分布点
        num_points = int(2 * math.pi * radius / 0.2)
        
        for i in range(num_points + 1):
            angle = 2 * math.pi * i / num_points
            pose = PoseStamped()
            pose.header.stamp = path.header.stamp
            pose.header.frame_id = 'odom'
            pose.pose.position.x = radius + radius * math.cos(angle)
            pose.pose.position.y = radius * math.sin(angle)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        
        return path

def main(args=None):
    rclpy.init(args=args)
    node = VirtualPathPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

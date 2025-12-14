#!/usr/bin/env python3
"""
位置环控制节点 (Position Loop Control)
使用P控制或PID控制，逐点到达目标
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist

def quaternion_to_yaw(q):
    """四元数转yaw角"""
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle(angle):
    """归一化角度到[-pi, pi]"""
    return (angle + math.pi) % (2 * math.pi) - math.pi

class PositionControl(Node):
    def __init__(self):
        super().__init__('position_control_node')
        
        # 声明参数
        self.declare_parameter('path_topic', '/action_path')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        
        # PID参数 - 线速度
        self.declare_parameter('kp_linear', 0.5)      # 线速度比例增益
        self.declare_parameter('ki_linear', 0.0)      # 线速度积分增益
        self.declare_parameter('kd_linear', 0.1)      # 线速度微分增益
        
        # PID参数 - 角速度
        self.declare_parameter('kp_angular', 2.0)     # 角速度比例增益
        self.declare_parameter('ki_angular', 0.0)     # 角速度积分增益
        self.declare_parameter('kd_angular', 0.2)     # 角速度微分增益
        
        # 控制参数
        self.declare_parameter('max_linear_vel', 0.3)   # 最大线速度
        self.declare_parameter('max_angular_vel', 1.5)  # 最大角速度
        self.declare_parameter('position_tolerance', 0.1)  # 位置到达容差(米)
        self.declare_parameter('yaw_tolerance', 0.1)       # 角度到达容差(弧度)
        self.declare_parameter('use_sequential_mode', True)  # 是否逐点到达
        
        # 状态变量
        self.path = []
        self.current_target_index = 0
        self.pos = None
        self.yaw = 0.0
        
        # PID积分和微分项
        self.linear_integral = 0.0
        self.linear_last_error = 0.0
        self.angular_integral = 0.0
        self.angular_last_error = 0.0
        self.last_time = self.get_clock().now()
        
        # 订阅和发布
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        
        self.create_subscription(Path, path_topic, self.path_cb, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 50)
        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        
        self.get_logger().info('Position Control Node Started')
        self.get_logger().info(f'Subscribing to: {path_topic}, {odom_topic}')
        self.get_logger().info(f'Publishing to: {cmd_topic}')

    def path_cb(self, msg):
        """接收路径回调"""
        if len(msg.poses) == 0:
            return
        
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.current_target_index = 0  # 重置目标索引
        
        # 重置PID积分项
        self.linear_integral = 0.0
        self.angular_integral = 0.0
        
        self.get_logger().info(f'Received path with {len(self.path)} points')

    def odom_cb(self, msg):
        """接收里程计回调"""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pos = (p.x, p.y)
        self.yaw = quaternion_to_yaw(q)
        self.control_loop()

    def control_loop(self):
        """位置环控制主循环"""
        if not self.path or self.pos is None:
            return
        
        # 获取参数
        use_sequential = self.get_parameter('use_sequential_mode').get_parameter_value().bool_value
        position_tol = self.get_parameter('position_tolerance').get_parameter_value().double_value
        
        # 选择目标点
        if use_sequential:
            # 逐点模式：到达当前目标后切换到下一个
            if self.current_target_index >= len(self.path):
                self.publish_stop()
                return
            target = self.path[self.current_target_index]
        else:
            # 直达模式：始终瞄准路径最后一个点
            target = self.path[-1]
        
        # 计算位置误差
        px, py = self.pos
        gx, gy = target
        dx = gx - px
        dy = gy - py
        distance = math.hypot(dx, dy)
        
        # 检查是否到达当前目标
        if distance < position_tol:
            if use_sequential:
                self.current_target_index += 1
                if self.current_target_index >= len(self.path):
                    self.get_logger().info('Reached final goal!')
                    self.publish_stop()
                    return
                else:
                    self.get_logger().info(f'Reached waypoint {self.current_target_index}, moving to next')
                    # 重置PID积分项
                    self.linear_integral = 0.0
                    self.angular_integral = 0.0
                    return
            else:
                self.get_logger().info('Reached goal!')
                self.publish_stop()
                return
        
        # 计算时间间隔
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0:
            dt = 0.01  # 防止除零
        
        # 计算目标角度和角度误差
        target_yaw = math.atan2(dy, dx)
        yaw_error = normalize_angle(target_yaw - self.yaw)
        
        # PID控制 - 线速度
        linear_vel = self.pid_control(
            error=distance,
            last_error=self.linear_last_error,
            integral=self.linear_integral,
            kp=self.get_parameter('kp_linear').get_parameter_value().double_value,
            ki=self.get_parameter('ki_linear').get_parameter_value().double_value,
            kd=self.get_parameter('kd_linear').get_parameter_value().double_value,
            dt=dt
        )
        self.linear_last_error = distance
        
        # PID控制 - 角速度
        angular_vel = self.pid_control(
            error=yaw_error,
            last_error=self.angular_last_error,
            integral=self.angular_integral,
            kp=self.get_parameter('kp_angular').get_parameter_value().double_value,
            ki=self.get_parameter('ki_angular').get_parameter_value().double_value,
            kd=self.get_parameter('kd_angular').get_parameter_value().double_value,
            dt=dt
        )
        self.angular_last_error = yaw_error
        
        # 当角度误差较大时，降低线速度（先转向再前进）
        if abs(yaw_error) > math.pi / 4:  # 45度
            linear_vel *= 0.3
        elif abs(yaw_error) > math.pi / 6:  # 30度
            linear_vel *= 0.6
        
        # 限制速度
        max_lin = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        max_ang = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        linear_vel = max(-max_lin, min(max_lin, linear_vel))
        angular_vel = max(-max_ang, min(max_ang, angular_vel))
        
        # 发布速度命令
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.pub.publish(twist)
        
        # 调试信息（降低频率）
        if self.get_clock().now().nanoseconds % 1000000000 < 50000000:  # 约20Hz
            self.get_logger().info(
                f'Target: ({gx:.2f}, {gy:.2f}), '
                f'Dist: {distance:.2f}m, Yaw err: {math.degrees(yaw_error):.1f}°, '
                f'Vel: ({linear_vel:.2f}, {angular_vel:.2f})'
            )

    def pid_control(self, error, last_error, integral, kp, ki, kd, dt):
        """PID控制器"""
        # 比例项
        p_term = kp * error
        
        # 积分项（防止积分饱和）
        integral += error * dt
        integral = max(-1.0, min(1.0, integral))  # 限制积分项
        i_term = ki * integral
        
        # 微分项
        derivative = (error - last_error) / dt if dt > 0 else 0.0
        d_term = kd * derivative
        
        return p_term + i_term + d_term

    def publish_stop(self):
        """发布停止命令"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PositionControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

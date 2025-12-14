#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist

def quaternion_to_yaw(q):
    x,y,z,w = q.x,q.y,q.z,q.w
    siny_cosp = 2.0*(w*z + x*y)
    cosy_cosp = 1.0 - 2.0*(y*y + z*z)
    return math.atan2(siny_cosp, cosy_cosp)

class PathToTwist(Node):
    def __init__(self):
        super().__init__('path_to_twist')
        self.declare_parameter('path_topic','/action_path')
        self.declare_parameter('odom_topic','/odom')
        self.declare_parameter('cmd_topic','/cmd_vel')
        self.declare_parameter('lookahead',0.6)
        self.declare_parameter('max_lin_vel',0.25)
        self.declare_parameter('max_ang_vel',1.0)
        self.path = []
        self.pos = None
        self.yaw = 0.0

        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value

        self.create_subscription(Path, path_topic, self.path_cb, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 50)
        self.pub = self.create_publisher(Twist, cmd_topic, 10)

    def path_cb(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.get_logger().info(f"Got path len={len(self.path)}")

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pos = (p.x, p.y)
        self.yaw = quaternion_to_yaw(q)
        self.follow()

    def follow(self):
        if not self.path or self.pos is None:
            return
        px,py = self.pos
        # 找第一个大于 lookahead 的点
        lookahead = self.get_parameter('lookahead').get_parameter_value().double_value
        goal = None
        for gx,gy in self.path:
            if math.hypot(gx-px, gy-py) >= lookahead:
                goal = (gx,gy); break
        if goal is None:
            goal = self.path[-1]
        gx,gy = goal
        ang = math.atan2(gy-py, gx-px)
        ang_err = (ang - self.yaw + math.pi)%(2*math.pi) - math.pi
        dist = math.hypot(gx-px, gy-py)
        max_lin = self.get_parameter('max_lin_vel').get_parameter_value().double_value
        max_ang = self.get_parameter('max_ang_vel').get_parameter_value().double_value
        lin = max_lin * max(0.0, 1.0 - abs(ang_err)/math.pi)
        ang_v = max(-max_ang, min(max_ang, 1.8*ang_err))
        t = Twist()
        t.linear.x = lin if dist>0.15 else 0.0
        t.angular.z = ang_v
        self.pub.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = PathToTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

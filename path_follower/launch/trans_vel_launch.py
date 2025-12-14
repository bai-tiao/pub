#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ===========================================
    # 🔧 Odom话题重映射解决方案
    # ===========================================
    # 
    # 问题分析:
    #   runner_ros(DualMap) 和 base_controller 都发布 /odom
    #   → 造成话题冲突,数据混乱
    #   → 导致车辆位置错乱,可能在墙里
    # 
    # 解决方案:
    #   方案A: trans_vel直接使用 /odom (来自base_controller的车轮odom)
    #          - 优点: 高频率(50-100Hz),低延迟,适合控制
    #          - 缺点: 如果runner_ros也在发布,会冲突
    #   
    #   方案B: trans_vel使用 /wheel_odom
    #          - 需要base_controller改成发布到 /wheel_odom
    #          - 或者在base_controller启动时重映射
    # 
    # 当前配置: 方案A (直接使用/odom)
    # 如果遇到冲突,请:
    #   1. 确保runner_ros发布到 /visual_odom 而不是 /odom
    #   2. 或者停止runner_ros的odom发布
    #   3. 或者使用下面注释的方案B配置
    # ===========================================
    
    # ===========================================
    # 🎯 Odom源选择 (根据你的情况选择)
    # ===========================================
    
    # 方案A: 使用车轮odom (当前推荐 - 今天测试)
    # 适用: 只有base_controller发布/odom
    remappings = []
    
    # 方案B: 使用相机的视觉odom (明天真实相机)
    # 适用: 相机发布到/camera_odom或/visual_odom
    # remappings = [
    #     ('/odom', '/camera_odom'),  # 或 '/visual_odom'
    # ]
    
    # 方案C: 使用DualMap的odom (如果DualMap发布odom)
    # 适用: runner_ros发布/odom,base_controller发布/wheel_odom
    # remappings = [
    #     ('/odom', '/odom'),  # 使用DualMap的odom
    # ]
    
    # 方案D: 明确指定车轮odom (如果有命名冲突)
    # 适用: 多个节点都发布odom,需要明确选择
    # remappings = [
    #     ('/odom', '/wheel_odom'),
    # ]
    # ===========================================
    
    params = [
        {'use_sim_time': True},           # 使用bag的仿真时间
        {'lookahead': 0.6},                # 前视距离(米)
        {'max_lin_vel': 0.25},             # 最大线速度(m/s)
        {'max_ang_vel': 1.0},              # 最大角速度(rad/s)
    ]
    
    node = Node(
        package='path_follower',
        executable='trans_vel',
        name='trans_vel_node',
        output='screen',
        parameters=params,
        remappings=remappings
    )
    return LaunchDescription([node])

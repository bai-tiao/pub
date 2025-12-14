from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    位置环控制Launch文件
    
    两种控制模式：
    1. use_sequential_mode=True  - 逐点到达模式（默认）
       车会依次到达路径上的每个点，到达一个点后再走向下一个
       
    2. use_sequential_mode=False - 直达模式
       车会直接朝向路径的最后一个点（目标点）移动
    
    参数调节指南：
    - kp_linear: 增大加快靠近速度，过大会震荡
    - kp_angular: 增大加快转向，过大会左右摆动
    - position_tolerance: 到达目标的距离容差（米）
    - max_linear_vel: 最大前进速度
    - max_angular_vel: 最大转向速度
    """
    
    return LaunchDescription([
        Node(
            package='path_follower',
            executable='position_control',
            name='position_control_node',
            output='screen',
            parameters=[{
                'path_topic': '/action_path',
                'odom_topic': '/odom',
                'cmd_topic': '/cmd_vel',
                
                # PID参数 - 线速度控制
                'kp_linear': 0.5,    # 比例增益，影响靠近速度
                'ki_linear': 0.0,    # 积分增益，消除稳态误差（通常保持0）
                'kd_linear': 0.1,    # 微分增益，减少超调
                
                # PID参数 - 角速度控制
                'kp_angular': 2.0,   # 比例增益，影响转向速度
                'ki_angular': 0.0,   # 积分增益（通常保持0）
                'kd_angular': 0.2,   # 微分增益，减少摆动
                
                # 速度限制
                'max_linear_vel': 0.3,    # 最大前进速度（米/秒）
                'max_angular_vel': 1.5,   # 最大角速度（弧度/秒）
                
                # 到达判定
                'position_tolerance': 0.1,  # 位置容差（米）
                'yaw_tolerance': 0.1,       # 角度容差（弧度，约5.7度）
                
                # 控制模式
                'use_sequential_mode': True,  # True=逐点到达, False=直达终点
                
                # 时间同步
                'use_sim_time': True,  # 使用bag时间
            }],
        ),
    ])

# 系统架构

```
数据流:
bag/camera → DualMap → /action_path → Path_Follower → /cmd_vel 
                                                         ↓
                                              /velocity_smoother/input
                                                         ↓
                                              velocity_smoother (平滑加速度)
                                                         ↓
                                              /smoother_cmd_vel
                                                         ↓
                                                    driver.py
                                                         ↓
                                                    Arduino
                                                         ↓
                                                    /odom ← 回路
```

## 控制器选择

1. **trans_vel** (Pure Pursuit路径跟踪)
   - 平滑跟踪曲线
   - 适合开阔环境、高速

2. **position_control** (PID位置环)
   - 精确到达目标点
   - 适合窄道、障碍物多

## 话题连接

- Path Follower发布: `/cmd_vel` 
- Remapping到: `/velocity_smoother/input`(now is /cmd_vel)
- Velocity Smoother发布: `/smoother_cmd_vel`(now not use)
- Driver订阅: `smoother_cmd_vel`(now /cmd_vel)
- Driver发布: `/odom`(not choosing use /odom from driver or camero now)

## 关键问题

1. **DualMap路径发布机制**
   - DualMap只在点击目标点时发布一次路径
   - 不会持续发布
   - position_control需要先收到一次路径，然后依赖/odom持续控制

2. **数据流要求**
   - position_control需要同时收到 /action_path 和 /odom
   - 缺少任何一个都不会发布 /cmd_vel

3. **DualMap自动关闭**
   - 当bag播放完或无新数据时，DualMap会shutdown
   - 需要重启bag (--loop) 和 DualMap

## 启动

```bash
./README.sh  # 查看启动命令
```

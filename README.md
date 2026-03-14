# Fuel 
## 🚀 使用方法

### 快速启动

#### 终端 1：启动 MAVROS fast-lio livox_ros_driver2 lio_to_mavros

```bash
roslaunch livox_ros_driver2 msg_Mid360.launch

roslaunch fast_lio mapping.launch

roslaunch mavros px4.launch

roslaunch lio_to_mavros lio_to_mavros.launch
```

#### 终端 2：启动 FUEL

```bash
roslaunch exploration_manager exploration.launch
```

#### 终端 3：启动轨迹跟踪控制器

```bash
rosrun fuel_planner fuel_planner.launch
```
## 反馈

若有问题 发邮件到256395180@qq.com

# Fast-LIVO2 Camera_Init坐标系旋转修改文档

## 📋 项目概述

**项目名称：** Fast-LIVO2 SLAM系统  
**修改目标：** 实现camera_init坐标系绕y轴向下旋转30度  
**修改日期：** 2025年1月27日  
**修改内容：** 坐标系变换与重力对齐功能增强  

## 🎯 需求分析

### 原始需求
- camera_init坐标系需要向下旋转30度
- 实现俯视角度的SLAM系统
- 保持系统稳定性和数据一致性

### 技术要求
- 绕y轴旋转30度（π/6弧度）
- 在重力对齐后应用变换
- 影响所有状态变量（位置、姿态、速度、重力）

## 🔧 详细修改内容

### 1. 核心算法修改

**修改文件：** `src/fast-livo2/src/LIVMapper.cpp`  
**修改函数：** `gravityAlignment()`  
**修改行数：** 306-323行

#### 原始代码：
```cpp
void LIVMapper::gravityAlignment() 
{
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Eigen::Quaterniond G_q_I0 = Eigen::Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}
```

#### 修改后代码：
```cpp
void LIVMapper::gravityAlignment() 
{
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Eigen::Quaterniond G_q_I0 = Eigen::Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    // 应用重力对齐
    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    
    // 添加camera_init坐标系绕y轴向下旋转30度的变换
    double rotation_angle = 30.0 * M_PI / 180.0; // 30度转弧度
    M3D R_camera_init;
    R_camera_init << cos(rotation_angle), 0, sin(rotation_angle),
                     0, 1, 0,
                     -sin(rotation_angle), 0, cos(rotation_angle);
    
    // 应用额外的旋转变换
    _state.pos_end = R_camera_init * _state.pos_end;
    _state.rot_end = R_camera_init * _state.rot_end;
    _state.vel_end = R_camera_init * _state.vel_end;
    _state.gravity = R_camera_init * _state.gravity;
    
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished with 30 degree Y-axis rotation" << std::endl;
  }
}
```

### 2. 配置文件修改

**修改文件：** `src/fast-livo2/config/mid360_realsense.yaml`  
**修改位置：** uav参数段

#### 原始配置：
```yaml
uav:
  imu_rate_odom: false
  gravity_align_en: false
```

#### 修改后配置：
```yaml
uav:
  imu_rate_odom: false
  gravity_align_en: true
```

**修改说明：** 启用重力对齐功能，使得camera_init坐标系旋转生效。

## 📊 数学原理

### 旋转矩阵推导

绕y轴旋转30度的旋转矩阵：

```
θ = 30° = π/6 弧度

R_y(30°) = [cos(30°)  0  sin(30°)]   [0.866  0   0.5 ]
           [   0      1     0    ] = [ 0    1    0  ]
           [-sin(30°) 0  cos(30°)]   [-0.5  0  0.866]
```

### 坐标变换公式

对于任意点 P = [x, y, z]，变换后的坐标为：

```
P' = R_y(30°) × P

[x']   [0.866  0   0.5 ] [x]   [0.866x + 0.5z]
[y'] = [ 0    1    0  ] [y] = [      y      ]
[z']   [-0.5  0  0.866] [z]   [-0.5x + 0.866z]
```

## 🏗️ 系统架构分析

### Fast-LIVO2 核心组件

1. **LiDAR处理模块：** 点云预处理和特征提取
2. **IMU处理模块：** 惯性导航和状态预测
3. **视觉处理模块：** 直接法视觉里程计
4. **状态估计模块：** 多传感器融合EKF
5. **地图管理模块：** Voxel Map构建和维护

### 坐标系定义

- **camera_init：** 系统全局参考坐标系
- **IMU坐标系：** 惯性测量单元本体坐标系
- **LiDAR坐标系：** 激光雷达本体坐标系
- **相机坐标系：** 视觉传感器坐标系

### 修改影响范围

修改主要影响以下数据流：

1. **状态向量：** 位置、姿态、速度
2. **发布数据：** 里程计、轨迹、点云
3. **坐标变换：** TF树中的camera_init到其他坐标系变换

## 🚀 编译和部署

### 编译

```bash
 colcon build --packages-select fast_livo 
```

### 运行命令

```bash
# 启动SLAM系统

chong xin qi dong lidar node
ros2 launch fast_livo2 mapping_mid360_realsense.launch.py
```



## 📝 注意事项

### 重要提醒

1. **角度调整：** 修改 `rotation_angle` 变量可调整旋转角度
2. **坐标一致性：** 确保所有传感器外参在同一坐标系下标定
3. **系统稳定性：** 旋转变换在重力对齐后应用，保证系统稳定

### 扩展可能

- **可配置角度：** 将旋转角度设为配置参数
- **多轴旋转：** 支持绕多个轴的复合旋转
- **动态调整：** 运行时动态调整坐标系方向

## 📚 参考资料

- [Fast-LIVO2 官方仓库](https://github.com/hku-mars/Fast-LIVO2)
- [ROS2 TF2 文档](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [Eigen 矩阵库文档](https://eigen.tuxfamily.org/dox/)

---

**文档版本：** v1.0  
**最后更新：** 2025年1月27日  
**维护者：** SLAM开发团队 
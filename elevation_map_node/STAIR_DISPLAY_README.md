# 楼梯显示优化说明

## 问题描述

原始的高程图显示存在以下问题：
1. **斜面过渡问题**：台阶之间显示为平滑的斜面，而不是清晰的台阶边界
2. **高程平均化**：多个不同高度的点被平均化，使台阶边界模糊
3. **插值伪影**：可视化系统的自动插值导致楼梯看起来不准确

## 解决方案

### 1. 可视化层面的改进 (`height_map_plot2.py`)

#### 新增功能：
- **楼梯模式**：专门针对楼梯结构的显示模式
- **高程离散化**：将相近的高程值归并到相同的台阶级别
- **阶梯式表面**：关闭平滑插值，使用网格边缘显示
- **体素化显示**：可选的立方体显示模式

#### 关键参数：
```python
self.stair_mode = True              # 启用楼梯显示模式
self.step_threshold = 0.05          # 台阶高度阈值（5cm）
self.use_stepped_surface = True     # 使用阶梯式表面
self.voxel_mode = False            # 体素化显示模式
```

#### 使用方法：
```python
# 启用楼梯模式
plotter.set_stair_mode(enabled=True, step_threshold=0.05, use_stepped_surface=True)

# 启用体素化显示
plotter.voxel_mode = True
```

### 2. 数据处理层面的改进 (`elevation_map_generator5.py`)

#### 新增功能：
- **楼梯检测算法**：自动识别网格是否包含楼梯结构
- **众数计算**：使用最常出现的高度值而不是平均值
- **主导高程**：为楼梯网格计算代表性高程值
- **历史数据追踪**：维护高程历史用于楼梯检测

#### 楼梯检测逻辑：
1. 维护每个网格的高程历史（最近10个点）
2. 检测是否存在多个明显的高度级别
3. 如果检测到楼梯，计算主导高程（众数）
4. 在地图生成时使用主导高程而不是平均值

#### 配置参数：
```python
self.declare_parameter('stair_detection_enabled', True)  # 启用楼梯检测
self.declare_parameter('stair_step_threshold', 0.05)    # 楼梯台阶高度阈值
```

### 3. GlobalGridCell 类的增强

#### 新增属性：
```python
self.elevation_history = []         # 高程历史记录
self.dominant_elevation = 0.0       # 主导高程值
self.is_stair_cell = False         # 楼梯网格标记
```

#### 新增方法：
- `_update_stair_detection()`: 更新楼梯检测状态
- `_calculate_mode_elevation()`: 计算高程众数
- `get_representative_elevation()`: 获取代表性高程值

## 使用指南

### 1. 基本配置

在启动节点时，可以通过参数配置楼梯检测：

```bash
ros2 run elevation_map_node elevation_map_generator5 \
  --ros-args \
  -p stair_detection_enabled:=true \
  -p stair_step_threshold:=0.05
```

### 2. 可视化模式选择

#### 阶梯式表面模式（推荐）：
- 保持表面连续性
- 清晰显示台阶边界
- 关闭平滑插值

#### 体素化模式：
- 将每个网格显示为立方体
- 最直观的楼梯显示
- 适合小范围、高分辨率地图

### 3. 参数调优

#### `stair_step_threshold`（台阶阈值）：
- **0.03m**：适合室内精细楼梯
- **0.05m**：适合一般楼梯（推荐）
- **0.08m**：适合粗糙环境或户外台阶

#### 可视化参数：
```python
# 在 height_map_plot2.py 中调整
self.step_threshold = 0.05          # 与数据处理保持一致
rstride=1, cstride=1               # 每个网格都显示
edgecolor='gray'                   # 显示网格边缘
shade=False                        # 关闭光影效果
```

## 效果对比

### 修改前：
- 楼梯显示为平滑斜面
- 台阶边界模糊不清
- 高程值被平均化

### 修改后：
- 清晰的台阶边界
- 离散化的高程级别
- 保持楼梯的几何特征

## 注意事项

1. **计算性能**：楼梯检测会增加一定的计算开销
2. **内存使用**：每个网格保存历史数据会增加内存使用
3. **参数敏感性**：台阶阈值需要根据实际环境调整
4. **数据质量**：需要足够的点云数据才能准确检测楼梯

## 故障排除

### 楼梯仍显示为斜面：
1. 检查 `stair_detection_enabled` 参数是否为 true
2. 调整 `stair_step_threshold` 参数
3. 确保点云数据质量足够

### 显示过于粗糙：
1. 减小 `step_threshold` 值
2. 增加网格分辨率
3. 考虑使用传统平滑模式

### 性能问题：
1. 增加 `min_points_per_cell` 阈值
2. 减少 `history_limit` 值
3. 关闭楼梯检测功能

## 扩展功能

未来可以考虑添加：
- 自适应阈值调整
- 楼梯方向检测
- 更复杂的聚类算法
- 实时参数调整界面 
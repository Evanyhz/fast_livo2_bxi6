import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from threading import Thread
import cv2
import datetime
import os

class HeightMapPlotter:
    def __init__(self, len_x, len_y, dx, dt, title="Height Map"):

        self.dx = dx
        self.title = title
        
        # 创建网格
        self.len_x = len_x
        self.len_y = len_y
        self.x = np.arange(0, self.len_x * dx, dx)
        self.y = np.arange(0, self.len_y * dx, dx)
        self.X, self.Y = np.meshgrid(self.x, self.y)
        self.height_map = np.zeros_like(self.X)
        
        # 机器人位姿信息（用于显示坐标系）
        self.robot_pose = None  # 格式: {'x': x, 'y': y, 'z': z, 'yaw': yaw}
        self.show_robot_frame = True  # 是否显示机器人坐标系
        self.axis_length = 0.5  # 坐标轴长度（米）
        
        # 高程基准信息
        self.elevation_reference = None  # 高程基准点（绝对值）
        self.show_reference_info = True  # 是否显示基准点信息
        
        self.refresh_freq = 25  # 绘图器窗口刷新频率
        
        self.init_3d_graph()
        
        self.fig.canvas.draw()  # 确保图形已渲染
        width, height = self.fig.canvas.get_width_height()
        image_array = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype=np.uint8)
        image_array = image_array.reshape(height, width, 3)

        # 定义视频编码器和输出视频对象
        current_time=datetime.datetime.now()
        formatted_datetime = current_time.strftime('%m%d_%H%M%S')
        output_video_dir = os.path.join(os.path.dirname(__file__),'video')
        os.makedirs(output_video_dir,exist_ok=True)
        output_video_filename = '{}.mp4'.format(formatted_datetime)
        output_video_fullpath = os.path.join(output_video_dir, output_video_filename)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 使用 mp4v 编码（H.264）
        self.video = cv2.VideoWriter(output_video_fullpath, fourcc, int(1/dt), (width, height))  # 50 FPS

    def init_3d_graph(self):
        """初始化3D图形"""
        self.fig = plt.figure(figsize=(10, 7))
        self.fig.canvas.manager.set_window_title(self.title)  # 设置窗口标题
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self.surf = self.ax.plot_surface(self.X, self.Y, self.height_map, 
                                       cmap='terrain', 
                                       edgecolor='none',
                                       antialiased=False)
        
        # 设置坐标轴标签
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Height (m, relative to lowest point)')
        
        # 设置视角
        self.ax.view_init(elev=45, azim=45)
        self.fig.colorbar(self.surf, shrink=0.5, aspect=5)
        
    def update_plot(self):
        """更新绘图"""
        self.ax.clear()

        # 重新绘制表面图
        print(f"{self.title} shape: {self.height_map.shape}")
        
        # 检查数据有效性
        finite_data = self.height_map[np.isfinite(self.height_map)]
        nonzero_finite_data = finite_data[finite_data != 0]
        
        print(f"{self.title} data analysis:")
        print(f"  Total points: {self.height_map.size}")
        print(f"  Finite points: {len(finite_data)}")
        print(f"  Non-zero finite points: {len(nonzero_finite_data)}")
        if len(finite_data) > 0:
            print(f"  Finite range: [{np.min(finite_data):.6f}, {np.max(finite_data):.6f}]")
        if len(nonzero_finite_data) > 0:
            print(f"  Non-zero range: [{np.min(nonzero_finite_data):.6f}, {np.max(nonzero_finite_data):.6f}]")
        
        # 更精确的有效性检查：如果有任何非零的有限数据，就认为是有效的
        if len(nonzero_finite_data) == 0:
            print(f"Warning: {self.title} has no valid non-zero data")
            # 但仍然尝试绘制，可能有接近0的有效数据
        
        plot_data = self.height_map.copy()
        # 将-inf和nan替换为0用于绘图
        plot_data[~np.isfinite(plot_data)] = 0
            
        # 检查维度匹配
        if plot_data.shape != self.X.shape:
            print(f"Dimension mismatch: plot_data {plot_data.shape} vs meshgrid {self.X.shape}")
            # 如果维度不匹配，尝试转置
            if plot_data.shape == (self.X.shape[1], self.X.shape[0]):
                plot_data = plot_data.T
                print(f"Transposed plot_data to {plot_data.shape}")
            else:
                print(f"Cannot fix dimension mismatch, using zeros")
                plot_data = np.zeros_like(self.X)
        
        self.surf = self.ax.plot_surface(self.X, self.Y, plot_data, 
                                        cmap='terrain', 
                                        edgecolor='none',
                                        antialiased=False)
        
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Height (m, relative to lowest point)')
        
        # 自动计算Z轴范围
        finite_data = plot_data[np.isfinite(plot_data)]
        if len(finite_data) > 0:
            z_min = np.min(finite_data)
            z_max = np.max(finite_data)
            z_range = z_max - z_min
            if z_range > 0:
                margin = z_range * 0.1  # 10%边距
                self.ax.set_zlim(z_min - margin, z_max + margin)
                print(f"{self.title} Z range: [{z_min:.3f}, {z_max:.3f}]")
            else:
                # 数据都相同，使用小范围
                self.ax.set_zlim(z_min - 0.1, z_min + 0.1)
        else:
            # 没有有限数据，使用默认范围
            self.ax.set_zlim(-1.0, 1.0)
        
        # 绘制机器人坐标系
        if self.show_robot_frame and self.robot_pose is not None:
            self.draw_robot_frame()
        
        # 设置坐标轴等比例显示
        self.ax.set_box_aspect([1,1,1])  # 强制等比例显示
        
        # 显示基准点信息
        if self.show_reference_info and self.elevation_reference is not None:
            self.ax.text2D(0.02, 0.98, f'Lowest point: {self.elevation_reference:.3f}m', 
                          transform=self.ax.transAxes, fontsize=10, 
                          bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
                          verticalalignment='top')
        
        # 实时显示
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)  # 短暂暂停以更新显示
        
        # 录制视频
        if hasattr(self, 'video') and self.video is not None:
            width, height = self.fig.canvas.get_width_height()
            image_array = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype=np.uint8)
            image_array = image_array.reshape(height, width, 3)
            self.video.write(image_array)
    
    def set_robot_pose(self, x, y, z, yaw):
        """
        设置机器人位姿信息
        
        Args:
            x (float): 机器人X坐标（米）
            y (float): 机器人Y坐标（米）
            z (float): 机器人Z坐标（米）
            yaw (float): 机器人yaw角（弧度）
        """
        self.robot_pose = {
            'x': x,
            'y': y, 
            'z': z,
            'yaw': yaw
        }
    
    def set_show_robot_frame(self, show):
        """设置是否显示机器人坐标系"""
        self.show_robot_frame = show
    
    def set_axis_length(self, length):
        """设置坐标轴长度"""
        self.axis_length = length
    
    def set_elevation_reference(self, reference_elevation):
        """
        设置高程基准点
        
        Args:
            reference_elevation (float): 基准点的绝对高程值
        """
        self.elevation_reference = reference_elevation
    
    def draw_robot_frame(self):
        """
        绘制机器人坐标系
        
        绘制XYZ坐标轴：
        - X轴（红色）：机器人前进方向
        - Y轴（绿色）：机器人左侧方向  
        - Z轴（蓝色）：机器人上方方向
        """
        if self.robot_pose is None:
            return
            
        # 机器人位置
        robot_x = self.robot_pose['x']
        robot_y = self.robot_pose['y'] 
        robot_z = self.robot_pose['z']
        yaw = self.robot_pose['yaw']
        
        # 计算坐标轴端点
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        
        # X轴（红色）- 机器人前进方向
        x_end = robot_x + self.axis_length * cos_yaw
        y_end = robot_y + self.axis_length * sin_yaw
        self.ax.plot([robot_x, x_end], [robot_y, y_end], [robot_z, robot_z], 
                     'r-', linewidth=4, label='X-axis (Forward)', alpha=1.0, zorder=10)
        
        # Y轴（绿色）- 机器人左侧方向（垂直于X轴）
        y_x_end = robot_x + self.axis_length * (-sin_yaw)  # 左转90度
        y_y_end = robot_y + self.axis_length * cos_yaw
        self.ax.plot([robot_x, y_x_end], [robot_y, y_y_end], [robot_z, robot_z], 
                     'g-', linewidth=4, label='Y-axis (Left)', alpha=1.0, zorder=10)
        
        # Z轴（蓝色）- 机器人上方向
        z_end = robot_z + self.axis_length  # Z轴与其他轴长度一致
        self.ax.plot([robot_x, robot_x], [robot_y, robot_y], [robot_z, z_end], 
                     'b-', linewidth=4, label='Z-axis (Up)', alpha=1.0, zorder=10)
        
        # 调试信息：打印实际坐标轴长度（仅在必要时）
        if hasattr(self, '_debug_axis_length') and self._debug_axis_length:
            x_length = np.sqrt((x_end - robot_x)**2 + (y_end - robot_y)**2)
            y_length = np.sqrt((y_x_end - robot_x)**2 + (y_y_end - robot_y)**2)
            z_length = abs(z_end - robot_z)
            print(f"{self.title} axis lengths - X: {x_length:.3f}, Y: {y_length:.3f}, Z: {z_length:.3f}, target: {self.axis_length:.3f}")
        
        # 在机器人位置绘制一个更明显的标记点
        self.ax.scatter([robot_x], [robot_y], [robot_z], 
                       c='black', s=200, marker='o', label='Robot', 
                       edgecolors='white', linewidth=2, alpha=1.0, zorder=11)
        
        # 添加文本标签，位置稍微抬高以确保可见
        text_height = robot_z + self.axis_length * 0.2
        self.ax.text(robot_x, robot_y, text_height, 'Robot', 
                    fontsize=10, ha='center', weight='bold',
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7))
        
        # 绘制箭头（可选，使坐标轴更明显）
        # 动态调整箭头大小，确保在不同视角下都可见
        arrow_scale = self.axis_length * 0.3  # 箭头大小与坐标轴长度成比例
        
        # X轴箭头（红色）- 更粗的线条
        self.ax.quiver(robot_x, robot_y, robot_z,
                      cos_yaw * arrow_scale, sin_yaw * arrow_scale, 0,
                      color='red', arrow_length_ratio=0.2, linewidth=3, alpha=1.0, zorder=12)
        
        # Y轴箭头（绿色）- 更粗的线条
        self.ax.quiver(robot_x, robot_y, robot_z,
                      -sin_yaw * arrow_scale, cos_yaw * arrow_scale, 0,
                      color='green', arrow_length_ratio=0.2, linewidth=3, alpha=1.0, zorder=12)
        
        # Z轴箭头（蓝色）- 与其他轴箭头长度一致
        self.ax.quiver(robot_x, robot_y, robot_z,
                      0, 0, arrow_scale,  # Z轴箭头与其他轴一致
                      color='blue', arrow_length_ratio=0.2, linewidth=3, alpha=1.0, zorder=12)

if __name__ == "__main__":
    # 示例：创建一个随机高度图
    len_x, len_y = 50, 50
    dx = 0.1  # 10cm分辨率
    
    # 生成一些有趣的地形
    x = np.linspace(-2, 2, len_x)
    y = np.linspace(-2, 2, len_y)
    X, Y = np.meshgrid(x, y)
    height_map = np.sin(X**2 + Y**2) / (X**2 + Y**2 + 0.1) * 2  # 二维sinc函数
    
    # 创建绘图器
    plotter = HeightMapPlotter(len_x, len_y, dx)
    
    # 创建动画
    ani = FuncAnimation(fig=plotter.fig,
                        func=plotter.update_plot,
                        frames=360,
                        interval=1000/plotter.refresh_freq,
                        blit=False)
    
    plt.tight_layout()
    plt.show()
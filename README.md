# 六轴机械臂仿真控制系统

基于 ROS 的六轴机械臂仿真、规划与控制系统，支持轨迹规划、虚拟执行、串口命令发送与状态反馈。

## 功能特点

- **MoveIt 集成**：利用 MoveIt 进行运动规划与轨迹生成
- **串口通信**：支持与实体机械臂的串口通信
- **虚拟反馈**：提供状态反馈模拟，便于系统测试
- **GUI 调试工具**：直观的可视化关节控制与监控界面
- **关节顺序自适应**：解决不同组件间关节顺序不一致的问题

## 主要组件

### 1. 仿真与规划

- URDF 模型：详细定义机械臂结构、关节限位与运动学参数
- MoveIt 配置：优化的规划与碰撞检测设置
- RViz 可视化：实时显示规划轨迹与执行状态

### 2. 反馈模拟

- `fake_feedback.py`：模拟机械臂状态反馈，便于无硬件测试
- 自动关节顺序映射：处理 MoveIt 与控制系统间的关节顺序差异
- 串口数据格式模拟：符合实际硬件通信协议

### 3. GUI 调试工具

- `gui_feedback.py`：图形界面控制与监控工具
- 关节滑动条控制：直观调整各关节位置
- 位置数组输入：支持直接输入或粘贴关节位置数组
- 监控面板：显示命令传输与数据映射情况

### 4. 通信与控制

- 轨迹处理器：将 MoveIt 规划轨迹转换为控制命令
- 串口通信：基于 ROS 串口节点的通信实现
- 状态同步：保持规划状态与反馈状态的一致性

## 使用方法

### 1. 启动基本测试

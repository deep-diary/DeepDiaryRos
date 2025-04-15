#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Protocol Handlers
Manage different protocol processors 
"""

import os
import sys

# 添加导入路径修复
# current_dir = os.path.dirname(os.path.abspath(__file__))
# if current_dir not in sys.path:
#     sys.path.insert(0, current_dir)

# 现在尝试导入
try:
    from protocol_processing.deep_arm.protocol import DeepArmProtocol
    print("Successfully imported DeepArmProtocol")
except ImportError as e:
    print("Error importing DeepArmProtocol: {}".format(e))
    # 尝试诊断问题
    print("\nDiagnosing import paths:")
    print("Python version: {}".format(sys.version))
    print("Current directory: {}".format(os.getcwd()))
    print("Python path:")
    for p in sys.path:
        print("  - {}".format(p))
    raise

import struct
import logging

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    stream=sys.stdout
)
logger = logging.getLogger('protocol_handler')

class ProtocolHandler:
    """协议处理器基类，定义了所有协议处理器必须实现的接口"""
    
    def __init__(self, config):
        """
        初始化协议处理器
        
        Args:
            config: 包含协议配置的字典
        """
        self.config = config
        # 初始化协议
        self.protocol = DeepArmProtocol()
    
    def format_trajectory_point(self, point):
        """
        将轨迹点格式化为串口协议格式
        
        Args:
            point: 包含位置、速度等信息的字典
            
        Returns:
            bytearray: 格式化后的数据
        """
        raise NotImplementedError("This method must be implemented by subclasses")
    
    def get_joint_mapping(self):
        """获取关节名称到ID的映射关系"""
        joint_names = self.config['arm']['joint_names']
        joint_ids = self.config['arm'].get('joint_ids', list(range(1, len(joint_names) + 1)))
        
        # 创建映射字典
        return dict(zip(joint_names, joint_ids))


class URDF_armv001_ProtocolHandler(ProtocolHandler):
    """URDF_armv001机械臂的协议处理器"""
    
    def format_trajectory_point(self, point):
        """格式化轨迹点为串口协议格式"""
        # joint_names: [fifth_joint, first_joint, fourth_joint, second_joint, sixth_joint, third_joint]
        # points: 
        # - 
        #     positions: [-0.0219647045875146, -0.01720273585453402, 0.008310587956624767, -0.04210852333746961, 0.01214546902507518, 0.0383061121936588]
        #     velocities: [-0.12879587499164882, -0.10617575295588436, 0.043764184375565875, -0.2444535451735695, 0.07358888776684064, 0.22839065168094047]
        #     accelerations: [-0.5164358359214164, -0.425735402907223, 0.17548227490111648, -0.9801911043645757, 0.2950710864836303, 0.9157833441880917]
        #     effort: []
        #     time_from_start: 
        #     secs: 0
        #     nsecs: 288975049
            # # 构造轨迹点数据
            # point_data = {
            #     'joint_names': joint_names,  # 添加关节名称
            #     'time_from_start': point.time_from_start,
            #     'positions': point.positions,
            #     'velocities': point.velocities if point.velocities else [0.0] * len(point.positions)
            # }
    
        # # 获取关节ID映射
        # joint_mapping = self.get_joint_mapping()
        
        # # 获取传入的关节名称和顺序（如果有）
        # joint_names = point.get('joint_names', self.config['arm']['joint_names'])
        
        # 提取位置和速度
        positions = point['positions']
        velocities = point['velocities']
        ids = [5, 1, 4, 2, 6, 3]
        
        frame = self.protocol.create_motor_pos_frame_all(ids, positions)
        # 将列表frame展开
        frame = [item for sublist in frame for item in sublist]
        
        logger.info("Generated data packet: {}".format([b for b in frame]))
        return frame

class DefaultProtocolHandler(ProtocolHandler):
    """默认协议处理器，当未找到特定的处理器时使用"""
    
    def format_trajectory_point(self, point):
        """
        默认的轨迹点格式化方法，尽量兼容URDF_armv001
        """
        logger.warning("Using default protocol handler, may not be compatible with your arm!")
        
        # 获取关节ID映射
        joint_mapping = self.get_joint_mapping()
        
        # 获取传入的关节名称和顺序（如果有）
        joint_names = point.get('joint_names', self.config['arm']['joint_names'])
        
        # 提取位置和速度
        positions = point['positions']
        velocities = point['velocities']
        
        # 关节数量
        joint_count = len(positions)
        
        # 关节数据，格式为 [(joint_id, position, velocity), ...]
        joint_data = []
        
        # 按joint_names的顺序获取对应的数据con
        if 'joint_names' in point:
            for i, name in enumerate(joint_names):
                joint_id = joint_mapping.get(name, i+1)
                joint_data.append((joint_id, positions[i], velocities[i]))
        else:
            for i, name in enumerate(self.config['arm']['joint_names']):
                joint_id = joint_mapping.get(name, i+1)
                if i < len(positions):
                    joint_data.append((joint_id, positions[i], velocities[i]))
        
        # 尝试使用配置中的协议参数
        START_MARKER = self.config.get('protocol', {}).get('start_marker', 0xFF)
        CMD_TYPE = self.config.get('protocol', {}).get('cmd_type', 0x01)
        END_MARKER = self.config.get('protocol', {}).get('end_marker', 0xFE)
        
        # 构建数据包
        data = bytearray()
        data.append(START_MARKER)  # 起始标记
        data.append(CMD_TYPE)      # 命令类型
        data.append(joint_count)   # 关节数量
        
        # 添加每个关节的ID、位置和速度
        for joint_id, position, velocity in joint_data:
            data.append(joint_id)
            pos_bytes = struct.pack('<f', position)
            vel_bytes = struct.pack('<f', velocity)
            
            data.extend(pos_bytes)
            data.extend(vel_bytes)
        
        # 计算校验和 (所有前面字节的和的低8位)
        checksum = sum(data) & 0xFF
        data.append(checksum)
        
        # 添加结束标记
        data.append(END_MARKER)
        
        return data


# 协议处理器映射表：将机械臂名称映射到对应的处理器类
PROTOCOL_HANDLERS = {
    'URDF_armv001': URDF_armv001_ProtocolHandler
    # 在这里添加其他机械臂的处理器
    # 'ARM_NAME': ARM_ProtocolHandler,
}

def get_protocol_handler(arm_name, config):
    """
    根据机械臂名称获取对应的协议处理器
    
    Args:
        arm_name: 机械臂名称
        config: 协议配置
        
    Returns:
        ProtocolHandler: 对应的协议处理器实例
    """
    handler_class = PROTOCOL_HANDLERS.get(arm_name)
    if handler_class:
        return handler_class(config)
    else:
        logger.warning("No specific protocol handler found for '{}', using default".format(arm_name))
        return DefaultProtocolHandler(config)


# 以下为直接测试代码，当模块被直接运行时执行
if __name__ == "__main__":
    # 测试配置
    test_config = {
        'arm': {
            'name': 'URDF_armv001',
            'joint_names': ['fifth_joint', 'first_joint', 'fourth_joint', 'second_joint', 'sixth_joint', 'third_joint'],
            'joint_ids': [5, 1, 4, 2, 6, 3]  # 每个关节对应的ID
        },
        'protocol': {
            'start_marker': 0xFF,
            'end_marker': 0xFE,
            'cmd_type': 0x01
        }
    }
    
    # 测试轨迹点
    test_point = {
        'joint_names': ['fifth_joint', 'first_joint', 'fourth_joint', 'second_joint', 'sixth_joint', 'third_joint'],
        'positions': [-0.02, -0.01, 0.008, -0.042, 0.012, 0.038],
        'velocities': [-0.12, -0.10, 0.043, -0.24, 0.073, 0.228]
    }
    
    # Create handler and test
    handler = get_protocol_handler('URDF_armv001', test_config)
    data = handler.format_trajectory_point(test_point)
    
    # Output results
    logger.info("Generated protocol data: {}".format([format(b, '02X') for b in data]))
    logger.info("Data length: {} bytes".format(len(data))) 
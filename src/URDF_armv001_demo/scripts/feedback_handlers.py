#!/usr/bin/env python
# -*- coding: utf-8 -*-

import struct
import logging
import sys
from protocol_processing.deep_arm.demo import DeepArmDemo

# 检测Python版本
PY3 = sys.version_info[0] == 3

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    stream=sys.stdout
)
logger = logging.getLogger('feedback_handler')

class FeedbackHandler(object):
    """反馈协议处理器基类，定义了所有反馈处理器必须实现的接口"""
    
    def __init__(self, config):
        """
        初始化反馈处理器
        
        Args:
            config: 包含协议配置的字典
        """
        self.config = config
    
    def parse_feedback_data(self, data):
        """
        解析串口反馈数据
        
        Args:
            data: 串口接收到的字节数组
            
        Returns:
            dict: 包含关节状态的字典，如果解析失败则返回None
        """
        raise NotImplementedError("This method must be implemented by subclasses")
    
    def get_joint_mapping(self):
        """获取关节名称到ID的映射关系"""
        joint_names = self.config['arm']['joint_names']
        joint_ids = self.config['arm'].get('joint_ids', list(range(1, len(joint_names) + 1)))
        
        # 创建映射字典
        return dict(zip(joint_names, joint_ids))

    def bytes_to_float(self, data, start_idx):
        """将字节数组转换为浮点数，兼容Python 2和3"""
        if PY3:
            # Python 3: 直接使用bytes对象
            return struct.unpack('<f', bytes(data[start_idx:start_idx+4]))[0]
        else:
            # Python 2: 使用字符串转换
            bytes_str = ''.join([chr(b) for b in data[start_idx:start_idx+4]])
            return struct.unpack('<f', bytes_str)[0]


class URDF_armv001_FeedbackHandler(FeedbackHandler):
    """URDF_armv001机械臂的反馈处理器"""
    def __init__(self, config):
        super(URDF_armv001_FeedbackHandler, self).__init__(config)
        self.logger = logging.getLogger('URDF_armv001_FeedbackHandler')
        self.logger.info("URDF_armv001 feedback handler initialized")
        self.arm = DeepArmDemo()
    
    def parse_feedback_data(self, data):
        """解析机器人反馈数据"""

        # 解析并保存单电机反馈的帧数据
        self.arm.data_callback(data,'data')
        # # 获取所有电机的状态数据
        # positions = [0.0] * len(self.config['arm']['joint_names'])
        # velocities = [0.0] * len(self.config['arm']['joint_names'])
        # accelerations = [0.0] * len(self.config['arm']['joint_names'])
        # effort = [0.0] * len(self.config['arm']['joint_names'])

        # status = self.arm.get_status(motor_ids=[5,1,4,2,6,3])

        # positions = status['positions']
        # velocities = status['velocities']
        # effort = status['effort']
        
        # return {
        #     'positions': positions,
        #     'velocities': velocities,
        #     'accelerations': accelerations,
        #     'effort': effort
        # }

class DefaultFeedbackHandler(FeedbackHandler):
    """默认反馈处理器，当未找到特定的处理器时使用"""
    
    def parse_feedback_data(self, data):
        """
        默认的反馈数据解析方法，尽量兼容URDF_armv001
        """
        logger.warning("Using default feedback handler, may not be compatible with your arm!")
        
        # 提取协议常量
        START_MARKER = self.config.get('protocol', {}).get('start_marker', 0xFF)
        END_MARKER = self.config.get('protocol', {}).get('end_marker', 0xFE)
        
        # 检查数据长度
        if len(data) < 4:
            return None
            
        # 检查起始和结束标记
        if data[0] != START_MARKER or data[-1] != END_MARKER:
            logger.warning("Invalid packet markers")
            return None
            
        # 获取关节数量
        joint_count = data[2]
        
        # 尝试解析不同的格式
        # 首先尝试带ID的格式
        try:
            # 获取关节ID映射
            joint_mapping = self.get_joint_mapping()
            reverse_mapping = {v: k for k, v in joint_mapping.items()}
            
            positions = [0.0] * len(self.config['arm']['joint_names'])
            velocities = [0.0] * len(self.config['arm']['joint_names'])
            
            for i in range(joint_count):
                # 假设每个关节数据是9字节: ID(1) + 位置(4) + 速度(4)
                base_idx = 3 + i * 9
                
                if base_idx + 8 < len(data) - 2:
                    # 获取关节ID和数据
                    joint_id = data[base_idx]
                    pos = self.bytes_to_float(data, base_idx+1)
                    vel = self.bytes_to_float(data, base_idx+5)
                    
                    # 查找ID对应的关节
                    joint_name = reverse_mapping.get(joint_id)
                    if joint_name:
                        joint_idx = self.config['arm']['joint_names'].index(joint_name)
                        positions[joint_idx] = pos
                        velocities[joint_idx] = vel
            
            return {
                'positions': positions,
                'velocities': velocities,
                'accelerations': [0.0] * len(positions),
                'effort': [0.0] * len(positions)
            }
        except Exception as e:
            logger.warning("First parsing method failed: {}".format(e))
        
        # 如果带ID的解析失败，尝试无ID的格式（旧版本）
        try:
            positions = []
            velocities = []
            
            for i in range(joint_count):
                # 假设每个关节数据是12字节: 位置(4) + 速度(4) + 加速度(4)
                base_idx = 3 + i * 12
                
                if base_idx + 11 < len(data) - 2:
                    pos = self.bytes_to_float(data, base_idx)
                    vel = self.bytes_to_float(data, base_idx+4)
                    
                    positions.append(pos)
                    velocities.append(vel)
            
            if positions:
                return {
                    'positions': positions,
                    'velocities': velocities,
                    'accelerations': [0.0] * len(positions),
                    'effort': [0.0] * len(positions)
                }
        except Exception as e:
            logger.warning("Second parsing method failed: {}".format(e))
        
        # 如果所有解析方法都失败
        return None


# 反馈处理器映射表：将机械臂名称映射到对应的处理器类
FEEDBACK_HANDLERS = {
    'URDF_armv001': URDF_armv001_FeedbackHandler
    # 在这里添加其他机械臂的处理器
    # 'ARM_NAME': ARM_FeedbackHandler,
}

def get_feedback_handler(arm_name, config):
    """
    根据机械臂名称获取对应的反馈处理器
    
    Args:
        arm_name: 机械臂名称
        config: 协议配置
        
    Returns:
        FeedbackHandler: 对应的反馈处理器实例
    """
    handler_class = FEEDBACK_HANDLERS.get(arm_name)
    if handler_class:
        return handler_class(config)
    else:
        logger.warning("No specific feedback handler found for '{}', using default".format(arm_name))
        return DefaultFeedbackHandler(config)


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
    
    # 创建测试数据
    test_data = bytearray([
        0x41, 0x54, 0x14, 0x00, 0x37, 0xEC, 0x08, 0x8A, 0x3C, 0x7F, 0x9C, 0x81, 0xC3, 0x01, 0x18, 0x0D, 0x0A
    ])
    

    
    # 创建处理器并测试
    handler = get_feedback_handler('URDF_armv001', test_config)
    result = handler.parse_feedback_data(test_data)
    
    # Output the result
    if result:
        logger.info("Parsed result:")
        logger.info("Position: {}".format(result['positions']))
        logger.info("Velocity: {}".format(result['velocities']))
    else:
        logger.error("Parsing failed") 
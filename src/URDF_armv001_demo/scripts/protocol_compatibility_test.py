#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""验证protocol_processing包在Python 2.7中的兼容性"""

import sys
import os
import logging

# 设置日志
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger('compatibility_test')

logger.info("Python版本: %s", sys.version)

# 导入DeepArmProtocol
try:
    from protocol_processing.deep_arm.protocol import DeepArmProtocol
    logger.info("成功导入DeepArmProtocol")
    
    # 创建实例
    protocol = DeepArmProtocol()
    logger.info("成功创建DeepArmProtocol实例")
    
    # 测试函数
    motor_ids = [5, 1, 4, 2, 6, 3]
    positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    
    frame = protocol.create_motor_pos_frame_all(motor_ids, positions)
    logger.info("成功调用create_motor_pos_frame_all函数")
    
    # 修复bytearray处理
    if sys.version_info[0] >= 3:
        # Python 3: 直接将整数转为十六进制
        hex_frame = " ".join([hex(b) for b in frame])
    else:
        # Python 2: 需要先获取ord值
        hex_frame = " ".join([hex(ord(b) if isinstance(b, str) else b) for b in frame])
    
    logger.info("生成的数据帧: %s", hex_frame)
    
except Exception as e:
    logger.error("测试出错: %s", str(e), exc_info=True) 
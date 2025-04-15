#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import struct
import array
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension

def create_test_message(counter):
    """创建测试消息，模拟发送到机械臂的指令"""
    # 创建UInt8MultiArray消息 (而不是ByteMultiArray)
    msg = UInt8MultiArray()
    
    # 设置维度信息
    msg.layout.dim.append(MultiArrayDimension(
        label="length",
        size=13,  # 消息总长度
        stride=1
    ))
    
    # 协议格式:
    # 起始标记(1字节) + 命令类型(1字节) + 关节数量(1字节) + 
    # 位置(4字节) + 速度(4字节) + 校验和(1字节) + 结束标记(1字节)
    
    # 创建测试数据 - 改变位置值作为测试
    position = 0.1 * (counter % 20)  # 在0到2范围内变化
    
    # 构建数据包
    data_list = []
    
    # 添加头部
    data_list.append(0xFF)  # 起始标记
    data_list.append(0x01)  # 命令类型
    data_list.append(0x01)  # 1个关节
    
    # 添加位置数据(浮点数转字节)
    pos_bytes = struct.pack('<f', position)
    for b in pos_bytes:
        # 处理Python 2/3兼容性
        if isinstance(b, str):
            data_list.append(ord(b))
        else:
            data_list.append(b)
    
    # 添加速度数据(0.0)
    vel_bytes = struct.pack('<f', 0.0)
    for b in vel_bytes:
        # 处理Python 2/3兼容性
        if isinstance(b, str):
            data_list.append(ord(b))
        else:
            data_list.append(b)
    
    # 计算校验和
    checksum = sum(data_list) & 0xFF
    data_list.append(checksum)
    
    # 添加结束标记
    data_list.append(0xFE)
    
    # 确保所有数据是无符号字节
    msg.data = data_list
    
    return msg

def serial_test_publisher():
    """发布测试数据到串口节点"""
    rospy.init_node('serial_test_publisher', anonymous=True)
    pub = rospy.Publisher('/serial_node/data_to_send', UInt8MultiArray, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz
    
    counter = 0
    
    print("开始发布测试数据到/serial_node/data_to_send...")
    print("按Ctrl+C停止")
    
    while not rospy.is_shutdown():
        msg = create_test_message(counter)
        pub.publish(msg)
        
        # 显示发送了什么
        data_str = ', '.join(['0x{:02X}'.format(b) for b in msg.data])
        # 从第3-7字节重建浮点数
        bytes_array = bytearray([msg.data[i] for i in range(3, 7)])
        position = struct.unpack('<f', bytes_array)[0]
        
        rospy.loginfo("发送数据 #%d: 位置=%.2f, 数据=[%s]", counter, position, data_str)
        
        counter += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        serial_test_publisher()
    except rospy.ROSInterruptException:
        pass 
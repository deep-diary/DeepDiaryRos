#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
测试 Python 2.7 中的字节处理
用于确保在 Python 2.7 环境中正确处理字节数据
"""

from __future__ import print_function, division

import sys
import struct
import binascii

# 强制设置编码
reload(sys)
sys.setdefaultencoding('utf-8')

def test_bytes_operations():
    """测试Python 2.7中的字节操作"""
    print("Python版本:", sys.version)
    
    # 字符串到字节
    s1 = "Hello"
    b1 = s1
    print("字符串:", s1)
    print("字节表示:", repr(b1))
    print("十六进制:", binascii.hexlify(b1))
    
    # 字符串与Unicode
    s2 = u"你好"
    b2 = s2.encode('utf-8')
    print("\nUnicode字符串:", s2)
    print("UTF-8编码字节:", repr(b2))
    print("十六进制:", binascii.hexlify(b2))
    print("解码回Unicode:", b2.decode('utf-8'))
    
    # 二进制数据创建
    b3 = struct.pack("<BH", 0xA5, 0x1234)
    print("\n二进制数据:", repr(b3))
    print("十六进制:", binascii.hexlify(b3))
    
    # 解包数据
    v1, v2 = struct.unpack("<BH", b3)
    print("解包值:", hex(v1), hex(v2))
    
    # 使用bytearray
    ba = bytearray([0x00, 0x01, 0x02, 0x03])
    print("\nBytearray:", repr(ba))
    print("十六进制:", binascii.hexlify(ba))
    
    # 修改bytearray
    ba[0] = 0xFF
    print("修改后:", repr(ba))
    print("十六进制:", binascii.hexlify(ba))
    
    # 连接字节数据
    b4 = b1 + b3
    print("\n连接字节:", repr(b4))
    print("十六进制:", binascii.hexlify(b4))
    
    # 创建特定长度的字节
    b5 = '\x00' * 5
    print("\n5个零字节:", repr(b5))
    print("十六进制:", binascii.hexlify(b5))
    
    # 查找子字节
    pos = b4.find(b3)
    print("\n在", repr(b4), "中查找", repr(b3), "位置:", pos)

if __name__ == "__main__":
    test_bytes_operations() 
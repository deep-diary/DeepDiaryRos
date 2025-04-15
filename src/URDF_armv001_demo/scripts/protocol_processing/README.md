# 协议处理包 (protocol_processing)

协议处理包是一个用于处理不同产品串口协议的 Python 包，作为串口通信层和应用层之间的桥梁。它提供了协议的抽象化，负责串口下发打包和反馈解析。

## 特点

- 可扩展的协议抽象框架
- 基于 YAML 配置的协议定义
- 支持多种数据类型的参数编码和解码
- 内置校验和计算和验证
- 集成了错误处理和日志记录

## 目录结构

protocol_processing/
├── init.py - 包初始化文件
├── base.py - 协议处理器抽象基类
├── exceptions.py - 协议相关异常
├── utils.py - 共享工具函数
├── deep_arm/ - Deep Arm 产品协议
│ ├── init.py
│ ├── config.yaml - Deep Arm 协议配置
│ ├── protocol.py - Deep Arm 协议实现
│ └── utils.py - Deep Arm 专用工具函数
└── template/ - 模板产品协议
├── init.py
├── config.yaml - 模板配置
├── protocol.py - 模板协议实现
└── utils.py - 模板工具函数

#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Protocol Processor Exceptions
Define exception types for protocol processing
"""

class ProtocolError(Exception):
    """Base class for all protocol exceptions"""
    pass

class ConfigError(ProtocolError):
    """Configuration error exception"""
    pass

class PacketError(ProtocolError):
    """Packet format error exception"""
    pass

class ChecksumError(PacketError):
    """Checksum error exception"""
    pass

class CommandError(ProtocolError):
    """Command error exception"""
    pass

class ResponseError(ProtocolError):
    """Response error exception"""
    pass

class TimeoutError(ProtocolError):
    """Timeout error exception"""
    pass 
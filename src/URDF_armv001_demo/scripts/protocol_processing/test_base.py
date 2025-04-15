#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Test the base protocol module compatibility
Verify that base.py works in Python 2.7 environment
"""

from __future__ import division, print_function, absolute_import

import sys
import os
import logging

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger('base_compatibility_test')

logger.info("Python version: %s", sys.version)

# Test imports
try:
    # Add current directory to search path
    current_dir = os.path.dirname(os.path.abspath(__file__))
    if current_dir not in sys.path:
        sys.path.insert(0, current_dir)
    
    # Ensure all dependency modules can be imported
    import protocol_exceptions
    logger.info("Successfully imported exception module")
    
    # Import base modules from current directory
    from base import ProtocolCommand, ProtocolResponse, ProtocolProcessor
    logger.info("Successfully imported base classes")
    
    # Test command class
    cmd = ProtocolCommand("test_command", {"param1": "value1", "param2": 123})
    logger.info("Successfully created command object: %s", cmd)
    
    # Test response class
    resp_success = ProtocolResponse("test_response", {"result": "ok"}, True)
    logger.info("Successfully created success response object: %s", resp_success)
    
    resp_error = ProtocolResponse("test_response", None, False, "test error")
    logger.info("Successfully created failure response object: %s", resp_error)
    
    # Test abstract class implementation
    logger.info("Creating ProtocolProcessor implementation class")
    
    class TestProcessor(ProtocolProcessor):
        """Test processor implementation"""
        
        def encode_command(self, command):
            """Implement encode command method"""
            return "ENCODED:{}:{}".format(command.command_type, str(command.parameters))
        
        def decode_response(self, data):
            """Implement decode response method"""
            return ProtocolResponse("test_decode", {"data": data}, True)
        
        def create_command(self, command_type, **parameters):
            """Implement create command method"""
            return ProtocolCommand(command_type, parameters)
        
        def validate_response(self, response, command=None):
            """Implement validate response method"""
            return response.success
    
    # Instantiate test processor
    processor = TestProcessor()
    logger.info("Successfully created processor instance")
    
    # Test config loading
    config = processor.load_config()
    logger.info("Successfully called load_config: %s", config)
    
    # Test create command
    test_cmd = processor.create_command("test", param1="value1", param2="value2")
    logger.info("Successfully called create_command: %s", test_cmd)
    
    # Test encode command
    encoded = processor.encode_command(test_cmd)
    logger.info("Successfully called encode_command: %s", encoded)
    
    # Test decode response
    decoded = processor.decode_response(encoded)
    logger.info("Successfully called decode_response: %s", decoded)
    
    # Test validate response
    valid = processor.validate_response(decoded)
    logger.info("Successfully called validate_response: %s", valid)
    
    logger.info("All tests passed!")
    
except Exception as e:
    logger.error("Test error: %s", str(e), exc_info=True) 
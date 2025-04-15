#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Test file to verify DeepArmProtocol class import
"""

import os
import sys
import logging

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger('protocol_import_test')

# Python version check
PY2 = sys.version_info[0] == 2
if PY2:
    logger.info("Running on Python 2.7")
else:
    logger.info("Running on Python 3.x")

logger.info("Python version: %s", sys.version)
logger.info("Current directory: %s", os.getcwd())
logger.info("Script location: %s", os.path.abspath(__file__))

# Set up paths
current_dir = os.path.dirname(os.path.abspath(__file__))
protocol_processing_dir = os.path.abspath(os.path.join(current_dir, '../'))
scripts_dir = os.path.abspath(os.path.join(protocol_processing_dir, '../'))

# Add directories to sys.path
paths_to_add = [current_dir, protocol_processing_dir, scripts_dir]
for path in paths_to_add:
    if path not in sys.path:
        sys.path.insert(0, path)
        logger.info("Added to Python path: %s", path)

# Create queue.py compatibility layer if needed
if PY2 and not os.path.exists(os.path.join(scripts_dir, 'queue.py')):
    logger.info("Creating queue compatibility layer for Python 2.7")
    try:
        with open(os.path.join(scripts_dir, 'queue.py'), 'w') as f:
            f.write('''#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Python 2.7 compatibility module for queue
This module forwards imports to the Queue module for Python 2.7
"""

import sys

# Check Python version
if sys.version_info[0] < 3:
    # For Python 2, import from Queue
    from Queue import Queue, Empty, Full, PriorityQueue, LifoQueue
else:
    # For Python 3, import from queue (though this shouldn't be executed)
    from queue import Queue, Empty, Full, PriorityQueue, LifoQueue

# Export all the required names
__all__ = ['Queue', 'Empty', 'Full', 'PriorityQueue', 'LifoQueue']
''')
        logger.info("Successfully created queue.py compatibility layer")
    except Exception as e:
        logger.error("Failed to create queue.py: %s", e)

# Try to import DeepArmProtocol
try:
    logger.info("Attempting to import protocol module")
    
    # Try direct import - now should work with the fixed protocol.py
    try:
        # Make sure we import from the current directory
        import protocol
        from protocol import DeepArmProtocol, DeepArmCommand, DeepArmResponse
        logger.info("Successfully imported DeepArmProtocol from %s", protocol.__file__)
    except ImportError as e:
        logger.error("Failed to import protocol: %s", e)
        sys.exit(1)
    
    # Test that the class is working
    logger.info("Creating DeepArmProtocol instance")
    proto = DeepArmProtocol()
    logger.info("DeepArmProtocol instance created successfully")
    
    # Test some protocol methods if they exist
    logger.info("Testing protocol methods:")
    
    # Display available methods
    methods = [m for m in dir(proto) if not m.startswith('_')]
    logger.info("Available methods: %s", methods)
    
    # Test a few basic methods that should exist
    if hasattr(proto, 'create_motor_pos_frame'):
        logger.info("Testing create_motor_pos_frame")
        pos_frame = proto.create_motor_pos_frame(motor_id=1, position=0.0)
        logger.info("Position frame created: %s", [hex(ord(b) if isinstance(b, str) else b) for b in pos_frame])
        pos_all_frame = proto.create_motor_pos_frame_all(motor_ids=[1,2,3,4,5,6], positions=[0.0,2.0,3.0,4.0,5.0,6.0])
        for frame in pos_all_frame:
            logger.info("Position all frame created: %s", [hex(ord(b) if isinstance(b, str) else b) for b in frame])
            

    if hasattr(proto, 'format_frame'):
        logger.info("Testing format_frame")
        test_data = bytearray([0x3E, 0x01, 0x00, 0x08, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x00, 0xFA, 0x0D, 0x0A])
        formatted = proto.format_frame(test_data)
        logger.info("Frame formatted: %s", formatted)
    
    logger.info("All tests completed successfully!")
    
except Exception as e:
    logger.error("Error in protocol test: %s", e, exc_info=True)
    sys.exit(1) 
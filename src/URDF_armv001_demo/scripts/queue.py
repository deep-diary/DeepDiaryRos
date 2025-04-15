#!/usr/bin/env python
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
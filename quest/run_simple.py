#!/usr/bin/env python3
"""
Simple startup script for Quest Controller Receiver (HTTP version)
"""

import sys
import os
import asyncio
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from simple_server import main

if __name__ == "__main__":
    asyncio.run(main())
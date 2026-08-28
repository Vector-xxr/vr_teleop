#!/usr/bin/env python3
"""
Simple startup script for Quest Controller Receiver
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from server import main

if __name__ == "__main__":
    main()
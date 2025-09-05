"""
masks.py

Defines color mask ranges in LAB color space for stripe and wall detection.
These are used for color segmentation in image processing.

Each color is defined by a lower and upper LAB value.
"""

import numpy as np

# LAB color ranges for different targets:
rMagenta = [np.array([22, 150, 50], np.uint8), np.array([200, 200, 150], np.uint8)]
rRed     = [np.array([0, 150, 120], np.uint8), np.array([255, 220, 180], np.uint8)]
rGreen   = [np.array([0, 50, 0], np.uint8), np.array([255, 120, 255], np.uint8)]
rBlue    = [np.array([0, 120, 60], np.uint8), np.array([255, 160, 120], np.uint8)]
rOrange  = [np.array([0, 117, 150], np.uint8), np.array([255, 200, 200], np.uint8)]
rBlack   = [np.array([0, 105, 105], np.uint8), np.array([80, 151, 151], np.uint8)]

# Optional: type descriptor for lighting condition
lotType = "light"  # 0 for dark purple, 1 for magenta, not used in main loop
"""
Simple program to get my feet wet with openCV.
"""

import cv2 as cv
import numpy as np

print(cv.__version__)

# Load image
img = cv.imread('cat.jpg', 0)

# Show image
cv.imshow('Cat Image', img)
cv.waitKey(0)
cv.destroyAllWindows()


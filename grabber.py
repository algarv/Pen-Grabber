from interbotix_xs_modules.arm import InterbotixManipulatorXS
import pyrealsense2 as rs
import numpy as np
import cv2 as cv
import matplotlib
from matplotlib import pyplot as plt
robot = InterbotixManipulatorXS("px100", "arm", "gripper")

purple = [130,0,75]


## Camera set-up ##


##Image set-up and display##

img = cv.imread('Northwestern.png')
cv.imshow("Logo",img)
cv.waitKey(0)
cv.destroyAllWindows()

##Measure the pen location##



##Move Arm to its starting position##



##Open the grippers##



##Turn at the waist to face pen##



##Adjust the height to level the grippers##



##Move forward to meet pen##



##Close the grippers##




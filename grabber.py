from interbotix_xs_modules.arm import InterbotixManipulatorXS
import pyrealsense2 as rs
import numpy as np
import cv2

robot = InterbotixManipulatorXS("px100", "arm", "gripper")

##Move Arm to its starting position##



##Open the grippers##



##Measure the pen location##



##Turn at the waist to face pen##



##Adjust the height to level the grippers##



##Move forward to meet pen##



##Close the grippers##




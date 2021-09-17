from interbotix_xs_modules.arm import InterbotixManipulatorXS
import pyrealsense2 as rs
import numpy as np
import cv2 as cv
import math
import matplotlib
import time
from matplotlib import pyplot as plt
import pyrealsense2 as rs
import numpy as np
import cv2

theta = -np.pi/2
robot = InterbotixManipulatorXS("px100", "arm", "gripper")
robot.arm.go_to_sleep_pose()
robot.arm.go_to_home_pose()
robot.gripper.open()
robot.arm.set_single_joint_position("waist",theta)


color = [135,0,75]

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

# Streaming loop
while True:
    # Get frameset of color and depth
    frames = pipeline.wait_for_frames()
    # frames.get_depth_frame() is a 640x360 depth image

    # Align the depth frame to color frame
    aligned_frames = align.process(frames)

    # Get aligned frames
    aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
    color_frame = aligned_frames.get_color_frame()

    # Validate that both frames are valid
    if not aligned_depth_frame or not color_frame:
        continue

    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    img = color_image

    purple_pixels = list()
    purple_pixels_x = list()
    purple_pixels_y = list()

    for r in range(img.shape[0]):
        for c in range(img.shape[1]):
            if abs(img[r][c][0] - color[0])<50 and abs(img[r][c][1] - color[1])<50 and abs(img[r][c][2]-color[2])<50:
                purple_pixels_x.append(r)
                purple_pixels_y.append(c)
                purple_pixels.append((r,c))
                #print('Purple Identified!')

    ##binary = np.zeros((img.shape[0],img.shape[1]))
    ##for r in range(binary.shape[0]):
    ##    for c in range(binary.shape[1]):
    ##        if (r,c) in purple_pixels:
    ##            binary[r][c] = 1
    ##        else:
    ##            binary[r][c] = 0

    image_withroi = img

    try: 
        roi= [(min(purple_pixels_x),min(purple_pixels_y)),(min(purple_pixels_x),max(purple_pixels_y)),(max(purple_pixels_x),min(purple_pixels_y)),(max(purple_pixels_x),max(purple_pixels_y))]
        print(roi)
##        for r in range(img.shape[0]):
##            for c in range(img.shape[1]):
##                if (r == min(purple_pixels_x) or r == max(purple_pixels_x)):
##                    if (c > min(purple_pixels_y) and c < max(purple_pixels_y)):
##                        for i in range(-2,2):
##                            image_withroi[r+i][c+1] = [0,0,0]
##                elif (c == min(purple_pixels_y) or c == max(purple_pixels_y)):
##                    if (r > min(purple_pixels_x) and r < max(purple_pixels_x)):
##                        for i in range(-2,2):
##                            image_withroi[r+i][c+i] = [0,0,0]
        center_x = round(min(purple_pixels_x) + ((max(purple_pixels_x)-min(purple_pixels_x))/2))
        center_y = round(min(purple_pixels_y) + ((max(purple_pixels_y)-min(purple_pixels_y))/2))
        center = [center_x,center_y]

        depth = list()
        for i in range(-2,2):
            for j in range(-2,2):
                if depth_image[center_x+i][center_y+j] != 0:
                    depth.append(depth_image[center_x+i][center_y+j] * depth_scale)
        depth = sum(depth) / len(depth)
        print("Depth: ", depth)

        if depth != 0:
            cfg = profile.get_stream(rs.stream.color)
            intr = cfg.as_video_stream_profile().get_intrinsics()

            coordinates = rs.rs2_deproject_pixel_to_point(intr, [center_x,center_y], depth) 
            print("Coordinates: ", coordinates)

            target_theta = math.atan(coordinates[0]/coordinates[2]) - np.pi/2
            print("Target theta: ", target_theta)

            print("Theta:", theta)
            if (abs(theta - target_theta)>.02):
                theta = target_theta
##                if (theta - target_theta)<0:
##                    if abs(theta - target_theta) > .2:
##                        theta = theta + .2
##                    else:
##                        theta = theta + .05
##                else:
##                    if abs(theta - target_theta) > .2:
##                        theta = theta - .2
##                    else:
##                        theta = theta - .05
                robot.arm.set_single_joint_position("waist",theta)
            else:
                robot.arm.set_single_joint_position("waist",theta)
                break

    except:
        print('ROI Empty')

depth = depth - .13
print("Depth: ",depth)

a = (depth-.23)/.12
print("A: ", a)
if a > .9:
    a = .9

target_i = np.pi/2 - math.acos(a)
print("Target i: ", target_i)
i = 0
while i<=target_i:
    robot.arm.set_single_joint_position("shoulder",i)
    robot.arm.set_single_joint_position("elbow",-1.25*i)
    i = i+.3

print("Closing grippers")
robot.gripper.close()
time.sleep(.5)
#robot.arm.set_single_joint_position("shoulder",0)
#robot.arm.set_single_joint_position("elbow",0)
robot.arm.go_to_home_pose()
robot.gripper.open()
robot.arm.go_to_sleep_pose()
cv.destroyAllWindows()





##Adjust the height to level the grippers##



##Move forward to meet pen##



##Close the grippers##




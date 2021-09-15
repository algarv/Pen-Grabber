#from interbotix_xs_modules.arm import InterbotixManipulatorXS
import pyrealsense2 as rs
import numpy as np
import cv2 as cv
import matplotlib
from matplotlib import pyplot as plt
#robot = InterbotixManipulatorXS("px100", "arm", "gripper")

color = [135,0,75]


## Camera set-up ##

## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

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

    # Remove background - Set pixels further than clipping_distance to grey
    #grey_color = 153
    #depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
    #bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
    
    # Render images:
    #   depth align to color on left
    #   depth on right
    #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    #images = np.hstack((bg_removed, depth_colormap))

    #cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
    #cv2.imshow('Align Example', images)
    #key = cv2.waitKey(1)

    ##Image set-up and display##

    img = color_image
    #cv.imshow("Logo",img)
    #cv.waitKey(0)
    #cv.destroyAllWindows()

    purple_pixels = list()
    purple_pixels_x = list()
    purple_pixels_y = list()

    for r in range(img.shape[0]):
        for c in range(img.shape[1]):
            if abs(img[r][c][0] - color[0])<50 and abs(img[r][c][1] - color[1])<50 and abs(img[r][c][2]-color[2])<50:
                purple_pixels_x.append(r)
                purple_pixels_y.append(c)
                purple_pixels.append((r,c))
                print('Purple Identified!')

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
        break
        for r in range(img.shape[0]):
            for c in range(img.shape[1]):
                if (r == min(purple_pixels_x) or r == max(purple_pixels_x)):
                    if (c > min(purple_pixels_y) and c < max(purple_pixels_y)):
                        for i in range(-5,5):
                            image_withroi[r+i][c+1] = [0,0,0]
                elif (c == min(purple_pixels_y) or c == max(purple_pixels_y)):
                    if (r > min(purple_pixels_x) and r < max(purple_pixels_x)):
                        for i in range(-5,5):
                            image_withroi[r+i][c+i] = [0,0,0]
    except:
        print('ROI Empty')
center_x = min(purple_pixels_x) + ((max(purple_pixels_x)-min(purple_pixels_x))/2)
center_x = round(center_x)
center_y = min(purple_pixels_y) + ((max(purple_pixels_y)-min(purple_pixels_y))/2)
center_y = round(center_y)
center = [center_x,center_y]
print(center)

depth = depth_image[center_x][center_y] * depth_scale
print(depth)

cv.imshow("ROI",image_withroi)
cv.waitKey(0)
cv.destroyAllWindows()


##Move Arm to its starting position##



##Open the grippers##



##Turn at the waist to face pen##



##Adjust the height to level the grippers##



##Move forward to meet pen##



##Close the grippers##




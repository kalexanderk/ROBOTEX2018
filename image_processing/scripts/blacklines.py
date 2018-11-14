#Import OpenCV for easy image rendering
import cv2
#Import Numpy for easy array manipulation
import numpy as np
import copy
import pyrealsense2 as rs
import os
from scipy.interpolate import interp1d

os.system('v4l2-ctl --device=/dev/video2 -c white_balance_temperature_auto=0')
os.system('v4l2-ctl --device=/dev/video2 -c exposure_auto_priority=0')

# Create a pipeline
pipeline = rs.pipeline()

#Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
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


frames = pipeline.wait_for_frames()
#frames.get_depth_frame() is a 640x360 depth image
        
#Align the depth frame to color frame
aligned_frames = align.process(frames)
        
# Get aligned frames
aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
color_frame = aligned_frames.get_color_frame()


depth_image = np.asanyarray(aligned_depth_frame.get_data())
color_image = np.asanyarray(color_frame.get_data())
        
#Remove background - Set pixels further than clipping_distance to grey
grey_color = 153
depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
        
# Render images
depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
images = np.hstack((bg_removed, depth_colormap))

hsv = cv2.cvtColor(images, cv2.COLOR_BGR2HSV) 


lH = 0
lS = 124
lV = 77

hH = 10
hS = 215
hV = 140


cannyLowBound = 0
cannyHighBound = 1500


rho = 548
theta = np.pi / 159
hueghshold = 15
min_line_length = 0
max_line_gap = 0



def updateCaLowB(lw):
    global cannyLowBound
    cannyLowBound = lw
    return

def updateCaHighB(lw):
    global cannyHighBound
    cannyHighBound = lw
    return




def updateRho(lw):
    global rho
    rho = lw
    return

def updateTheta(lw):
    global theta
    theta = np.pi/lw
    return

def updateHueghshold(lw):
    global hueghshold
    hueghshold = lw
    return

def updateMinLineLen(lw):
    global max_line_gap
    max_line_gap = lw
    return

def updateMaxLineGap(lw):
    global max_line_gap
    max_line_gap = lw
    return






def updatelH(lw):
    global lH
    lH = lw
    return


def updatelS(lw):
    global lS
    lS = lw
    return


def updatelV(lw):
    global lV
    lV = lw
    return


def updatehH(lw):
    global hH
    hH = lw
    return


def updatehS(lw):
    global hS
    hS = lw
    return


def updatehV(lw):
    global hV
    hV = lw
    return



lowerLimits = np.array([lH, lS, lV])
upperLimits = np.array([hH, hS, hV])

thresholded = cv2.inRange(hsv, lowerLimits, upperLimits)

cv2.imshow('thresh', thresholded)
        
cv2.createTrackbar("lH", "thresh", lH, 255, updatelH) 
cv2.createTrackbar("lS", "thresh", lS, 255, updatelS) 
cv2.createTrackbar("lV", "thresh", lV, 255, updatelV) 
cv2.createTrackbar("hH", "thresh", hH, 255, updatehH) 
cv2.createTrackbar("hS", "thresh", hS, 255, updatehS) 
cv2.createTrackbar("hV", "thresh", hV, 255, updatehV)

cv2.createTrackbar("rho", "thresh", rho, 10000, updateRho) 
cv2.createTrackbar("theta*pi", "thresh", int(theta), 10000, updateTheta) 
cv2.createTrackbar("hueghshold", "thresh", hueghshold, 255, updateHueghshold) 
cv2.createTrackbar("min_line_length", "thresh", min_line_length, 10000, updateMinLineLen) 
cv2.createTrackbar("max_line_gap", "thresh", max_line_gap, 10000, updateMaxLineGap) 

cv2.createTrackbar("Canny Low Bound", "thresh", cannyLowBound, 1500, updateCaLowB) 
cv2.createTrackbar("Canny Upper Bound", "thresh", cannyHighBound, 1500, updateCaHighB)

x_cent = 320
y_cent = 479

while True:

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
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
    grey_color = 153
    depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
    bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

    # Render images
    #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    #images = np.hstack((bg_removed, depth_colormap))
    images = color_image


    imgcopy = copy.deepcopy(images)
    hsv = cv2.cvtColor(images, cv2.COLOR_BGR2HSV)

    lowerLimits = np.array([lH, lS, lV])
    upperLimits = np.array([hH, hS, hV])

    hsv = cv2.inRange(hsv, lowerLimits, upperLimits)
    cv2.imshow('hsv', hsv)

    thresholded = cv2.Canny(hsv, cannyLowBound, cannyHighBound)
    cv2.imshow('canny', thresholded)
    lineImg = np.copy(thresholded) * 0

    lines = cv2.HoughLinesP(thresholded, rho, theta, hueghshold, min_line_length, max_line_gap)

    if lines != None:
        for line in lines:
            for x1,y1,x2,y2 in line:
                cv2.line(lineImg, (x1,y1), (x2,y2), (17,102,74), 5)

    cv2.imshow('thresh', lineImg)

    # x_ball, y_ball --- get the values from the topic
    x_ball = 223
    y_ball = 400

    if x_ball < x_cent:
        x1 = x_ball
        x2 = x_cent
        y1 = y_ball
        y2 = y_cent
    else:
        x1 = x_cent
        x2 = x_ball
        y1 = y_cent
        y2 = y_ball

    f_cent_ball = interp1d([x1, x2], [y1, y2])

    for x_i in range(x1, x2):
        if hsv[x_i, int(f_cent_ball(x_i))] == 255:
            print(x_i, int(f_cent_ball(x_i)))
            print('False')
            break
        print("True")

pipeline.stop()
cv2.destroyAllWindows()


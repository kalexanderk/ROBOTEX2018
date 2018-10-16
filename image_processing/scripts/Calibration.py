#Import OpenCV for easy image rendering
import cv2
#Import Numpy for easy array manipulation
import numpy as np
import copy
import pyrealsense2 as rs
import os

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


#detectcontrl = detectcontrl[r[0]:r[1],r[2]:r[3]]

f = open("/home/megatron/.ros/values.txt", "r")

lH = int(f.readline().strip('\n'))
lS = int(f.readline().strip('\n'))
lV = int(f.readline().strip('\n'))
hH = int(f.readline().strip('\n'))
hS = int(f.readline().strip('\n'))
hV = int(f.readline().strip('\n'))


bil = int(f.readline().strip('\n'))
dil = int(f.readline().strip('\n'))
err = int(f.readline().strip('\n'))

ByAreaBool = int(f.readline().strip('\n'))
minAreaf = int(f.readline().strip('\n'))
maxAreaf = int(f.readline().strip('\n'))
ByColorBool = int(f.readline().strip('\n'))
blobColorf = int(f.readline().strip('\n'))
ByCircularityBool = int(f.readline().strip('\n'))
minCircularityf = int(f.readline().strip('\n'))
maxCircularityf = int(f.readline().strip('\n'))
ByConvexityBool = int(f.readline().strip('\n'))
minConvexityf = int(f.readline().strip('\n'))
maxConvexityf = int(f.readline().strip('\n'))
ByInertiaBool =int(f.readline().strip('\n'))
minInertiaRatio = int(f.readline().strip('\n'))
maxInertiaRatio = int(f.readline().strip('\n'))

f.close()



#------------------------------------------
pBAB = ByAreaBool
pmA = minAreaf
pMA = maxAreaf
pBCB = ByColorBool
pbC = blobColorf
pBCB = ByCircularityBool 
pmCf = minCircularityf
pMCf = maxCircularityf
pbCB = ByConvexityBool 
pmCof = minConvexityf
pMCof = maxConvexityf
pbIB = ByInertiaBool
pmIr = minInertiaRatio
pMIr = maxInertiaRatio


def updateBABool(lw):
    global ByAreaBool
    ByAreaBool = lw
    return


def updateMiAF(lw):
    global minAreaf
    minAreaf = lw
    return


def updateMaAF(lw):
    global maxAreaf
    maxAreaf = lw
    return


def updateBCoBool(lw):
    global ByColorBool
    ByColorBool = lw
    return


def updateBCF(lw):
    global blobColorf
    blobColorf = lw
    return


def updateBCiBool(lw):
    global ByCircularityBool
    ByCircularityBool = lw
    return


def updateMiCF(lw):
    global minCircularityf
    minCircularityf = lw
    return


def updateMaCF(lw):
    global maxCircularityf
    maxCircularityf = lw
    return


def updateBConBool(lw):
    global ByConvexityBool
    ByConvexityBool = lw
    return


def updateMiConF(lw):
    global minConvexityf
    minConvexityf = lw
    return


def updateMaConF(lw):
    global maxConvexityf
    maxConvexityf = lw
    return


def updateBIBool(lw):
    global ByInertiaBool
    ByInertiaBool = lw
    return


def updateMiIR(lw):
    global minInertiaRatio
    minInertiaRatio = lw
    return


def updateMaIR(lw):
    global maxInertiaRatio
    maxInertiaRatio = lw
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


def updatedil(lw):
    global dil
    dil = lw
    return


def updateErr(lw):
    global err
    err = lw
    return


def updateBil(lw):
     global bil
     bil = lw
     return

lowerLimits = np.array([lH, lS, lV])
upperLimits = np.array([hH, hS, hV])

hsv = cv2.cvtColor(images, cv2.COLOR_BGR2HSV) 
thresholded = cv2.inRange(images, lowerLimits, upperLimits)
outimage = cv2.bitwise_and(hsv, hsv, mask = thresholded)


r = [len(images)-200,len(images)-190,0,len(images[0])]

detectcontrl = copy.deepcopy(images)
detectcontrl = detectcontrl[r[0]:r[1],r[2]:r[3]]

cv2.imshow('Threshold', outimage)
cv2.imshow('original', images)
cv2.imshow('detectcontrl', detectcontrl)

cv2.createTrackbar("lH", "original", lH, 255, updatelH) 
cv2.createTrackbar("lS", "original", lS, 255, updatelS) 
cv2.createTrackbar("lV", "original", lV, 255, updatelV) 
cv2.createTrackbar("hH", "original", hH, 255, updatehH) 
cv2.createTrackbar("hS", "original", hS, 255, updatehS) 
cv2.createTrackbar("hV", "original", hV, 255, updatehV)

cv2.createTrackbar("Dilation", 'original', dil, 255, updatedil) 
cv2.createTrackbar("Erosion", 'original', err, 255, updateErr) 
cv2.createTrackbar("Gauss", "original", bil, 255, updateBil) 
cv2.createTrackbar("ByAreaBool", 'detectcontrl', ByAreaBool, 1, updateBABool)
cv2.createTrackbar("minAreaf", 'detectcontrl', minAreaf,100000, updateMiAF)
cv2.createTrackbar("maxAreaf", 'detectcontrl', maxAreaf, 100000, updateMaAF)
cv2.createTrackbar("ByColorBool", 'detectcontrl', ByColorBool, 1, updateBCoBool) 
cv2.createTrackbar("blobColorf", 'detectcontrl', blobColorf, 255, updateBCF)
cv2.createTrackbar("ByCircularityBool", 'detectcontrl', ByCircularityBool, 1, updateBCiBool) 
cv2.createTrackbar("minCircularityf", 'detectcontrl', minCircularityf, 100, updateMiCF)
cv2.createTrackbar("maxCircularityf", 'detectcontrl', maxCircularityf, 100, updateMaCF)
cv2.createTrackbar("ByConvexityBool", 'detectcontrl', ByConvexityBool, 1, updateBConBool) 
cv2.createTrackbar("minConvexityf", 'detectcontrl', minConvexityf, 100, updateMiConF)
cv2.createTrackbar("maxConvexityf", 'detectcontrl', maxConvexityf, 100, updateMaConF)
cv2.createTrackbar("ByInertiaBool", 'detectcontrl', ByInertiaBool, 1, updateBIBool) 
cv2.createTrackbar("minInertiaRatio", 'detectcontrl', minInertiaRatio, 100, updateMiIR)
cv2.createTrackbar("maxInertiaRatio", 'detectcontrl', maxInertiaRatio, 100, updateMaIR)
#------------------------------------------


params = cv2.SimpleBlobDetector_Params()
params.filterByArea = ByAreaBool
params.minArea = minAreaf
params.maxArea = maxAreaf
            
params.filterByColor = ByColorBool
params.blobColor = blobColorf
params.filterByCircularity = ByCircularityBool
params.minCircularity = minCircularityf/100.0
params.maxCircularity = maxCircularityf/100.0
            
params.filterByConvexity = ByConvexityBool
params.minConvexity = minConvexityf/100.0
params.maxConvexity = maxConvexityf/100.0
            
params.filterByInertia = ByInertiaBool
params.minInertiaRatio = minInertiaRatio/100.0
params.maxInertiaRatio = maxInertiaRatio/100.0


detector = cv2.SimpleBlobDetector_create(params)

try:
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


        # ------------------------------------------
        if (pBAB != ByAreaBool or pmA != minAreaf or pMA != maxAreaf or
            pBCB != ByColorBool or pbC != blobColorf or pBCB != ByCircularityBool or
            pmCf != minCircularityf or pMCf != maxCircularityf or
            pbCB != ByConvexityBool or pmCof != minConvexityf or
            pMCof != maxConvexityf or pbIB != ByInertiaBool or
            pmIr != minInertiaRatio or pMIr != maxInertiaRatio):
                params = cv2.SimpleBlobDetector_Params()
                params.filterByArea = ByAreaBool
                params.minArea = minAreaf
                params.maxArea = maxAreaf
                
                params.filterByColor = ByColorBool
                params.blobColor = blobColorf
                params.filterByCircularity = ByCircularityBool
                params.minCircularity = minCircularityf/100.0
                params.maxCircularity = maxCircularityf/100.0
                
                params.filterByConvexity = ByConvexityBool
                params.minConvexity = minConvexityf/100.0
                params.maxConvexity = maxConvexityf/100.0
                
                params.filterByInertia = ByInertiaBool
                params.minInertiaRatio = minInertiaRatio/100.0
                params.maxInertiaRatio = maxInertiaRatio/100.0
             
                detector = cv2.SimpleBlobDetector_create(params)
        # ------------------------------------------



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

        lowerLimits = np.array([lH, lS, lV])
        upperLimits = np.array([hH, hS, hV])
        
        imgcopy = copy.deepcopy(images)
        hsv = cv2.cvtColor(images, cv2.COLOR_BGR2HSV)
        thresholded = cv2.inRange(hsv, lowerLimits, upperLimits)


        thresholded = cv2.GaussianBlur(thresholded, (1 + (2*bil), 1 + (2*bil)), 0)
        erernel = np.ones((1+err,1+err),np.uint8)
        thresholded = cv2.erode(thresholded,erernel,iterations = 1)
        dilernel = np.ones((1+dil,1+dil),np.uint8)
        thresholded = cv2.dilate(thresholded,dilernel,iterations = 1)


        outimage = cv2.bitwise_and(hsv, hsv, mask = thresholded)
        keypoints = detector.detect(outimage)
        print(len(keypoints))
        imgcopy = cv2.drawKeypoints(imgcopy, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv2.imshow('threshold', outimage)
        cv2.imshow('original', imgcopy)
        # cv2.imshow('detectcontrl', detectcontrl)

        if cv2.waitKey(1) & 0xFF == ord('s'):


            break

finally:
    f = open("/home/megatron/.ros/values.txt", "w")

    f.write(str(lH) + "\n")
    f.write(str(lS) + "\n")
    f.write(str(lV) + "\n")
    f.write(str(hH) + "\n")
    f.write(str(hS) + "\n")
    f.write(str(hV) + "\n")

    f.write(str(bil) + "\n")
    f.write(str(dil) + "\n")
    f.write(str(err) + "\n")

    f.write(str(ByAreaBool) + "\n")
    f.write(str(minAreaf) + "\n")
    f.write(str(maxAreaf) + "\n")
    f.write(str(ByColorBool) + "\n")
    f.write(str(blobColorf) + "\n")
    f.write(str(ByCircularityBool) + "\n")
    f.write(str(minCircularityf) + "\n")
    f.write(str(maxCircularityf) + "\n")
    f.write(str(ByConvexityBool) + "\n")
    f.write(str(minConvexityf) + "\n")
    f.write(str(maxConvexityf) + "\n")
    f.write(str(ByInertiaBool) + "\n")
    f.write(str(minInertiaRatio) + "\n")
    f.write(str(maxInertiaRatio) + "\n")

    f.close()

    pipeline.stop()
    cv2.destroyAllWindows()


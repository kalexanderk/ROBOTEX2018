import cv2
#Import Numpy for easy array manipulation
import numpy as np
import copy
import pyrealsense2 as rs


pipeline = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)


clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

align_to = rs.stream.color
align = rs.align(align_to)


frames = pipeline.wait_for_frames()

aligned_frames = align.process(frames)

# Get aligned frames
aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
color_frame = aligned_frames.get_color_frame()

depth_image = np.asanyarray(aligned_depth_frame.get_data())
color_image = np.asanyarray(color_frame.get_data())

# Remove background - Set pixels further than clipping_distance to grey
grey_color = 153
depth_image_3d = np.dstack((depth_image, depth_image, depth_image))  # depth image is 1 channel, color is 3 channels
bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

# Render images
depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
images = np.hstack((bg_removed, depth_colormap))


lH = 60
lS = 100
lV = 40
hH = 90
hS = 255
hV = 255

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

try:
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack(
            (depth_image, depth_image, depth_image))  # depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        # Render images
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        # images = np.hstack((bg_removed, depth_colormap))
        images = color_image

        lowerLimits = np.array([lH, lS, lV])
        upperLimits = np.array([hH, hS, hV])

        imgcopy = copy.deepcopy(images)
        hsv = cv2.cvtColor(images, cv2.COLOR_BGR2HSV)
        thresholded = cv2.inRange(hsv, lowerLimits, upperLimits)

        img, contours, hierarchy = cv2.findContours(thresholded, cv2.RETR_EXTERNAL,
                                                    cv2.CHAIN_APPROX_SIMPLE)

        balls_in_frame = []

        for contour in contours:
            contour_size = cv2.contourArea(contour)

            if contour_size < 10:
                continue

            rect = cv2.boundingRect(contour)

            balls_in_frame.append(rect)

        outimage = cv2.bitwise_and(hsv, hsv, mask=thresholded)

        for i in balls_in_frame:
            cv2.circle(imgcopy, (int(i[0]), int(i[1])), int(i[2]), (0, 255, 0), 2)
            cv2.putText(imgcopy, str(i[0]) + ' ; ' + str(i[1]), (int(i[0]), int(i[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 255), 2)

        cv2.imshow('threshold', outimage)
        cv2.imshow('original', imgcopy)
        cv2.imshow('detectcontrl', detectcontrl)

        if cv2.waitKey(1) & 0xFF == ord('s'):
            pipeline.stop()
            cv2.destroyAllWindows()
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
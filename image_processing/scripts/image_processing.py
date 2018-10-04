#!/usr/bin/env python
# image_processing.py
# Reads from the camera and writes object positions to the objects channel

import rospy
from std_msgs.msg import String
import numpy as np
import cv2
import pyrealsense2 as rs
import copy

# Constants: TODO put this in a configuration file
bil = 0
dil = 14
err = 0
ByAreaBool = 1
minAreaf = 8
maxAreaf = 100000
ByColorBool = 1
blobColorf = 255
ByCircularityBool = 0
minCircularityf = 0
maxCircularityf = 64
ByConvexityBool = 0
minConvexityf = 0
maxConvexityf = 100
ByInertiaBool = 0
minInertiaRatio = 0
maxInertiaRatio = 100

DEBUG = False

# Important reading about cv2 color spaces:
# https://docs.opencv.org/3.4.2/df/d9d/tutorial_py_colorspaces.html
# Hue goes from 0 to 179
BALL_COLOR_LOWER_BOUND = np.array([36, 53, 47])
BALL_COLOR_UPPER_BOUND = np.array([80, 144, 153])

# These aren't actually calibrated yet
BLUE_BASKET_LOWER_BOUND = np.array([90, 100, 40])
BLUE_BASKET_UPPER_BOUND = np.array([125, 255, 255])

MAGENTA_BASKET_LOWER_BOUND = np.array([125, 100, 40])
MAGENTA_BASKET_UPPER_BOUND = np.array([170, 255, 255])


def debug_log(text):
    if DEBUG:
        rospy.loginfo(text)


class ImageProcessor:

    def __init__(self):

        self.object_publisher = rospy.Publisher(
            "image_processing/objects", String, queue_size=10)
        self.balls_in_frame = []
        self.pipeline = None
        self.profile = None
        self.depth_scale = None
        self.clipping_distance = None
        self.align = None
        self.depth_image = None
        self.ball_keypoints = None
        self.blue_basket_keypoints = None
        self.magenta_basket_keypoints = None
        self.hsv = None

    def run(self):

        self.pipeline = rs.pipeline()
        '''Create a config and configure the pipeline to stream different resolutions of color and depth streams'''
        config = rs.config()
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        '''Start streaming'''
        self.profile = self.pipeline.start(config)

        '''Getting the depth sensor's depth scale (see rs-align example for explanation)'''
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: ", self.depth_scale)

        '''We will be removing the background of objects more than
           clipping_distance_in_meters meters away'''
        clipping_distance_in_meters = 3  # 3 meters
        self.clipping_distance = clipping_distance_in_meters / self.depth_scale

        '''Create an align object
           rs.align allows us to perform alignment of depth frames to others frames
           The "align_to" is the stream type to which we plan to align depth frames.'''
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def process_image(self):

        rospy.loginfo("Image: scanning frame")

        '''Get frameset of color and depth'''
        frames = self.pipeline.wait_for_frames()

        '''Align the depth frame to color frame'''
        aligned_frames = self.align.process(frames)

        '''Get aligned frames'''
        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        '''Validate that both frames are valid'''
        if not aligned_depth_frame or not color_frame:
            return

        self.depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        '''Remove background - Set pixels further than clipping_distance to grey'''
        grey_color = 153
        depth_image_3d = np.dstack(
            (self.depth_image, self.depth_image, self.depth_image))  # depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        '''Render images'''
        self.hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        '''Find balls'''
        self.find_balls()

        '''Get the closest ball coordinates'''
        self.closest_ball = self.get_closest_ball_coordinates()

        # filedepth = open("workfile.txt", "w")
        # for i in range(0, len(self.depth_image)):
        #     for j in range(0, len(self.depth_image[0])):
        #         filedepth.write(str(self.depth_image[i][j]) + " ")
        #     filedepth.write("\n")
        # filedepth.close()

        self.send_objects()

        if DEBUG:
            rospy.loginfo(str(self.ball_keypoints))


    def find_balls(self):
        imgcopy = copy.deepcopy(self.hsv)
        thresholded = cv2.inRange(self.hsv, BALL_COLOR_LOWER_BOUND, BALL_COLOR_UPPER_BOUND)
        outimage = cv2.bitwise_and(self.hsv, self.hsv, mask=thresholded)
        '''Point of interest (blobs)'''
        self.ball_keypoints = detector.detect(outimage)
        # imgcopy = cv2.drawKeypoints(imgcopy, self.ball_keypoints, np.array([]), (0, 0, 255),
        #                             cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # cv2.imshow('threshold', outimage)
        # cv2.imshow('original', imgcopy)


    def find_basket(self, color):
        imgcopy = copy.deepcopy(self.hsv)
        if color == 'blue':
            thresholded = cv2.inRange(self.hsv, BLUE_BASKET_LOWER_BOUND, BLUE_BASKET_UPPER_BOUND)
            outimage = cv2.bitwise_and(self.hsv, self.hsv, mask=thresholded)
            '''Point of interest (blobs)'''
            self.blue_basket_keypoints = detector.detect(outimage)
            # imgcopy = cv2.drawKeypoints(imgcopy, self.blue_basket_keypoints, np.array([]), (0, 0, 255),
            #                             cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        elif color == 'magenta':
            thresholded = cv2.inRange(self.hsv, MAGENTA_BASKET_LOWER_BOUND, MAGENTA_BASKET_UPPER_BOUND)
            outimage = cv2.bitwise_and(self.hsv, self.hsv, mask=thresholded)
            '''Point of interest (blobs)'''
            self.magenta_basket_keypoints = detector.detect(outimage)
            # imgcopy = cv2.drawKeypoints(imgcopy, self.magenta_basket_keypoints, np.array([]), (0, 0, 255),
            #                             cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        else:
            return

        # cv2.imshow('threshold', outimage)
        # cv2.imshow('original', imgcopy)


    def get_center_distances(self, type):
        distances = []
        '''Access the image pixels and create a 1D numpy array then add to list'''
        for i in range(len(self.ball_keypoints)):
            if type == 'ball':
                pt = self.ball_keypoints[i].pt
            elif type == 'blue_basket':
                pt = self.blue_basket_keypoints[i].pt
            elif type == 'magenta_basket':
                pt = self.magenta_basket_keypoints[i].pt
            distances.append(self.depth_image[pt[1], pt[0]]*self.depth_scale)
        return distances


    def get_closest_ball_coordinates(self):
        debug_log(str(len(self.ball_keypoints)) + " balls found")
        # print number of detected "balls"
        print(len(self.ball_keypoints))
        if len(self.ball_keypoints) > 0:
            distances = self.get_center_distances('ball')
            print(distances)
            try:
                # return self.ball_keypoints[distances.index(max(distances))]
                return self.ball_keypoints[distances.index(min(distances))]
            except:
                return np.nan


    def send_objects(self):
        '''Distances'''
        # ball = self.get_center_distances()
        # baskets = "{}:{}".format(0, 0)
        # message = "{}\n{}".format(ball, baskets)
        '''Coordinates'''
        try:
            print(self.closest_ball)
            message = "{};{}\n".format(self.closest_ball.pt[0],
                                       self.closest_ball.pt[1])
        except:
            message = "None\n"
        self.object_publisher.publish(message)


if __name__ == "__main__":
    try:
        rospy.init_node("image_processor")

        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = ByAreaBool
        params.minArea = minAreaf
        params.maxArea = maxAreaf

        params.filterByColor = ByColorBool
        params.blobColor = blobColorf
        params.filterByCircularity = ByCircularityBool
        params.minCircularity = minCircularityf / 100.0
        params.maxCircularity = maxCircularityf / 100.0

        params.filterByConvexity = ByConvexityBool
        params.minConvexity = minConvexityf / 100.0
        params.maxConvexity = maxConvexityf / 100.0

        params.filterByInertia = ByInertiaBool
        params.minInertiaRatio = minInertiaRatio / 100.0
        params.maxInertiaRatio = maxInertiaRatio / 100.0

        detector = cv2.SimpleBlobDetector_create(params)

        camera = ImageProcessor()
        camera.run()
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            camera.process_image()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

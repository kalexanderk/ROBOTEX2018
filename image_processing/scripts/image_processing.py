#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Reads from the camera and writes object positions to the objects channel


import rospy
from std_msgs.msg import String
import numpy as np
import cv2
import pyrealsense2 as rs
import copy
import os


def generate_detector_from_file(textFile):
    f = open(textFile, "r")
    params = cv2.SimpleBlobDetector_Params()

    lH = int(f.readline().strip('\n'))
    lS = int(f.readline().strip('\n'))
    lV = int(f.readline().strip('\n'))
    hH = int(f.readline().strip('\n'))
    hS = int(f.readline().strip('\n'))
    hV = int(f.readline().strip('\n'))

    bil = int(f.readline().strip('\n'))
    dil = int(f.readline().strip('\n'))
    err = int(f.readline().strip('\n'))

    params.filterByArea = int(f.readline().strip('\n'))
    params.minArea = int(f.readline().strip('\n'))
    params.maxArea = int(f.readline().strip('\n'))

    params.filterByColor = int(f.readline().strip('\n'))
    params.blobColor = int(f.readline().strip('\n'))

    params.filterByCircularity = int(f.readline().strip('\n'))
    params.minCircularity = int(f.readline().strip('\n')) / 100.0
    params.maxCircularity = int(f.readline().strip('\n')) / 100.0

    params.filterByConvexity = int(f.readline().strip('\n'))
    params.minConvexity = int(f.readline().strip('\n')) / 100.0
    params.maxConvexity = int(f.readline().strip('\n')) / 100.0

    params.filterByInertia = int(f.readline().strip('\n'))
    params.minInertiaRatio = int(f.readline().strip('\n')) / 100.0
    params.maxInertiaRatio = int(f.readline().strip('\n')) / 100.0

    f.close()

    detector = cv2.SimpleBlobDetector_create(params)

    colour_lower_bound = np.array([lH, lS, lV])
    colour_upper_bound = np.array([hH, hS, hV])

    return detector, colour_lower_bound, colour_upper_bound


DEBUG = False


# Important reading about cv2 color spaces:
# https://docs.opencv.org/3.4.2/df/d9d/tutorial_py_colorspaces.html

def debug_log(text):
    if DEBUG:
        rospy.loginfo(text)


class ImageProcessor:

    def __init__(self):

        os.system('v4l2-ctl --device=/dev/video2 -c white_balance_temperature_auto=0')
        os.system('v4l2-ctl --device=/dev/video2 -c exposure_auto_priority=0')

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
        # print("Depth Scale is: ", self.depth_scale)

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

        # rospy.loginfo("Image: scanning frame")

        '''Get frameset of color and depth'''
        frames = self.pipeline.wait_for_frames()

        '''Align the depth frame to color frame'''
        aligned_frames = self.align.process(frames)

        '''Get aligned frames'''
        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 1280x720 depth image
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
        bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), grey_color,
                              color_image)

        '''Render images'''
        self.hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        '''Find balls'''
        self.find_balls()

        '''Find baskets'''
        self.find_basket('blue')

        '''Get the closest ball coordinates'''
        self.closest_ball = self.get_closest_ball_coordinates()

        self.send_objects()

        if DEBUG:
            rospy.loginfo(str(self.ball_keypoints))

    def find_balls(self):
        imgcopy = copy.deepcopy(self.hsv)
        thresholded = cv2.inRange(self.hsv, BALL_COLOR_LOWER_BOUND, BALL_COLOR_UPPER_BOUND)
        outimage = cv2.bitwise_and(self.hsv, self.hsv, mask=thresholded)
        '''Point of interest (blobs)'''
        self.ball_keypoints = balltector.detect(outimage)
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
            self.basket_keypoints = blue_gatector.detect(outimage)
        elif color == 'magenta':
            # thresholded = cv2.inRange(self.hsv, MAGENTA_BASKET_LOWER_BOUND, MAGENTA_BASKET_UPPER_BOUND)
            # outimage = cv2.bitwise_and(self.hsv, self.hsv, mask=thresholded)
            '''Point of interest (blobs)'''
        else:
            return

        try:
            self.basket_keypoints = self.basket_keypoints[0]
            pt = [int(self.basket_keypoints.pt[0]), int(self.basket_keypoints.pt[1])]
            loc = []
            for ix in range(pt[0] - 3, pt[0] + 4):
                if ix > 1279 or ix < 0:
                    continue
                for iy in range(pt[1] - 3, pt[1] + 4):
                    if iy > 719 or iy < 0:
                        continue
                    if self.depth_image[iy, ix] != 0:
                        loc.append(self.depth_image[iy, ix] * self.depth_scale)

            if len(loc) == 0:
                self.basket_distance = 99999900
            else:
                a = 0
                for el in loc:
                    a += el
                    self.basket_distance = a / len(loc)
        except:
            self.basket_keypoints = None

    def get_center_distances(self):
        distances = []
        '''Access the image pixels and create a 1D numpy array then add to list'''
        for i in range(len(self.ball_keypoints)):
            pt = [int(self.ball_keypoints[i].pt[0]), int(self.ball_keypoints[i].pt[1])]

            loc = []
            for ix in range(pt[0] - 3, pt[0] + 4):
                if ix > 1279 or ix < 0:
                    continue
                for iy in range(pt[1] - 3, pt[1] + 4):
                    if iy > 719 or iy < 0:
                        continue
                    if self.depth_image[iy, ix] != 0:
                        loc.append(self.depth_image[iy, ix] * self.depth_scale)

            if len(loc) == 0:
                distances.append(99999900)
            else:
                a = 0
                for el in loc:
                    a += el

                # print("lel", str(a))
                # print("Distance", str(a / len(loc)))
                distances.append(a / len(loc))
        return distances

    def get_closest_ball_coordinates(self):
        debug_log(str(len(self.ball_keypoints)) + " balls found")
        if len(self.ball_keypoints) > 0:
            distances = self.get_center_distances()
            try:
                # return self.ball_keypoints[distances.index(max(distances))]
                # print('Closest ball', self.ball_keypoints[distances.index(min(distances))], min(distances), '\n')
                return self.ball_keypoints[distances.index(min(distances))]
            except:
                return np.nan

    def send_objects(self):
        '''Distances'''
        # ball = self.get_center_distances()
        # baskets = "{}:{}".format(0, 0)
        # message = "{}\n{}".format(ball, baskets)
        '''Coordinates'''
        if self.closest_ball != None:
            message = "{};{}\n".format(self.closest_ball.pt[0], self.closest_ball.pt[1])
        else:
            message = "None\n"

        if self.basket_keypoints != None:
            message += "{};{};{}".format(self.basket_keypoints.pt[0],
                                         self.basket_keypoints.pt[1], self.basket_distance)
        else:
            message += "None"

        self.object_publisher.publish(message)


if __name__ == "__main__":
    try:
        flag_ = True

        rospy.init_node("image_processor")

        balltector, BALL_COLOR_LOWER_BOUND, BALL_COLOR_UPPER_BOUND = generate_detector_from_file("values.txt")

        blue_gatector, BLUE_BASKET_LOWER_BOUND, BLUE_BASKET_UPPER_BOUND = generate_detector_from_file("Gatevalues.txt")
        # magenta_gatector, MAGENTA_BASKET_LOWER_BOUND, MAGENTA_BASKET_UPPER_BOUND = generate_detector_from_file("Gatevalues.txt")

        camera = ImageProcessor()
        camera.run()
        rate = rospy.Rate(25)

        while not rospy.is_shutdown():
            camera.process_image()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
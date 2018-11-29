#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Reads from the camera and writes object positions to the objects channel

import rospy
from std_msgs.msg import String
import numpy as np
import cv2
import pyrealsense2 as rs
import os
from scipy.interpolate import interp1d


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
        self.field_number_sub = rospy.Subscriber("field_number",
                                                 String, self.field_number_callback)
        self.balls_in_frame = []
        self.pipeline = None
        self.profile = None
        self.depth_scale = None
        self.align = None
        self.depth_image = None
        self.ball_keypoints = None
        self.hsv = None
        self.black_lines_image = None
        self.basket_distance = None
        self.basket_keypoints = None
        self.closest_ball = None
        self.closest_ball_distance = None
        self.field_number = None

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

        '''Creating an align object'''
        #rs.align allows us to perform alignment of depth frames to others frames
        #"align_to" is the stream type to which we plan to align depth frames
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def process_image(self):
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

        '''Render images'''
        self.hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        '''Detect black lines'''
        self.detect_black_line()

        '''Find balls'''
        self.find_balls()

        '''Find baskets'''
        if self.field_number == magenta_field:
            self.find_basket('magenta')
        elif self.field_number == blue_field:
            self.find_basket('blue')

        '''Get the closest ball coordinates'''
        self.get_closest_ball()

        '''Send the information about objects detected'''
        self.send_objects()

        if DEBUG:
            rospy.loginfo(str(self.ball_keypoints))


    '''BASKETS' DETECTION'''
    def find_basket(self, color):
        if color == 'blue':
            thresholded = cv2.inRange(self.hsv, BLUE_BASKET_LOWER_BOUND, BLUE_BASKET_UPPER_BOUND)
            outimage = cv2.bitwise_and(self.hsv, self.hsv, mask=thresholded)
            '''Point of interest (blobs)'''
            self.basket_keypoints = blue_gatector.detect(outimage)
        elif color == 'magenta':
            thresholded = cv2.inRange(self.hsv, MAGENTA_BASKET_LOWER_BOUND, MAGENTA_BASKET_UPPER_BOUND)
            outimage = cv2.bitwise_and(self.hsv, self.hsv, mask=thresholded)
            '''Point of interest (blobs)'''
            self.basket_keypoints = magenta_gatector.detect(outimage)
        else:
            return

        try:
            self.basket_keypoints = self.basket_keypoints[0]
            pt = [int(self.basket_keypoints.pt[0]), int(self.basket_keypoints.pt[1])]
            distances_around_center = []
            # take the radius of 3 pixels
            for ix in range(pt[0] - 3, pt[0] + 4):
                # working with the image 1280 x 720
                if ix > 1279 or ix < 0:
                    continue
                # take the radius of 3 pixels
                for iy in range(pt[1] - 3, pt[1] + 4):
                    # working with the image 1280 x 720
                    if iy > 719 or iy < 0:
                        continue
                    if self.depth_image[iy, ix] != 0:
                        distances_around_center.append(self.depth_image[iy, ix] * self.depth_scale)
            if len(distances_around_center) == 0:
                self.basket_distance = 999999
            else:
                cumulative_distance = 0
                for distance in distances_around_center:
                    cumulative_distance += distance
                    self.basket_distance = cumulative_distance/len(distances_around_center)
        except:
            self.basket_keypoints = None


    '''CLOSEST BALL'S DETECTION'''
    def find_balls(self):
        thresholded = cv2.inRange(self.hsv, BALL_COLOR_LOWER_BOUND, BALL_COLOR_UPPER_BOUND)
        outimage = cv2.bitwise_and(self.hsv, self.hsv, mask=thresholded)
        '''Point of interest (blobs)'''
        self.ball_keypoints = balltector.detect(outimage)

    def get_center_distances(self):
        balls_distances = []
        '''Access the image pixels and create a 1D numpy array then add to list'''
        for i in range(len(self.ball_keypoints)):
            pt = [int(self.ball_keypoints[i].pt[0]), int(self.ball_keypoints[i].pt[1])]
            if self.beyond_black_line(pt):
                # remove it from the list
                self.ball_keypoints[i] = None
                pass
            distances_around_center = []
            # take the radius of 3 pixels
            for ix in range(pt[0] - 3, pt[0] + 4):
                # working with the image 1280 x 720
                if ix > 1279 or ix < 0:
                    continue
                # take the radius of 3 pixels
                for iy in range(pt[1] - 3, pt[1] + 4):
                    # working with the image 1280 x 720
                    if iy > 719 or iy < 0:
                        continue
                    if self.depth_image[iy, ix] != 0:
                        distances_around_center.append(self.depth_image[iy, ix] * self.depth_scale)

            if len(distances_around_center) == 0:
                balls_distances.append(999999)
            else:
                cumulative_distance = 0
                for distance in distances_around_center:
                    cumulative_distance += distance
                    balls_distances.append(cumulative_distance / len(distances_around_center))
        return [distance for distance in balls_distances if distance is not None]


    def get_closest_ball(self):
        debug_log(str(len(self.ball_keypoints)) + " balls found")
        if len(self.ball_keypoints) > 0:
            distances = self.get_center_distances()
            try:
                index_minimum_dist = distances.index(min(distances))
                self.closest_ball_distance = distances[index_minimum_dist]
                self.closest_ball = self.ball_keypoints[index_minimum_dist]
            except:
                self.closest_ball_distance = None
                self.closest_ball = None


    '''BLACK LINES' DETECTION'''
    def detect_black_line(self):
        self.black_lines_image = cv2.inRange(self.hsv, BLACK_LOWER_BOUND, BLACK_UPPER_BOUND)

    def beyond_black_line(self, pt):
        x_cent = 640
        y_max = 719
        if pt[0] < x_cent:
            x1 = pt[0]
            x2 = x_cent
            y1 = pt[1]
            y2 = y_max
        else:
            x1 = x_cent
            x2 = pt[0]
            y1 = y_max
            y2 = pt[1]
        f_cent_ball = interp1d([x1, x2], [y1, y2])
        for x_i in range(x1, x2):
            y_interpolated = int(f_cent_ball(x_i))
            if y_interpolated>=0 or y_interpolated<720:
                if self.black_lines_image[x_i, int(f_cent_ball(x_i))] == 255:
                    print(x_i, int(f_cent_ball(x_i)))
                    return True
            return False


    '''SENDING THE COORDINATES OF OBJECTS DETECTED'''
    def send_objects(self):
        '''Coordinates'''
        if self.closest_ball is not None:
            message = "{};{};{}\n".format(self.closest_ball.pt[0], self.closest_ball.pt[1], self.closest_ball_distance)
        else:
            message = "None\n"

        if self.basket_keypoints is not None:
            message += "{};{};{}".format(self.basket_keypoints.pt[0],
                                         self.basket_keypoints.pt[1], self.basket_distance)
        else:
            message += "None"

        self.object_publisher.publish(message)

    '''GETTING FIELD NUMBER FROM GAME LOGIC NODE'''
    def field_number_callback(self, message):
        self.field_number = str(message)[-2]


if __name__ == "__main__":
    try:
        flag_ = True

        rospy.init_node("image_processing")

        # colours files are in .ros folder
        balltector, BALL_COLOR_LOWER_BOUND, BALL_COLOR_UPPER_BOUND = generate_detector_from_file("Ballvalues.txt")

        blue_gatector, BLUE_BASKET_LOWER_BOUND, BLUE_BASKET_UPPER_BOUND = generate_detector_from_file("Gatevalues_blue.txt")
        magenta_gatector, MAGENTA_BASKET_LOWER_BOUND, MAGENTA_BASKET_UPPER_BOUND = generate_detector_from_file("Gatevalues_magenta.txt")

        BLACK_LOWER_BOUND = np.array([0, 124, 77])
        BLACK_UPPER_BOUND = np.array([10, 215, 140])

        camera = ImageProcessor()
        camera.run()
        rate = rospy.Rate(25)

        '''SET FIELDS' LETTERS'''
        magenta_field = 'A'
        blue_field = 'B'

        while not rospy.is_shutdown():
            camera.process_image()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        print('Ending Image Processing...')
        camera.pipeline.stop()

import numpy as np
import cv2
import pyrealsense2 as rs

# Constants: TODO put this in a conf file
WIDTH = 1280
HEIGHT = 720
FRAME_RATE = 30

MIN_CONTOUR_AREA = 10
DEBUG = False

# Important reading about cv2 color spaces:
# https://docs.opencv.org/3.4.2/df/d9d/tutorial_py_colorspaces.html
# Hue goes from 0 to 179
BALL_COLOR_LOWER_BOUND = np.array([60, 100, 40])
BALL_COLOR_UPPER_BOUND = np.array([90, 255, 255])

# These aren't actually calibrated yet
BLUE_BASKET_LOWER_BOUND = np.array([90, 100, 40])
BLUE_BASKET_UPPER_BOUND = np.array([125, 255, 255])

MAGENTA_BASKET_LOWER_BOUND = np.array([125, 100, 40])
MAGENTA_BASKET_UPPER_BOUND = np.array([170, 255, 255])


class ImageProcessor():
    def __init__(self):
        self.balls_in_frame = []

    def run(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FRAME_RATE)
        config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FRAME_RATE)
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        self.profile = self.pipeline.start(config)

        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

    def process_image(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        ball_mask = cv2.inRange(hsv_image, BALL_COLOR_LOWER_BOUND, BALL_COLOR_UPPER_BOUND)
        self.find_balls(ball_mask, depth_image)

        blue_basket_mask = cv2.inRange(hsv_image,
                                       BLUE_BASKET_LOWER_BOUND,
                                       BLUE_BASKET_UPPER_BOUND)
        self.blue_basket = self.find_basket(blue_basket_mask)

        magenta_basket_mask = cv2.inRange(hsv_image,
                                          MAGENTA_BASKET_LOWER_BOUND,
                                          MAGENTA_BASKET_UPPER_BOUND)
        self.magenta_basket = self.find_basket(magenta_basket_mask)

        self.send_objects()

        if DEBUG:
            self.print_mask(ball_mask)

    def find_balls(self, mask, depth):
        img, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                                    cv2.CHAIN_APPROX_SIMPLE)

        self.balls_in_frame = []

        for contour in contours:
            contour_size = cv2.contourArea(contour)

            if contour_size < MIN_CONTOUR_AREA:
                continue

            rect = cv2.boundingRect(contour)

            distance = self.get_corrected_mean_distance(contour, depth)

            self.balls_in_frame.append((rect, distance))

    def get_corrected_mean_distance(self, contour, depth):
        cimg = np.zeros_like(depth)
        cv2.drawContours(cimg, [contour], -1, color=255, thickness=-1)

        # Access the image pixels and create a 1D numpy array then add to list
        pts = np.where(cimg == 255)
        distances = []
        distances.append(depth[pts[0], pts[1]])

        distances = np.sort(distances)

        distance = np.mean(distances)

        return distance * self.depth_scale

    def find_basket(self, mask):
        img, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                                    cv2.CHAIN_APPROX_SIMPLE)

        basket = None
        biggest_contour_size = MIN_CONTOUR_AREA

        for contour in contours:
            contour_size = cv2.contourArea(contour)

            if contour_size > biggest_contour_size:
                biggest_contour_size = contour_size
                basket = cv2.boundingRect(contour)

        return basket

    def get_largest_ball(self):
        # Only send the horizontal location of the biggest ball for
        # now
        if len(self.balls_in_frame) > 0:
            max_size = 0
            max_ball = ()
            max_ball_distance = 0
            for ((x, y, width, height), distance) in self.balls_in_frame:
                if (width * height) > max_size:
                    max_size = width * height
                    max_ball = (x, y, width, height)
                    max_ball_distance = distance
            (x, y, width, height) = max_ball
            ball_pos = str(((x + (width / 2)) / float(WIDTH)) - 0.5)
            return ball_pos + ":" + str(max_ball_distance)
        else:
            return "None"

    def send_objects(self):
        ball = self.get_largest_ball()
        baskets = "{}:{}".format(self.blue_basket, self.magenta_basket)
        message = "{}\n{}".format(ball, baskets)
        # self.object_publisher.publish(message)

    def print_mask(self, mask):
        coverage = [0] * 64
        for y in range(1920):
            for x in range(1080):
                c = mask[y, x]
                if c > 128:
                    coverage[x // 10] += 1

            if y % 20 is 19:
                line = ""
                for c in coverage:
                    line += " .:nhBXWW"[c // 25]
                coverage = [0] * 64


camera = ImageProcessor()
camera.run

while True:
    camera.process_image


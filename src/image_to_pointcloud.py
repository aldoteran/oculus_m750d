#!/usr/bin/env python
"""
ROS node that converts a sonar image to a pointcloud on
the sonar frame.

Uses the simple_ping_result and the sonar_image to
get a pointcloud in the sonar frame.

The simple ping result is composed of:
    [seq, ros_timestamp, freq, speed_sound, ping_stat_time,
     image_resolution, number_ranges (rows), number_bearings (cols),
     image_start_offset, image_size]

Author: Aldo Teran <aldot@kth.se>
"""
import cv2
import numpy as np

class ImageConverter:
    """
    Class to handle Sonar image.
    """

    def __init__(self):
        """
        Not done yet.
        """
        self.sonar_result = {
            "image_msg" : None,
            "ping_result" : None,
            "image_array" : None
        }

        # Image processing parameters
        self.image_threshold = 120
        self.sigma_blur = 10.0
        self.ignore_below = 0.1

        # Init bearing arrays
        self.low_freq_brgs = None
        self.high_freq_brgs = None
        self._init_bearings()

    def _init_bearings(self):
        """
        Initialize the bearing vectors (since constant)
        to avoid redundant computations.
        """
        # Calculate for low frequency
        iterable = (-65+x*0.5078125 for x in range(256))
        bearings = np.fromiter(iterable, float)
        self.low_freq_brgs = np.expand_dims(bearings, 0)

        # Calculate for high frequency
        iterable = (-35+x*0.2734375 for x in range(256))
        bearings = np.fromiter(iterable, float)
        self.high_freq_brgs = np.expand_dims(bearings, 0)

    def _polar_to_cartesian(self, image, ping_result):
        resolution = ping_result[4]
        [rows, cols] = np.shape(image)

        # Check if image was taken in high frequency
        high_freq = (ping_result[2] > 1000000)
        if high_freq:
            bearings = np.tile(self.high_freq_brgs, (rows, 1))
        else:
            bearings = np.tile(self.low_freq_brgs, (rows, 1))

        # Build array with ranges
        iterable = (i*resolution for i in range(rows))
        ranges = np.fromiter(iterable, float)
        ranges = np.expand_dims(ranges, 1)
        ranges = np.tile(ranges, (1, cols))

        # Convert to to cartesian
        [x, y] = cv2.polarToCart(ranges, bearings, angleInDegrees=True)

        return [x, y]

    def detect_edges(self, image):
        """
        Build pointcloud and publish.
        """
        # Threshold image
        ret, image = cv2.threshold(image, self.image_threshold, 255, cv2.THRESH_TOZERO)

        # TODO: Maybe can tune parameters better, good results with current values.
        # Detect edges with second Laplacian
        image = cv2.Canny(image, 200, 255, L2gradient=True)

        # Convert from polar coordinates to cartesian
        [x_coords, y_coords] = self._polar_to_cartesian(image)




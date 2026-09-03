import math
import numpy as np


class GapFollower:

    def __init__(self):

        print("Gap Follower Initialized")

        ####################################################
        # Parameters
        ####################################################

        self.max_range = 10.0
        self.bubble_radius = 10

        self.max_speed = 3.0
        self.medium_speed = 2.0
        self.min_speed = 1.0

    ####################################################
    # Main Controller
    ####################################################

    def drive(self, state):

        ####################################################
        # Read Vehicle State
        ####################################################

        scan = state.scan

        ranges = np.array(scan.ranges)

        ####################################################
        # Clean LiDAR Data
        ####################################################

        ranges = np.nan_to_num(
            ranges,
            nan=0.0,
            posinf=self.max_range,
            neginf=0.0
        )

        ranges = np.clip(ranges, 0.0, self.max_range)

        ####################################################
        # Ignore Rear of Vehicle
        ####################################################

        front_angle = math.radians(90)

        start = int(
            (-front_angle - scan.angle_min)
            / scan.angle_increment
        )

        end = int(
            (front_angle - scan.angle_min)
            / scan.angle_increment
        )

        ranges = ranges[start:end]

        ####################################################
        # Find Closest Obstacle
        ####################################################

        closest = np.argmin(ranges)

        ####################################################
        # Create Safety Bubble
        ####################################################

        bubble_start = max(0, closest - self.bubble_radius)
        bubble_end = min(len(ranges), closest + self.bubble_radius)

        ranges[bubble_start:bubble_end] = 0.0

        ####################################################
        # Find Largest Gap
        ####################################################

        best_start = 0
        best_end = 0

        gap_start = None

        for i in range(len(ranges)):

            if ranges[i] > 0.0:

                if gap_start is None:
                    gap_start = i

            else:

                if gap_start is not None:

                    if i - gap_start > best_end - best_start:

                        best_start = gap_start
                        best_end = i

                    gap_start = None

        # Handle a gap that reaches the last LiDAR point
        if gap_start is not None:

            if len(ranges) - gap_start > best_end - best_start:

                best_start = gap_start
                best_end = len(ranges)

        ####################################################
        # Aim At Center Of Gap
        ####################################################

        target_index = (best_start + best_end) // 2

        original_index = target_index + start

        steering = (
            scan.angle_min
            + original_index * scan.angle_increment
        )

        ####################################################
        # Speed Selection
        ####################################################

        steering_abs = abs(steering)

        if steering_abs < math.radians(10):
            speed = self.max_speed

        elif steering_abs < math.radians(20):
            speed = self.medium_speed

        else:
            speed = self.min_speed

        ####################################################
        # Return Command
        ####################################################

        return speed, steering
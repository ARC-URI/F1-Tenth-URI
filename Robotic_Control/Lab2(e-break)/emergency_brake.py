import math


class EmergencyBrake:
    """
    Emergency braking system using the RoboRacer iTTC formula.
    """

    def __init__(self):

        print("E-Braking Initialized")

        # Brake if Time-To-Collision falls below this value
        self.ttc_threshold = 0.85  # seconds

        # Ignore tiny LiDAR readings caused by sensor noise
        self.minimum_valid_distance = 0.05  # meters

    def evaluate(self, state):
        """
        Main entry point.

        Returns:
            True  -> Brake immediately
            False -> Continue normal driving
        """

        minimum_ttc = self.calculate_ttc(state)

        # print({minimum_ttc})

        return self.should_brake(minimum_ttc)

    def calculate_ttc(self, state):
        """
        Calculates the minimum Time-To-Collision (iTTC)
        across all LiDAR beams.
        """

        scan = state.scan
        speed = state.speed

        # print(speed)

        if scan is None:
            return None

        minimum_ttc = float("inf")

        # Evaluate every LiDAR beam
        for beam_index, beam_distance in enumerate(scan.ranges):

            # Ignore invalid measurements
            if math.isnan(beam_distance) or math.isinf(beam_distance):
                continue

            # Ignore extremely small values (sensor noise)
            if beam_distance < self.minimum_valid_distance:
                continue

            # Angle of this LiDAR beam
            angle = (
                scan.angle_min
                + beam_index * scan.angle_increment
            )

            # Speed toward the obstacle
            closing_speed = speed * math.cos(angle)

            # Ignore beams that aren't in our direction of travel
            if closing_speed <= 30:
                continue

            # Compute Time-To-Collision
            ttc = beam_distance / closing_speed

            # Keep the smallest TTC
            minimum_ttc = min(minimum_ttc, ttc)

        # No valid TTC could be calculated
        if minimum_ttc == float("inf"):
            return None

        return minimum_ttc

    def should_brake(self, minimum_ttc):
        """
        Decide whether the vehicle should brake.
        """

        if minimum_ttc is None:
            return False

        return minimum_ttc < self.ttc_threshold 
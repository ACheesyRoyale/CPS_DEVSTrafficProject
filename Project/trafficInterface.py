# Traffic states representing different traffic conditions
from pypdevs.infinity import INFINITY


FLUID = 0  # Normal traffic flow, no jams
JAMMED = 1  # Partial traffic jam, some cars are slowed or stopped
FULL_JAM = 2  # Complete traffic jam, no cars are moving
JAMMED_OUTPUT = 3  # Cars are exiting a partial jam
FULL_JAM_OUTPUT = 4  # Cars are exiting a full jam
JAMMED_TO_FULL_JAM = 5  # Transition from a partial to a full traffic jam
IDLE = 6
GENERATING = 7
PEDESTRIAN = 8
UPDATE = 9

# States related to car movement in or out of a jam
JAM_CAR_SPOT_AVAILABLE = 0  # A spot is available in the jam for another car
TO_JAM = 1  # A car is transitioning into a jammed state
TO_FLUID = 2  # A car is transitioning back into fluid (normal) traffic flow


def calculate_time_from_distance_speed(speed, distance):
    """
    t = d/v
    :param speed: speed in km/h
    :param distance: the distance to travel
    :return time in seconds
    """
    
    return distance / (speed * (5.0 / 18.0))
        


def calculate_distance_from_time_speed(speed, time):
    """
    d = v*t
    :param speed: speed in km/h
    :param time: the time in seconds
    :return distance: the distance to travel
    """
    return speed * (5.0 / 18.0) * time

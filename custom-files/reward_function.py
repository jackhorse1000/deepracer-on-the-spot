import math
import numpy as np
from enum import Enum
from copy import copy

MINIMAL_REWARD = 1e-3


class Direction(Enum):
    RIGHT = 1
    LEFT = 2
    STRAIGHT = 3


class PreviousState:
    all_wheels_on_track = None
    x = None
    y = None
    distance_from_center = None
    is_left_of_center = None
    heading = None
    progress = None
    steps = None
    speed = None
    steering_angle = None
    track_width = None
    waypoints = None
    closest_waypoints = None
    is_offtrack = None

    @classmethod
    def reset(cls):
        cls.all_wheels_on_track = None
        cls.x = None
        cls.y = None
        cls.distance_from_center = None
        cls.is_left_of_center = None
        cls.heading = None
        cls.progress = None
        cls.steps = None
        cls.speed = None
        cls.steering_angle = None
        cls.track_width = None
        cls.waypoints = None
        cls.closest_waypoints = None
        cls.is_offtrack = None

    @classmethod
    def update_params(cls, params):
        cls.all_wheels_on_track = params['all_wheels_on_track']
        cls.x = params['x']
        cls.y = params['y']
        cls.distance_from_center = params['distance_from_center']
        cls.is_left_of_center = params['is_left_of_center']
        cls.heading = params['heading']
        cls.progress = params['progress']
        cls.steps = params['steps']
        cls.speed = params['speed']
        cls.steering_angle = params['steering_angle']
        cls.track_width = params['track_width']
        cls.waypoints = params['waypoints']
        cls.closest_waypoints = params['closest_waypoints']
        cls.is_offtrack = params['is_offtrack']


class TrackInfo:
    MIN_SPEED = 2.1
    MAX_SPEED = 4.0
    STANDARD_TIME = 20.0
    FASTEST_TIME = 15.5
    racing_track = [[-0.31623, -3.66329, 4.0, 0.04281],
                    [-0.20228, -3.72585, 4.0, 0.0325],
                    [-0.08832, -3.78842, 4.0, 0.0325],
                    [0.06179, -3.87083, 4.0, 0.04281],
                    [0.32586, -4.01581, 4.0, 0.07531],
                    [0.58992, -4.16079, 4.0, 0.07531],
                    [0.85399, -4.30577, 4.0, 0.07531],
                    [1.11805, -4.45076, 4.0, 0.07531],
                    [1.38236, -4.59531, 4.0, 0.07531],
                    [1.64712, -4.73882, 4.0, 0.07529],
                    [1.91259, -4.8807, 4.0, 0.07525],
                    [2.17899, -5.02027, 4.0, 0.07519],
                    [2.44651, -5.1568, 4.0, 0.07509],
                    [2.71531, -5.28957, 4.0, 0.07495],
                    [2.98554, -5.41774, 4.0, 0.07477],
                    [3.25725, -5.54056, 4.0, 0.07455],
                    [3.5305, -5.65718, 4.0, 0.07428],
                    [3.8053, -5.76677, 4.0, 0.07396],
                    [4.08159, -5.86862, 3.77507, 0.078],
                    [4.35923, -5.96228, 3.39587, 0.08628],
                    [4.63802, -6.04767, 3.02253, 0.09647],
                    [4.91785, -6.12382, 2.6812, 0.10816],
                    [5.19856, -6.18975, 2.32074, 0.12425],
                    [5.4799, -6.24388, 2.32074, 0.12345],
                    [5.76153, -6.2839, 2.32074, 0.12257],
                    [6.04282, -6.30705, 2.32074, 0.12162],
                    [6.32272, -6.30976, 2.32074, 0.12061],
                    [6.59944, -6.2873, 2.32074, 0.11963],
                    [6.86983, -6.23378, 2.32074, 0.11877],
                    [7.12724, -6.14069, 2.34408, 0.11677],
                    [7.36649, -6.01165, 2.54282, 0.1069],
                    [7.58653, -5.85482, 2.76094, 0.09787],
                    [7.78794, -5.67663, 2.94781, 0.09123],
                    [7.97147, -5.48146, 3.09195, 0.08665],
                    [8.13761, -5.27241, 3.2003, 0.08344],
                    [8.28667, -5.05179, 3.24373, 0.08208],
                    [8.41841, -4.82122, 3.38361, 0.07848],
                    [8.53372, -4.58267, 3.40741, 0.07776],
                    [8.63245, -4.33727, 3.19827, 0.0827],
                    [8.71478, -4.08622, 2.92871, 0.09021],
                    [8.78025, -3.83038, 2.69239, 0.09808],
                    [8.82946, -3.57088, 2.39366, 0.11034],
                    [8.86243, -3.30865, 2.1, 0.12586],
                    [8.87905, -3.0446, 2.1, 0.12599],
                    [8.87798, -2.77964, 2.1, 0.12617],
                    [8.85648, -2.51503, 2.1, 0.12642],
                    [8.81061, -2.25279, 2.1, 0.12677],
                    [8.7361, -1.99611, 2.1, 0.12728],
                    [8.62599, -1.75066, 2.1, 0.1281],
                    [8.47198, -1.52741, 2.43466, 0.1114],
                    [8.28788, -1.32592, 2.68642, 0.1016],
                    [8.08067, -1.14506, 2.91379, 0.09439],
                    [7.85487, -0.98355, 3.16483, 0.08772],
                    [7.61404, -0.83981, 3.39044, 0.08272],
                    [7.36067, -0.71262, 3.58855, 0.079],
                    [7.09657, -0.60113, 3.80271, 0.07538],
                    [6.82339, -0.50443, 4.0, 0.07245],
                    [6.54264, -0.42144, 3.95021, 0.07411],
                    [6.25559, -0.35133, 3.31861, 0.08904],
                    [5.96343, -0.29324, 3.10842, 0.09583],
                    [5.66724, -0.24629, 3.10842, 0.09648],
                    [5.36759, -0.21119, 3.10842, 0.09706],
                    [5.06533, -0.1889, 3.10842, 0.09751],
                    [4.76184, -0.18157, 3.10842, 0.09766],
                    [4.48854, -0.19, 3.10842, 0.08796],
                    [4.22073, -0.17868, 3.10842, 0.08623],
                    [3.95624, -0.1456, 3.15468, 0.08449],
                    [3.69264, -0.09095, 3.409, 0.07897],
                    [3.42921, -0.01696, 3.55101, 0.07705],
                    [3.16952, 0.07283, 3.55101, 0.07738],
                    [2.91138, 0.18143, 4.0, 0.07001],
                    [2.65006, 0.30199, 4.0, 0.07195],
                    [2.39412, 0.42876, 4.0, 0.0714],
                    [2.14048, 0.56926, 3.51644, 0.08246],
                    [1.88897, 0.71075, 3.51644, 0.08206],
                    [1.64726, 0.86406, 3.51644, 0.0814],
                    [1.40777, 1.02198, 3.51644, 0.08158],
                    [1.18131, 1.18764, 3.38242, 0.08295],
                    [0.96152, 1.36551, 3.12079, 0.0906],
                    [0.75106, 1.54892, 3.12079, 0.08945],
                    [0.557, 1.74215, 3.12079, 0.08775],
                    [0.37434, 1.94838, 3.12079, 0.08828],
                    [0.20411, 2.16416, 3.12079, 0.08807],
                    [0.0499, 2.38834, 3.12079, 0.08719],
                    [-0.08654, 2.61998, 3.12079, 0.08614],
                    [-0.20307, 2.8609, 3.23418, 0.08275],
                    [-0.30169, 3.11208, 4.0, 0.06746],
                    [-0.40722, 3.37393, 4.0, 0.07058],
                    [-0.51864, 3.63443, 4.0, 0.07083],
                    [-0.63914, 3.89212, 4.0, 0.07112],
                    [-0.77077, 4.14518, 3.72897, 0.07649],
                    [-0.91388, 4.39205, 3.41883, 0.08347],
                    [-1.06784, 4.63183, 3.0402, 0.09373],
                    [-1.23195, 4.86395, 3.0402, 0.0935],
                    [-1.40571, 5.08791, 3.0402, 0.09324],
                    [-1.58932, 5.30275, 3.0402, 0.09296],
                    [-1.78309, 5.50735, 3.0402, 0.09269],
                    [-1.98826, 5.69943, 3.0402, 0.09244],
                    [-2.20617, 5.87602, 3.0402, 0.09226],
                    [-2.43878, 6.03208, 3.29099, 0.08511],
                    [-2.6826, 6.16979, 3.47693, 0.08054],
                    [-2.93547, 6.29046, 3.62402, 0.07731],
                    [-3.19579, 6.395, 3.74632, 0.07488],
                    [-3.46229, 6.48403, 3.75652, 0.0748],
                    [-3.73402, 6.55754, 3.75652, 0.07494],
                    [-4.01016, 6.61492, 3.81572, 0.07391],
                    [-4.28959, 6.65647, 3.81572, 0.07404],
                    [-4.5713, 6.68238, 3.81572, 0.07414],
                    [-4.85434, 6.69214, 3.81572, 0.07422],
                    [-5.13767, 6.68552, 3.55468, 0.07973],
                    [-5.42026, 6.66297, 3.26348, 0.08687],
                    [-5.70127, 6.62563, 2.93351, 0.09663],
                    [-5.98003, 6.57456, 2.58878, 0.10947],
                    [-6.25571, 6.5095, 2.58878, 0.10941],
                    [-6.52735, 6.4301, 2.58878, 0.10932],
                    [-6.79372, 6.33546, 2.58878, 0.1092],
                    [-7.05282, 6.22353, 2.58878, 0.10902],
                    [-7.30178, 6.09187, 2.58878, 0.10879],
                    [-7.53604, 5.9372, 2.58878, 0.10843],
                    [-7.74776, 5.75611, 2.88612, 0.09653],
                    [-7.93889, 5.55671, 3.07258, 0.0899],
                    [-8.11073, 5.34318, 3.21821, 0.08517],
                    [-8.2643, 5.11834, 3.3161, 0.08211],
                    [-8.40024, 4.88412, 3.395, 0.07977],
                    [-8.51905, 4.64202, 3.39279, 0.07949],
                    [-8.62114, 4.39327, 3.12809, 0.08596],
                    [-8.70686, 4.1389, 2.87924, 0.09323],
                    [-8.77649, 3.87981, 2.55133, 0.10516],
                    [-8.8299, 3.61675, 2.27273, 0.11811],
                    [-8.86673, 3.35047, 2.27273, 0.11828],
                    [-8.8862, 3.08174, 2.27273, 0.11855],
                    [-8.88692, 2.8114, 2.27273, 0.11895],
                    [-8.86531, 2.54058, 2.27273, 0.11954],
                    [-8.81702, 2.27123, 2.27273, 0.12041],
                    [-8.73445, 2.00716, 2.27273, 0.12174],
                    [-8.60908, 1.75628, 2.89377, 0.09692],
                    [-8.4581, 1.51725, 3.1622, 0.08941],
                    [-8.28612, 1.28994, 3.50898, 0.08123],
                    [-8.09741, 1.07338, 3.81378, 0.07532],
                    [-7.89485, 0.86683, 4.0, 0.07233],
                    [-7.68102, 0.66938, 4.0, 0.07276],
                    [-7.45787, 0.48031, 4.0, 0.07312],
                    [-7.22699, 0.29894, 4.0, 0.0734],
                    [-6.98968, 0.12464, 4.0, 0.07361],
                    [-6.74704, -0.04329, 4.0, 0.07377],
                    [-6.49999, -0.20567, 4.0, 0.07391],
                    [-6.24925, -0.36338, 4.0, 0.07405],
                    [-5.99535, -0.51737, 4.0, 0.07424],
                    [-5.73863, -0.66855, 4.0, 0.07448],
                    [-5.47944, -0.81778, 4.0, 0.07477],
                    [-5.21817, -0.96567, 4.0, 0.07506],
                    [-4.95531, -1.11264, 4.0, 0.07529],
                    [-4.69154, -1.25895, 4.0, 0.07541],
                    [-4.42745, -1.40486, 4.0, 0.07543],
                    [-4.1633, -1.55047, 4.0, 0.07541],
                    [-3.89919, -1.69579, 4.0, 0.07536],
                    [-3.63532, -1.8409, 4.0, 0.07528],
                    [-3.37123, -1.98604, 4.0, 0.07534],
                    [-3.10709, -2.13112, 4.0, 0.07534],
                    [-2.84296, -2.27612, 4.0, 0.07533],
                    [-2.57887, -2.42108, 4.0, 0.07531],
                    [-2.31481, -2.56603, 4.0, 0.07531],
                    [-2.05075, -2.71099, 4.0, 0.07531],
                    [-1.78669, -2.85597, 4.0, 0.07531],
                    [-1.52263, -3.00096, 4.0, 0.07531],
                    [-1.25856, -3.14594, 4.0, 0.07531],
                    [-0.99449, -3.29092, 4.0, 0.07531],
                    [-0.73042, -3.4359, 4.0, 0.07531],
                    [-0.46635, -3.58088, 4.0, 0.07531]]


class GlobalParams:
    progress_reward_list = None
    prev_direction_diff = None
    first_racingpoint_index = None

    @classmethod
    def reset(cls):
        cls.progress_reward_list = None
        cls.prev_direction_diff = None
        cls.first_racingpoint_index = None


class Reward:
    def __init__(self, verbose=True):
        self.verbose = verbose

    def reward_function(self, params):

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1 - x2) ** 2 + abs(y1 - y2) ** 2) ** 0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):

            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a ** 4) + 2 * (a ** 2) * (b ** 2) + 2 * (a ** 2) * (c ** 2) -
                               (b ** 4) + 2 * (b ** 2) * (c ** 2) - (c ** 4)) ** 0.5 / (2 * a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0] + heading_vector[0],
                              car_coords[1] + heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radians, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the raw difference between the track direction and the heading direction of the car
            raw_direction_diff = track_direction - heading

            # Normalize the difference to the range [-180, 180]
            if raw_direction_diff > 180:
                raw_direction_diff -= 360
            elif raw_direction_diff < -180:
                raw_direction_diff += 360

            direction = Direction.RIGHT if raw_direction_diff > 0 else Direction.LEFT
            if -2 > raw_direction_diff > 2:
                direction = Direction.STRAIGHT
            direction_diff = abs(raw_direction_diff)

            return direction_diff, direction

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):
            if start is not None and end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def calc_projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = max(0, (step_count - 1)) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                result = (current_actual_time / current_expected_time) * total_expected_time
            except:
                result = 9999

            return result
        
        def position_relative_to_race_line(car_coords, closest_coords, second_closest_coords, heading):
            
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)
            
            x1, y1 = prev_point
            x2, y2 = next_point
            car_x, car_y = car_coords

            z = (x2 - x1) * (car_y - y1) - (y2 - y1) * (car_x - x1)

            if z > 0:
                return Direction.RIGHT
            elif z < 0:
                return Direction.LEFT
            return Direction.STRAIGHT

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ################## PREVIOUS STATE PARAMETERS ###################
        if PreviousState.steps is None or steps < PreviousState.steps:
            PreviousState.reset()
            GlobalParams.reset()

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            TrackInfo.racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = TrackInfo.racing_track[closest_index]
        optimals_second = TrackInfo.racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if GlobalParams.first_racingpoint_index is None:
            GlobalParams.first_racingpoint_index = closest_index
            
            print("first_racingpoint_index is set to: ", GlobalParams.first_racingpoint_index)

        ############### HELPER VARIABLES ################

        def angle_between_vectors(u, v):
            dot_product = u[0] * v[0] + u[1] * v[1]
            determinant = u[0] * v[1] - u[1] * v[0]
            return math.degrees(math.atan2(determinant, dot_product))

        # TODO: Determine if straight, right or left turn
        # Figure out if car is heading straight, left or right
        def track_lookahed_degree_turns(closest_index, lookahead=5):
            coords = []
            for i in range(0, lookahead):
                current_index = (closest_index + i) % len(TrackInfo.racing_track)
                current_point = TrackInfo.racing_track[current_index]
                coords.append([current_point[0], current_point[1]])
            vectors = [(coords[i + 1][0] - coords[i][0], coords[i + 1][1] - coords[i][1]) for i in
                       range(len(coords) - 1)]

            angles = [angle_between_vectors(vectors[i], vectors[i + 1]) for i in range(len(vectors) - 1)]

            total_angle = sum(angles)

            return total_angle

        def get_track_direction(closest_index, lookahead=5):
            degrees_turned = track_lookahed_degree_turns(closest_index, lookahead)
            # print(degrees_turned)
            if -1 < degrees_turned < 1:
                return Direction.STRAIGHT
            elif degrees_turned > 1:
                return Direction.RIGHT
            else:
                return Direction.LEFT

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ################ SPEED ################
        # optimal_speed = min(TrackInfo.racing_track[closest_index][2], TrackInfo.MAX_SPEED)
        # sigma_speed = abs(TrackInfo.MAX_SPEED - TrackInfo.MIN_SPEED) / 6.0
        # speed_diff = abs(optimal_speed - speed)
        # speed_reward = math.exp(-0.5 * abs(speed_diff) ** 2 / sigma_speed ** 2)

        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = MINIMAL_REWARD
        # reward += speed_reward * SPEED_MULTIPLE

        ################ DISTANCE ################

        # distance_reward = 0
        # TODO: Figure out
        # if "center" in bearing: #i.e. on the route line
        #     distance_from_route = 0
        #     distance_reward = 1
        # elif "right" in bearing: #i.e. on right side of the route line
        #     sigma=abs(normalized_route_distance_from_inner_border / 4) 
        #     distance_reward = math.exp(-0.5*abs(normalized_car_distance_from_route)**2/sigma**2)
        # elif "left" in bearing: #i.e. on left side of the route line
        #     sigma=abs(normalized_route_distance_from_outer_border / 4) 
        #     distance_reward = math.exp(-0.5*abs(normalized_car_distance_from_route)**2/sigma**2)

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 4
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(MINIMAL_REWARD, 1 - (dist / (track_width * 0.5)))

        # reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##

        ################ HEADING ################
        racing_direction_diff, direction_to_align_with_track = racing_direction_diff(optimals[0:2],
                                                                                     optimals_second[0:2], [x, y],
                                                                                     heading) 
        # abs_heading_reward = 1 - (racing_direction_diff / 180.0)
        # heading_reward = abs_heading_reward
        
        # Reward if steering angle is aligned with direction difference
        # abs_steering_reward = 1 - (abs(steering_angle - racing_direction_diff) / 180.0)
        # steering_reward = abs_steering_reward

        ################ CURVE SECTION BONUS ################
        # TODO: Reward for following the curve correctly. The sharper the curve the more reward

        ################ STRAIGHT SECTION BONUS ################
        # TODO: Reward for following the line correctly at high speeds, without going off course.
        #  Faster the bigger the reward

        ############# INCREMENTAL PROGRESS REWARD ##############

        # Reward for making steady progress
        # SECTIONS = 10
        # if steps <= 5:
        #     progress_reward = MINIMAL_REWARD  # ignore progress in the first 5 steps
        # else:
        #     progress_reward = max(MINIMAL_REWARD, 5 * progress / steps)

        # Bonus that the agent gets for completing every 10 percent of track
        # Is exponential in the progress / steps. 
        # exponent increases with an increase in fraction of lap completed

        # intermediate_progress_bonus = 0.0
        

        # if pi != 0 and GlobalParams.progress_reward_list[pi] == 0:
        #     if pi == int(100 // SECTIONS):  # 100% track completion
        #         intermediate_progress_bonus = progress_reward ** 14
        #     else:
        #         intermediate_progress_bonus = progress_reward ** (5 + 0.75 * pi)
        #     GlobalParams.progress_reward_list[pi] = intermediate_progress_bonus

        ################ LESS STEPS REWARD ################

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        times_list = [row[3] for row in TrackInfo.racing_track]
        projected_time = calc_projected_time(GlobalParams.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(MINIMAL_REWARD, (-REWARD_PER_STEP_FOR_FASTEST_TIME * (TrackInfo.FASTEST_TIME) /
                                                     (TrackInfo.STANDARD_TIME - TrackInfo.FASTEST_TIME)) * (
                                                steps_prediction - (TrackInfo.STANDARD_TIME * 15 + 1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = MINIMAL_REWARD

        # TODO: Review and commented code
        # TODO: Add to punishment
        ################ CHECKPOINT REWARD ################
        
        def get_time_between_two_indicies(starting_index, current_index):
            assert 0 <= starting_index < len(TrackInfo.racing_track)
            assert 0 <= current_index <len(TrackInfo.racing_track)
            
            result = 0.0
            if starting_index == current_index:
                return result

            counter_index = copy(starting_index) + 1
            
            while counter_index != current_index:
                result += TrackInfo.racing_track[counter_index][3]
                counter_index = (counter_index + 1) % len(TrackInfo.racing_track)
                
            result += TrackInfo.racing_track[current_index][3]
            return result
        
        SECTIONS = 10
        pi = int(progress // SECTIONS)

        if GlobalParams.progress_reward_list is None:
            GlobalParams.progress_reward_list = [0] * (SECTIONS + 1)
        
        checkpoint_reward = 0.0
        REWARD_FOR_FASTEST_CHECKPOINT_TIME = 200
        standard_multiplier = float(TrackInfo.STANDARD_TIME)/float(TrackInfo.FASTEST_TIME)
        if pi != 0 and GlobalParams.progress_reward_list[pi] == 0 and steps != 0 and GlobalParams.first_racingpoint_index != closest_index:
            fastest_time = get_time_between_two_indicies(GlobalParams.first_racingpoint_index, closest_index)
            standard_time = standard_multiplier * fastest_time
            checkpoint_reward = max(MINIMAL_REWARD, (-REWARD_FOR_FASTEST_CHECKPOINT_TIME / (15 * (standard_time - fastest_time))) * (steps - standard_time * 15))
            GlobalParams.progress_reward_list[pi] = checkpoint_reward


            

        ################ FINAL REWARD ################

        # Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 3000  # should be adapted to track length and other rewards
        if progress == 100:
            finish_reward = max(MINIMAL_REWARD, (-REWARD_FOR_FASTEST_TIME /
                                                 (15 * (TrackInfo.STANDARD_TIME - TrackInfo.FASTEST_TIME))) * (steps - TrackInfo.STANDARD_TIME * 15))
        else:
            finish_reward = 0

        #################### UNFORGIVALABLE ACTIONS ####################
        unforgivable_action = False
        # TODO: Distance from center line exceed half the track width

        # TODO: Direction diff exceeds 30 degrees
        # Zero reward if obviously wrong direction (e.g. spin)
        
        if racing_direction_diff > 30:
            print("Unforgivable action racing direction %f > 30" % racing_direction_diff)
            unforgivable_action = True
        # TODO: The car is currently heading in the right direction but takes a steering action that causes it to point off-course.
        # TODO: The car turns to the left when it should be taking a right turn.
        # if get_track_direction(closest_index) == Direction.RIGHT and steering_angle < -5:
        #     print("Unforgivable action. Track goes Right. Action is Light.")
        #     unforgivable_action = True
        #
        # if get_track_direction(closest_index) == Direction.LEFT and steering_angle < -5:
        #     print("Unforgivable action. Track goes Left. Action is Right.")
        #     unforgivable_action = True
        
        # def compute_tolerance(track_width, alpha=0.2):
        #     return alpha * track_width

        # car_direction_relative_to_racing_line = position_relative_to_race_line(optimals[0:2], optimals_second[0:2], [x, y], heading)
        # # TODO: The car turns to the right when it should be taking a left turn.
        # if car_direction_relative_to_racing_line == Direction.LEFT and dist > compute_tolerance(track_width) and not (steering_angle < 2) \
        #     and get_track_direction(closest_index) != Direction.LEFT:
        #     # if direction_to_align_with_track == Direction.RIGHT and not (steering_angle < 2):
        #         print("Unforgivable action. Should turn right. Action is left. %f angle diff, %f steering angle", racing_direction_diff, steering_angle)
        #         unforgivable_action = True

        # if car_direction_relative_to_racing_line == Direction.RIGHT and dist > compute_tolerance(track_width) and not (steering_angle > -2) \
        #     and get_track_direction(closest_index) != Direction.RIGHT:
        #     # if direction_to_align_with_track == Direction.LEFT and not (steering_angle > -2):
        #         print("Unforgivable action. Should turn left. Action is right. %f angle diff, %f steering angle", racing_direction_diff, steering_angle)
        #         unforgivable_action = True
        
        # TODO: Add when trying to optimise further
        # if (closest_waypoints[0] > 144 or closest_waypoints[0] < 15) and speed < 3.5:
        #     print("Unforgivable action main staight speed < 3.5")
        #     unforgivable_action = True

        # TODO: The car’s speed is at least 1 m/s greater than its optimal speed while it is making a turn. Essentially the car is turning too fast.
        speed_diff = optimals[2] - speed
        # if speed_diff < -0.5 and get_track_direction(closest_index) != Direction.STRAIGHT:
        #     print("Unforgivable action speed difference on turn %f < -0.5" % speed_diff)
        #     unforgivable_action = True
        # # TODO: The speed of the car is 1.5 m/s slower than its optimal speed on a straight section. Essentially the car is going too slow on straight sections.
        
        if speed_diff > 1 and get_track_direction(closest_index) == Direction.STRAIGHT:
            print("Unforgivable action speed difference on straight %f > 1.0" % speed_diff)
            unforgivable_action = True
            
        if speed_diff > 1.5:
            print("Unforgivable action speed difference %f > 1.5" % speed_diff)
            unforgivable_action = True
            
        # Distance from racing line punishment
        if dist > 0.5 and get_track_direction(closest_index) != Direction.STRAIGHT:
            print("Unforgivable action distance from racing line on turn %f > 0.5" % dist)
            unforgivable_action = True

        if dist > 0.3 and get_track_direction(closest_index) == Direction.STRAIGHT:
            print("Unforgivable action distance from racing line on straight %f > 0.3" % dist)
            unforgivable_action = True
            

        ## Zero reward if off track ##
        if not all_wheels_on_track:
            print("Unforgivable action all_wheels_on_track = %s" % all_wheels_on_track)
            unforgivable_action = True

        if GlobalParams.prev_direction_diff is not None and \
                abs(racing_direction_diff) > 30 and (abs(racing_direction_diff) > abs(GlobalParams.prev_direction_diff)):
            print("FAR AWAY FROM DIRECTION AND GETTING WORST: %f %f" % racing_direction_diff, GlobalParams.prev_direction_diff)
            unforgivable_action = True
            
        
        ################ REWARD ################
        reward += speed_reward * SPEED_MULTIPLE if speed_reward != MINIMAL_REWARD else MINIMAL_REWARD
        reward += distance_reward * DISTANCE_MULTIPLE if distance_reward != MINIMAL_REWARD else MINIMAL_REWARD
        # reward += heading_reward
        # reward += steering_reward
        reward += steps_reward
        
        if unforgivable_action:
            reward = MINIMAL_REWARD
        
        reward += checkpoint_reward
        reward += finish_reward
        

        #################### UPDATE PREVIOUS STATE ####################
        PreviousState.update_params(params)

        ####################### VERBOSE #######################

        # TODO: Update logging
        print("Reward: %f" % reward)
        if self.verbose:
            print("Speed Reward: %f" % (speed_reward * SPEED_MULTIPLE if speed_reward != MINIMAL_REWARD else MINIMAL_REWARD))
            print("Distance Reward: %f" % (distance_reward * DISTANCE_MULTIPLE if distance_reward != MINIMAL_REWARD else MINIMAL_REWARD))
            # print("Heading Reward: %f" % heading_reward)
            # print("Steering Reward: %f" % steering_reward)

            # print("Progress Reward: %f" % progress_reward)
            print("Steps Reward: %f" % steps_reward)
            if finish_reward > 0:
                print("Finish Reward: %f" % finish_reward)
            if checkpoint_reward > 0:
                print("Checkpoint Reward: %f" % checkpoint_reward)

            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


reward_object = Reward()  # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)


def get_test_params():
    return {
        'x': -0.11087, 
        'y': -3.28924,
        'heading': 160.0,
        'track_width': 0.45,
        'is_reversed': False,
        'steering_angle': 0.0,
        'all_wheels_on_track': False,
        'progress': 0,
        'steps': 0,
        'distance_from_center': 0.0,
        'closest_waypoints': [0, 1, 2],
        'is_left_of_center': False,
        'speed': 2.0,
        'is_offtrack': False,
        'waypoints': [
            [0.75, -0.7],
            [1.0, 0.0],
            [0.7, 0.52],
            [0.58, 0.7],
            [0.48, 0.8],
            [0.15, 0.95],
            [-0.1, 1.0],
            [-0.7, 0.75],
            [-0.9, 0.25],
            [-0.9, -0.55],
        ]
    }


def test_reward():
    params = get_test_params()

    reward = reward_function(params)

    print("test_reward: {}".format(reward))

    assert reward > 0.0


# test_reward()

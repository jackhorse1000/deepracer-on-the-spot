import math
import numpy as np
from enum import Enum

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
    MIN_SPEED = 2
    MAX_SPEED = 4
    STANDARD_TIME = 20
    FASTEST_TIME = 15
    racing_track = [[-0.11087, -3.28924, 4.0, 0.04281],
                    [0.00309, -3.3518, 4.0, 0.0325],
                    [0.11704, -3.41436, 4.0, 0.0325],
                    [0.26716, -3.49678, 4.0, 0.04281],
                    [0.53122, -3.64176, 4.0, 0.07531],
                    [0.79529, -3.78674, 4.0, 0.07531],
                    [1.05936, -3.93172, 4.0, 0.07531],
                    [1.32342, -4.07671, 4.0, 0.07531],
                    [1.58749, -4.22169, 4.0, 0.07531],
                    [1.85157, -4.36664, 4.0, 0.07531],
                    [2.11566, -4.51158, 4.0, 0.07531],
                    [2.37975, -4.65651, 4.0, 0.07531],
                    [2.6438, -4.80153, 4.0, 0.07531],
                    [2.9078, -4.94666, 4.0, 0.07531],
                    [3.1719, -5.09111, 4.0, 0.07526],
                    [3.43619, -5.23401, 4.0, 0.07511],
                    [3.70083, -5.37432, 4.0, 0.07488],
                    [3.96595, -5.51096, 4.0, 0.07456],
                    [4.23165, -5.64284, 3.88362, 0.07638],
                    [4.49803, -5.76868, 3.40003, 0.08665],
                    [4.76514, -5.8871, 3.01951, 0.09677],
                    [5.033, -5.99659, 2.67777, 0.10806],
                    [5.30156, -6.09527, 2.34925, 0.12179],
                    [5.57071, -6.1809, 2.08155, 0.13569],
                    [5.84017, -6.25066, 2.08155, 0.13372],
                    [6.1094, -6.30053, 2.08155, 0.13154],
                    [6.37739, -6.32622, 2.08155, 0.12934],
                    [6.64241, -6.3225, 2.08155, 0.12733],
                    [6.90131, -6.28217, 2.08155, 0.12588],
                    [7.14812, -6.19703, 2.33719, 0.11171],
                    [7.3816, -6.07749, 2.52502, 0.10388],
                    [7.60098, -5.92907, 2.70307, 0.09799],
                    [7.80571, -5.75591, 2.87195, 0.09336],
                    [7.99531, -5.56135, 3.0244, 0.08982],
                    [8.16924, -5.34827, 3.19277, 0.08615],
                    [8.32731, -5.11961, 3.29774, 0.08429],
                    [8.46885, -4.87768, 3.38103, 0.0829],
                    [8.5932, -4.62484, 3.39626, 0.08296],
                    [8.69963, -4.36341, 3.31111, 0.08525],
                    [8.78748, -4.09568, 3.21962, 0.08752],
                    [8.85639, -3.82383, 2.9544, 0.09492],
                    [8.90615, -3.54985, 2.72004, 0.10238],
                    [8.93671, -3.27544, 2.426, 0.11381],
                    [8.94787, -3.00215, 2.1774, 0.12561],
                    [8.93917, -2.73152, 2.0, 0.13539],
                    [8.91012, -2.46504, 2.0, 0.13403],
                    [8.85783, -2.20499, 2.0, 0.13263],
                    [8.77938, -1.95434, 2.0, 0.13132],
                    [8.66981, -1.718, 2.0, 0.13025],
                    [8.52415, -1.50305, 2.0, 0.12983],
                    [8.33939, -1.31892, 2.22115, 0.11744],
                    [8.1259, -1.16416, 2.49379, 0.10573],
                    [7.8913, -1.03551, 2.79066, 0.09588],
                    [7.64081, -0.92969, 3.13745, 0.08667],
                    [7.37837, -0.84336, 3.54852, 0.07786],
                    [7.10696, -0.77323, 4.0, 0.07008],
                    [6.82909, -0.71577, 4.0, 0.07094],
                    [6.54657, -0.668, 4.0, 0.07163],
                    [6.26102, -0.62682, 4.0, 0.07213],
                    [5.974, -0.58891, 4.0, 0.07238],
                    [5.6843, -0.54794, 4.0, 0.07315],
                    [5.39532, -0.50406, 4.0, 0.07307],
                    [5.10722, -0.45695, 4.0, 0.07298],
                    [4.82015, -0.40622, 4.0, 0.07288],
                    [4.53437, -0.35122, 4.0, 0.07276],
                    [4.25025, -0.29104, 4.0, 0.07261],
                    [3.96824, -0.22461, 4.0, 0.07243],
                    [3.68886, -0.15089, 4.0, 0.07224],
                    [3.41299, -0.06806, 4.0, 0.07201],
                    [3.14205, 0.02635, 4.0, 0.07173],
                    [2.87677, 0.13277, 4.0, 0.07146],
                    [2.61679, 0.24944, 4.0, 0.07124],
                    [2.36202, 0.37556, 4.0, 0.07107],
                    [2.1124, 0.5105, 4.0, 0.07094],
                    [1.86788, 0.65376, 3.77757, 0.07502],
                    [1.62843, 0.80493, 3.60507, 0.07855],
                    [1.39434, 0.96405, 3.60507, 0.07851],
                    [1.16618, 1.13152, 3.60507, 0.07851],
                    [0.94493, 1.30815, 3.60507, 0.07853],
                    [0.73179, 1.4948, 3.60507, 0.07859],
                    [0.52962, 1.69386, 3.60507, 0.0787],
                    [0.34031, 1.90587, 3.7599, 0.07559],
                    [0.16352, 2.12915, 4.0, 0.0712],
                    [-0.00224, 2.36149, 4.0, 0.07135],
                    [-0.15862, 2.60104, 4.0, 0.07152],
                    [-0.30706, 2.84633, 4.0, 0.07168],
                    [-0.44876, 3.09625, 4.0, 0.07182],
                    [-0.58477, 3.34979, 4.0, 0.07193],
                    [-0.71585, 3.60608, 4.0, 0.07197],
                    [-0.84265, 3.86423, 3.46522, 0.083],
                    [-0.96558, 4.12338, 2.92637, 0.09802],
                    [-1.08499, 4.38269, 2.58062, 0.11063],
                    [-1.20462, 4.63579, 2.58062, 0.10848],
                    [-1.32991, 4.88413, 2.58062, 0.10779],
                    [-1.46472, 5.12435, 2.58062, 0.10674],
                    [-1.61354, 5.35226, 2.58062, 0.10548],
                    [-1.78089, 5.56289, 2.58062, 0.10424],
                    [-1.97028, 5.75066, 2.75467, 0.09681],
                    [-2.17568, 5.91993, 2.75467, 0.09662],
                    [-2.39689, 6.06929, 2.75467, 0.09689],
                    [-2.63438, 6.19597, 3.05821, 0.08801],
                    [-2.8845, 6.30316, 3.27606, 0.08306],
                    [-3.14525, 6.39244, 3.49798, 0.07879],
                    [-3.41501, 6.46512, 3.74166, 0.07467],
                    [-3.69227, 6.52251, 3.97031, 0.07131],
                    [-3.97573, 6.56563, 4.0, 0.07168],
                    [-4.26402, 6.59553, 4.0, 0.07246],
                    [-4.55572, 6.61298, 4.0, 0.07306],
                    [-4.84936, 6.61847, 4.0, 0.07342],
                    [-5.14342, 6.61261, 3.78024, 0.0778],
                    [-5.43645, 6.5954, 3.51637, 0.08348],
                    [-5.72714, 6.56692, 3.08969, 0.09454],
                    [-6.01426, 6.5268, 2.76215, 0.10496],
                    [-6.29663, 6.47442, 2.45577, 0.11694],
                    [-6.57303, 6.40869, 2.45577, 0.11569],
                    [-6.84201, 6.32795, 2.45577, 0.11436],
                    [-7.10216, 6.23062, 2.45577, 0.11311],
                    [-7.35053, 6.11273, 2.45577, 0.11195],
                    [-7.58352, 5.97027, 2.45577, 0.1112],
                    [-7.79546, 5.79819, 2.71059, 0.10072],
                    [-7.9883, 5.60296, 2.92913, 0.09368],
                    [-8.16311, 5.38891, 3.14505, 0.08787],
                    [-8.32075, 5.15967, 3.29217, 0.08451],
                    [-8.46134, 4.91792, 3.42287, 0.0817],
                    [-8.58498, 4.6661, 3.3922, 0.0827],
                    [-8.6916, 4.40634, 3.20141, 0.08771],
                    [-8.78115, 4.14056, 3.00601, 0.0933],
                    [-8.85385, 3.87051, 2.63303, 0.10621],
                    [-8.90941, 3.59767, 2.35958, 0.118],
                    [-8.94731, 3.32344, 2.10536, 0.13149],
                    [-8.96567, 3.04924, 2.10536, 0.13053],
                    [-8.96249, 2.77681, 2.10536, 0.12941],
                    [-8.93556, 2.50816, 2.10536, 0.12824],
                    [-8.87877, 2.24677, 2.10536, 0.12705],
                    [-8.78637, 1.99743, 2.10536, 0.1263],
                    [-8.65109, 1.76749, 2.59185, 0.10293],
                    [-8.48816, 1.5528, 2.83873, 0.09494],
                    [-8.30206, 1.35234, 3.11481, 0.08782],
                    [-8.09642, 1.165, 3.4317, 0.08106],
                    [-7.87443, 0.98954, 3.86193, 0.07327],
                    [-7.63931, 0.82429, 4.0, 0.07184],
                    [-7.39382, 0.66765, 4.0, 0.0728],
                    [-7.14022, 0.51809, 4.0, 0.0736],
                    [-6.88076, 0.3738, 4.0, 0.07422],
                    [-6.61729, 0.233, 4.0, 0.07468],
                    [-6.35165, 0.09388, 4.0, 0.07497],
                    [-6.0854, -0.04677, 4.0, 0.07528],
                    [-5.81947, -0.18807, 4.0, 0.07528],
                    [-5.55369, -0.32968, 4.0, 0.07529],
                    [-5.28805, -0.47154, 4.0, 0.07529],
                    [-5.02252, -0.61363, 4.0, 0.07529],
                    [-4.75712, -0.75596, 4.0, 0.07529],
                    [-4.49183, -0.89852, 4.0, 0.07529],
                    [-4.22666, -1.04133, 4.0, 0.07529],
                    [-3.96161, -1.18436, 4.0, 0.0753],
                    [-3.69667, -1.32762, 4.0, 0.0753],
                    [-3.43185, -1.47112, 4.0, 0.0753],
                    [-3.16715, -1.61484, 4.0, 0.0753],
                    [-2.90257, -1.7588, 4.0, 0.0753],
                    [-2.6381, -1.90298, 4.0, 0.07531],
                    [-2.37374, -2.0474, 4.0, 0.07531],
                    [-2.10951, -2.19205, 4.0, 0.07531],
                    [-1.84539, -2.33694, 4.0, 0.07531],
                    [-1.58133, -2.48192, 4.0, 0.07531],
                    [-1.31726, -2.6269, 4.0, 0.07531],
                    [-1.05319, -2.77189, 4.0, 0.07531],
                    [-0.78912, -2.91687, 4.0, 0.07531],
                    [-0.52505, -3.06184, 4.0, 0.07531],
                    [-0.26098, -3.20682, 4.0, 0.07531]]


class GlobalParams:
    progress_reward_list = None
    prev_direction_diff = None

    @classmethod
    def reset(cls):
        cls.progress_reward_list = None


class Reward:
    def __init__(self, verbose=True):
        self.first_racingpoint_index = None
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
        if self.first_racingpoint_index is None:
            self.first_racingpoint_index = closest_index
            print("first_racingpoint_index is set to: ", self.first_racingpoint_index)

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
        SPEED_MULTIPLE = 4
        speed_diff = abs(optimals[2] - speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff / (SPEED_DIFF_NO_REWARD)) ** 2) ** 2
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
        abs_heading_reward = 1 - (racing_direction_diff / 180.0)
        heading_reward = abs_heading_reward

        # Reward if steering angle is aligned with direction difference
        abs_steering_reward = 1 - (abs(steering_angle - racing_direction_diff) / 180.0)
        steering_reward = abs_steering_reward

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
        # pi = int(progress // SECTIONS)

        # if GlobalParams.progress_reward_list is None:
        #     GlobalParams.progress_reward_list = [0] * (SECTIONS + 1)

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
        projected_time = calc_projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
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

        ################ FINAL REWARD ################

        # Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500  # should be adapted to track length and other rewards
        if progress == 100:
            finish_reward = max(MINIMAL_REWARD, (-REWARD_FOR_FASTEST_TIME /
                                                 (15 * (TrackInfo.STANDARD_TIME - TrackInfo.FASTEST_TIME))) * (
                                        steps - TrackInfo.STANDARD_TIME * 15))
        else:
            finish_reward = 0

        ################ SUM UP REWARD ################
        reward += speed_reward * SPEED_MULTIPLE
        reward += distance_reward * DISTANCE_MULTIPLE
        reward += heading_reward
        reward += steering_reward

        # reward += progress_reward
        reward += steps_reward
        reward += finish_reward

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

        def compute_tolerance(track_width, alpha=0.1):
            return alpha * track_width

        car_direction_relative_to_racing_line = position_relative_to_race_line(optimals[0:2], optimals_second[0:2],
                                                                               [x, y], heading)
        # TODO: The car turns to the right when it should be taking a left turn.
        if car_direction_relative_to_racing_line == Direction.LEFT and dist > compute_tolerance(track_width) and not (
                steering_angle < 2) \
                and get_track_direction(closest_index) != Direction.LEFT:
            # if direction_to_align_with_track == Direction.RIGHT and not (steering_angle < 2):
            print("Unforgivable action. Should turn right. Action is left. %f angle diff, %f steering angle",
                  racing_direction_diff, steering_angle)
            unforgivable_action = True

        if car_direction_relative_to_racing_line == Direction.RIGHT and dist > compute_tolerance(track_width) and not (
                steering_angle > -2) \
                and get_track_direction(closest_index) != Direction.RIGHT:
            # if direction_to_align_with_track == Direction.LEFT and not (steering_angle > -2):
            print("Unforgivable action. Should turn left. Action is right. %f angle diff, %f steering angle",
                  racing_direction_diff, steering_angle)
            unforgivable_action = True

        # TODO: The car’s speed is at least 1 m/s greater than its optimal speed while it is making a turn. Essentially the car is turning too fast.
        speed_diff = optimals[2] - speed
        if speed_diff < -0.5 and get_track_direction(closest_index) != Direction.STRAIGHT:
            print("Unforgivable action speed difference on turn %f < -0.5" % speed_diff)
            unforgivable_action = True
        # TODO: The speed of the car is 1.5 m/s slower than its optimal speed on a straight section. Essentially the car is going too slow on straight sections.

        if speed_diff > 1 and get_track_direction(closest_index) == Direction.STRAIGHT:
            print("Unforgivable action speed difference on straight %f > 1.0" % speed_diff)
            unforgivable_action = True

        if speed_diff > 1:
            print("Unforgivable action speed difference %f > 1" % speed_diff)
            unforgivable_action = True

        ## Zero reward if off track ##
        if not all_wheels_on_track:
            print("Unforgivable action all_wheels_on_track = %s" % all_wheels_on_track)
            unforgivable_action = True

        if GlobalParams.prev_direction_diff is not None and \
                abs(racing_direction_diff) > 30 and (
                abs(racing_direction_diff) > abs(GlobalParams.prev_direction_diff)):
            print("FAR AWAY FROM DIRECTION AND GETTING WORST: %f %f" % racing_direction_diff,
                  GlobalParams.prev_direction_diff)
            unforgivable_action = True

        if unforgivable_action:
            reward = MINIMAL_REWARD

        #################### UPDATE PREVIOUS STATE ####################
        PreviousState.update_params(params)

        ####################### VERBOSE #######################

        # TODO: Update logging
        print("Reward: %f" % reward)
        if self.verbose:
            print("Speed Reward: %f" % (speed_reward * SPEED_MULTIPLE))
            print("Distance Reward: %f" % (distance_reward * DISTANCE_MULTIPLE))
            print("Heading Reward: %f" % heading_reward)

            # print("Progress Reward: %f" % progress_reward)
            print("Steps Reward: %f" % steps_reward)
            print("Finish Reward: %f" % finish_reward)

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
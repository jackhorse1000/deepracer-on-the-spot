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
    MIN_SPEED = 2.2
    MAX_SPEED = 4.0
    STANDARD_TIME = 20.0
    FASTEST_TIME = 14.0
    racing_track = [[-0.32907, -3.68667, 4.0, 0.04281],
                    [-0.21511, -3.74923, 4.0, 0.0325],
                    [-0.10115, -3.8118, 4.0, 0.0325],
                    [0.04896, -3.89421, 4.0, 0.04281],
                    [0.31302, -4.03919, 4.0, 0.07531],
                    [0.57709, -4.18417, 4.0, 0.07531],
                    [0.84115, -4.32915, 4.0, 0.07531],
                    [1.10522, -4.47414, 4.0, 0.07531],
                    [1.36945, -4.61885, 4.0, 0.07532],
                    [1.6341, -4.76272, 4.0, 0.07531],
                    [1.89944, -4.9051, 4.0, 0.07528],
                    [2.16576, -5.04523, 4.0, 0.07523],
                    [2.43335, -5.18219, 4.0, 0.07515],
                    [2.70236, -5.31519, 4.0, 0.07502],
                    [2.97296, -5.44329, 4.0, 0.07485],
                    [3.24521, -5.56554, 4.0, 0.07461],
                    [3.51912, -5.68115, 4.0, 0.07433],
                    [3.79466, -5.7891, 4.0, 0.07398],
                    [4.07175, -5.88846, 4.0, 0.07359],
                    [4.35018, -5.97888, 3.88959, 0.07526],
                    [4.62975, -6.05983, 3.59466, 0.08097],
                    [4.91024, -6.1308, 3.12236, 0.09266],
                    [5.19139, -6.19093, 2.78469, 0.10325],
                    [5.4729, -6.239, 2.49196, 0.1146],
                    [5.75435, -6.27323, 2.4334, 0.11651],
                    [6.03507, -6.29092, 2.4334, 0.11559],
                    [6.31412, -6.28965, 2.4334, 0.11467],
                    [6.58962, -6.26388, 2.4334, 0.11371],
                    [6.8585, -6.20827, 2.4334, 0.11283],
                    [7.11543, -6.11706, 2.4334, 0.11204],
                    [7.35505, -5.99109, 2.64447, 0.10237],
                    [7.57658, -5.83817, 2.83461, 0.09497],
                    [7.78017, -5.6638, 3.02485, 0.08862],
                    [7.96643, -5.47228, 3.143, 0.085],
                    [8.13548, -5.26638, 3.19601, 0.08335],
                    [8.28689, -5.04808, 3.34384, 0.07945],
                    [8.42145, -4.81975, 3.36851, 0.07868],
                    [8.53883, -4.58279, 3.38639, 0.07809],
                    [8.63875, -4.33846, 3.40015, 0.07763],
                    [8.72131, -4.0881, 3.24391, 0.08127],
                    [8.786, -3.8327, 2.9948, 0.08797],
                    [8.83341, -3.57352, 2.75999, 0.09546],
                    [8.86356, -3.3116, 2.48296, 0.10618],
                    [8.87661, -3.04801, 2.2, 0.11996],
                    [8.8709, -2.78377, 2.2, 0.12013],
                    [8.8445, -2.52032, 2.2, 0.12035],
                    [8.79389, -2.25977, 2.2, 0.12065],
                    [8.71504, -2.00535, 2.2, 0.12107],
                    [8.60204, -1.76246, 2.2, 0.12177],
                    [8.44775, -1.541, 2.5071, 0.10766],
                    [8.26406, -1.3409, 2.78945, 0.09738],
                    [8.05808, -1.1606, 3.02571, 0.09047],
                    [7.83404, -0.99879, 3.27029, 0.08451],
                    [7.59517, -0.85408, 3.52638, 0.0792],
                    [7.344, -0.72511, 3.76428, 0.07501],
                    [7.08241, -0.61079, 3.9981, 0.0714],
                    [6.81195, -0.51017, 4.0, 0.07214],
                    [6.53392, -0.42241, 4.0, 0.07289],
                    [6.24941, -0.34689, 3.75336, 0.07843],
                    [5.95942, -0.28308, 3.38428, 0.08774],
                    [5.66475, -0.23091, 3.16824, 0.09445],
                    [5.36602, -0.19115, 3.16824, 0.09512],
                    [5.06407, -0.1649, 3.16824, 0.09566],
                    [4.76027, -0.15495, 3.16824, 0.09594],
                    [4.4887, -0.16333, 3.16824, 0.08576],
                    [4.22295, -0.1521, 3.16824, 0.08395],
                    [3.9606, -0.11928, 3.21579, 0.08222],
                    [3.69895, -0.06503, 3.47707, 0.07685],
                    [3.43718, 0.00849, 3.62318, 0.07504],
                    [3.17904, 0.09774, 3.62318, 0.07538],
                    [2.92213, 0.20583, 4.0, 0.06968],
                    [2.66157, 0.32605, 4.0, 0.07174],
                    [2.4065, 0.45238, 4.0, 0.07116],
                    [2.15348, 0.59255, 4.0, 0.07231],
                    [1.90266, 0.73364, 3.58764, 0.08021],
                    [1.66174, 0.88645, 3.58764, 0.07952],
                    [1.42299, 1.04388, 3.58764, 0.07971],
                    [1.19757, 1.20877, 3.58764, 0.07785],
                    [0.97867, 1.38593, 3.44976, 0.08163],
                    [0.76925, 1.56843, 3.18097, 0.08733],
                    [0.5764, 1.76045, 3.18097, 0.08555],
                    [0.3948, 1.96549, 3.18097, 0.0861],
                    [0.22558, 2.17998, 3.18097, 0.08589],
                    [0.07239, 2.40267, 3.18097, 0.08497],
                    [-0.06303, 2.63256, 3.18097, 0.08388],
                    [-0.17864, 2.8716, 3.29746, 0.08052],
                    [-0.27654, 3.12095, 4.0, 0.06697],
                    [-0.38257, 3.38265, 4.0, 0.07059],
                    [-0.4951, 3.64302, 4.0, 0.07091],
                    [-0.61694, 3.90062, 4.0, 0.07124],
                    [-0.75012, 4.15356, 3.98081, 0.07181],
                    [-0.89515, 4.4001, 3.96517, 0.07214],
                    [-1.05191, 4.63882, 3.8252, 0.07466],
                    [-1.22005, 4.86864, 3.64101, 0.07821],
                    [-1.39922, 5.08862, 3.64101, 0.07792],
                    [-1.58916, 5.29792, 3.64101, 0.07763],
                    [-1.78967, 5.49566, 3.64101, 0.07734],
                    [-2.00024, 5.68138, 3.64101, 0.07711],
                    [-2.22099, 5.85381, 3.64101, 0.07693],
                    [-2.45223, 6.01114, 3.68292, 0.07594],
                    [-2.69282, 6.15349, 3.68292, 0.0759],
                    [-2.94195, 6.28066, 3.68292, 0.07595],
                    [-3.19911, 6.39174, 3.69116, 0.07589],
                    [-3.46342, 6.48634, 3.71512, 0.07556],
                    [-3.73387, 6.5642, 3.75354, 0.07498],
                    [-4.00937, 6.62525, 3.81638, 0.07394],
                    [-4.2887, 6.66967, 3.82864, 0.07388],
                    [-4.57076, 6.69729, 3.84259, 0.07375],
                    [-4.85433, 6.70819, 3.84259, 0.07385],
                    [-5.13823, 6.70212, 3.84259, 0.0739],
                    [-5.42121, 6.67908, 3.78718, 0.07497],
                    [-5.70207, 6.63933, 3.42908, 0.08272],
                    [-5.97986, 6.58381, 3.12295, 0.09071],
                    [-6.25363, 6.51287, 2.7948, 0.10119],
                    [-6.52245, 6.42677, 2.7948, 0.101],
                    [-6.78522, 6.32538, 2.7948, 0.10078],
                    [-7.04072, 6.20842, 2.7948, 0.10054],
                    [-7.28611, 6.07321, 2.7948, 0.10025],
                    [-7.51756, 5.91731, 2.7948, 0.09985],
                    [-7.72881, 5.73805, 2.97693, 0.09307],
                    [-7.92013, 5.54085, 3.19768, 0.08592],
                    [-8.09308, 5.32999, 3.28602, 0.08299],
                    [-8.24796, 5.10771, 3.42633, 0.07907],
                    [-8.38581, 4.87615, 3.44528, 0.07822],
                    [-8.50649, 4.63647, 3.45445, 0.07768],
                    [-8.60982, 4.38972, 3.39413, 0.07882],
                    [-8.69601, 4.13696, 3.20204, 0.0834],
                    [-8.76484, 3.87906, 2.9857, 0.0894],
                    [-8.81643, 3.61698, 2.67057, 0.10002],
                    [-8.85083, 3.35166, 2.37302, 0.11274],
                    [-8.86813, 3.08402, 2.37302, 0.11302],
                    [-8.86596, 2.81486, 2.37302, 0.11343],
                    [-8.84166, 2.5454, 2.37302, 0.11401],
                    [-8.79158, 2.27751, 2.37302, 0.11485],
                    [-8.70899, 2.01463, 2.37302, 0.11612],
                    [-8.58546, 1.76396, 2.99229, 0.09339],
                    [-8.43684, 1.5245, 3.31493, 0.08502],
                    [-8.26817, 1.29586, 3.60298, 0.07886],
                    [-8.08283, 1.0776, 3.89925, 0.07343],
                    [-7.88359, 0.86914, 4.0, 0.07209],
                    [-7.67276, 0.66983, 4.0, 0.07253],
                    [-7.45233, 0.47891, 4.0, 0.0729],
                    [-7.22403, 0.29555, 4.0, 0.07321],
                    [-6.98912, 0.11902, 4.0, 0.07346],
                    [-6.74864, -0.05137, 4.0, 0.07368],
                    [-6.50342, -0.21636, 4.0, 0.07389],
                    [-6.25418, -0.3767, 4.0, 0.07409],
                    [-6.0015, -0.53316, 4.0, 0.0743],
                    [-5.74591, -0.68652, 4.0, 0.07452],
                    [-5.48787, -0.83746, 4.0, 0.07474],
                    [-5.22783, -0.98662, 4.0, 0.07494],
                    [-4.96619, -1.13444, 4.0, 0.07513],
                    [-4.70337, -1.2813, 4.0, 0.07527],
                    [-4.43984, -1.4275, 4.0, 0.07534],
                    [-4.17603, -1.57336, 4.0, 0.07536],
                    [-3.9121, -1.71895, 4.0, 0.07536],
                    [-3.64817, -1.86427, 4.0, 0.07532],
                    [-3.38407, -2.00942, 4.0, 0.07534],
                    [-3.11993, -2.1545, 4.0, 0.07534],
                    [-2.8558, -2.2995, 4.0, 0.07533],
                    [-2.59171, -2.44446, 4.0, 0.07531],
                    [-2.32765, -2.58941, 4.0, 0.07531],
                    [-2.06359, -2.73437, 4.0, 0.07531],
                    [-1.79953, -2.87935, 4.0, 0.07531],
                    [-1.53546, -3.02433, 4.0, 0.07531],
                    [-1.27139, -3.16932, 4.0, 0.07531],
                    [-1.00732, -3.3143, 4.0, 0.07531],
                    [-0.74325, -3.45928, 4.0, 0.07531],
                    [-0.47918, -3.60425, 4.0, 0.07531]]


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
        SPEED_MULTIPLE = 4
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
        REWARD_FOR_FASTEST_CHECKPOINT_TIME = 100
        standard_multiplier = float(TrackInfo.STANDARD_TIME)/float(TrackInfo.FASTEST_TIME)
        if pi != 0 and GlobalParams.progress_reward_list[pi] == 0 and steps != 0 and GlobalParams.first_racingpoint_index != closest_index:
            fastest_time = get_time_between_two_indicies(GlobalParams.first_racingpoint_index, closest_index)
            standard_time = standard_multiplier * fastest_time
            checkpoint_reward = max(MINIMAL_REWARD, (-REWARD_FOR_FASTEST_CHECKPOINT_TIME / (15 * (standard_time - fastest_time))) * (steps - standard_time * 15))
            GlobalParams.progress_reward_list[pi] = checkpoint_reward


            

        ################ FINAL REWARD ################

        # Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500  # should be adapted to track length and other rewards
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
        
        def compute_tolerance(track_width, alpha=0.1):
            return alpha * track_width

        car_direction_relative_to_racing_line = position_relative_to_race_line(optimals[0:2], optimals_second[0:2], [x, y], heading)
        # TODO: The car turns to the right when it should be taking a left turn.
        if car_direction_relative_to_racing_line == Direction.LEFT and dist > compute_tolerance(track_width) and not (steering_angle < 2) \
            and get_track_direction(closest_index) != Direction.LEFT:
            # if direction_to_align_with_track == Direction.RIGHT and not (steering_angle < 2):
                print("Unforgivable action. Should turn right. Action is left. %f angle diff, %f steering angle", racing_direction_diff, steering_angle)
                unforgivable_action = True

        if car_direction_relative_to_racing_line == Direction.RIGHT and dist > compute_tolerance(track_width) and not (steering_angle > -2) \
            and get_track_direction(closest_index) != Direction.RIGHT:
            # if direction_to_align_with_track == Direction.LEFT and not (steering_angle > -2):
                print("Unforgivable action. Should turn left. Action is right. %f angle diff, %f steering angle", racing_direction_diff, steering_angle)
                unforgivable_action = True

        # TODO: The car’s speed is at least 1 m/s greater than its optimal speed while it is making a turn. Essentially the car is turning too fast.
        speed_diff = optimals[2] - speed
        if speed_diff < -0.2 and get_track_direction(closest_index) != Direction.STRAIGHT:
            print("Unforgivable action speed difference on turn %f < -0.2" % speed_diff)
            unforgivable_action = True
        # TODO: The speed of the car is 1.5 m/s slower than its optimal speed on a straight section. Essentially the car is going too slow on straight sections.
        
        if speed_diff > 1 and get_track_direction(closest_index) == Direction.STRAIGHT:
            print("Unforgivable action speed difference on straight %f > 1.0" % speed_diff)
            unforgivable_action = True
            
        if speed_diff > 1.5:
            print("Unforgivable action speed difference %f > 1.5" % speed_diff)
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
        reward += speed_reward * SPEED_MULTIPLE
        reward += distance_reward * DISTANCE_MULTIPLE
        reward += heading_reward
        reward += steering_reward

        # reward += progress_reward
        reward += steps_reward
        
        if unforgivable_action:
            reward = MINIMAL_REWARD
        
        reward += finish_reward
        reward += checkpoint_reward

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

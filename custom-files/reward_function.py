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
    MIN_SPEED = 2
    MAX_SPEED = 4
    STANDARD_TIME = 20.0
    FASTEST_TIME = 14.5
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
                    [2.64384, -4.80107, 4.0, 0.07527],
                    [2.90802, -4.94464, 4.0, 0.07517],
                    [3.17239, -5.08644, 4.0, 0.075],
                    [3.437, -5.22566, 4.0, 0.07475],
                    [3.70194, -5.36146, 4.0, 0.07443],
                    [3.96726, -5.49295, 4.0, 0.07403],
                    [4.23299, -5.6192, 3.81111, 0.07719],
                    [4.49915, -5.73913, 3.46554, 0.08424],
                    [4.76574, -5.85148, 3.03411, 0.09535],
                    [5.03274, -5.95458, 2.67575, 0.10696],
                    [5.30002, -6.04659, 2.32472, 0.1216],
                    [5.56737, -6.12538, 2.0, 0.13936],
                    [5.83441, -6.18828, 2.0, 0.13718],
                    [6.10058, -6.23276, 2.0, 0.13493],
                    [6.36485, -6.25427, 2.0, 0.13257],
                    [6.62551, -6.24763, 2.0, 0.13037],
                    [6.87934, -6.20546, 2.0, 0.12866],
                    [7.11948, -6.11752, 2.20756, 0.11585],
                    [7.3441, -5.99372, 2.4019, 0.10678],
                    [7.55237, -5.84081, 2.60406, 0.09922],
                    [7.74403, -5.66425, 2.79754, 0.09315],
                    [7.91899, -5.46849, 3.01906, 0.08696],
                    [8.07785, -5.25755, 3.18477, 0.08292],
                    [8.22087, -5.03428, 3.32151, 0.07983],
                    [8.34828, -4.801, 3.42995, 0.0775],
                    [8.46024, -4.55961, 3.53118, 0.07535],
                    [8.55704, -4.31177, 3.33879, 0.07969],
                    [8.63903, -4.05892, 3.10259, 0.08567],
                    [8.70662, -3.8023, 2.83978, 0.09345],
                    [8.7598, -3.54294, 2.56736, 0.10312],
                    [8.79815, -3.2818, 2.28941, 0.11529],
                    [8.82097, -3.01987, 2.00034, 0.13144],
                    [8.82584, -2.75835, 2.00034, 0.13076],
                    [8.81026, -2.49895, 2.00034, 0.12991],
                    [8.7709, -2.24407, 2.00034, 0.12893],
                    [8.70355, -1.99724, 2.00034, 0.12791],
                    [8.60276, -1.76388, 2.00034, 0.12707],
                    [8.46122, -1.55346, 2.00623, 0.12641],
                    [8.28264, -1.37139, 2.21936, 0.11491],
                    [8.07617, -1.21647, 2.48182, 0.104],
                    [7.84868, -1.08591, 2.77146, 0.09464],
                    [7.60499, -0.97679, 3.1076, 0.08592],
                    [7.3488, -0.88616, 3.50344, 0.07757],
                    [7.08298, -0.8111, 4.0, 0.06905],
                    [6.81009, -0.74831, 4.0, 0.07001],
                    [6.53203, -0.69496, 4.0, 0.07078],
                    [6.25056, -0.64803, 4.0, 0.07134],
                    [5.96743, -0.60431, 4.0, 0.07162],
                    [5.67992, -0.55805, 4.0, 0.0728],
                    [5.39297, -0.50931, 4.0, 0.07277],
                    [5.10686, -0.45733, 4.0, 0.0727],
                    [4.82189, -0.40141, 4.0, 0.0726],
                    [4.53842, -0.34072, 4.0, 0.07247],
                    [4.25688, -0.27445, 4.0, 0.07231],
                    [3.97774, -0.20173, 4.0, 0.07211],
                    [3.70151, -0.12171, 4.0, 0.07189],
                    [3.42873, -0.03368, 4.0, 0.07166],
                    [3.1599, 0.06296, 4.0, 0.07142],
                    [2.89556, 0.16874, 4.0, 0.07118],
                    [2.63624, 0.28406, 4.0, 0.07095],
                    [2.38245, 0.40922, 4.0, 0.07074],
                    [2.13471, 0.54439, 4.0, 0.07056],
                    [1.89342, 0.68957, 4.0, 0.0704],
                    [1.65879, 0.84448, 3.84072, 0.0732],
                    [1.4305, 1.00811, 3.84072, 0.07313],
                    [1.2085, 1.18003, 3.84072, 0.0731],
                    [0.99331, 1.36043, 3.84072, 0.07311],
                    [0.78581, 1.54992, 3.84072, 0.07316],
                    [0.58728, 1.74934, 3.84072, 0.07327],
                    [0.39968, 1.95992, 4.0, 0.07051],
                    [0.22257, 2.18033, 4.0, 0.07069],
                    [0.05403, 2.40817, 4.0, 0.07085],
                    [-0.10721, 2.64192, 4.0, 0.07099],
                    [-0.26228, 2.88028, 4.0, 0.07109],
                    [-0.4122, 3.12217, 4.0, 0.07114],
                    [-0.55783, 3.3667, 4.0, 0.07115],
                    [-0.69993, 3.61319, 4.0, 0.07113],
                    [-0.83921, 3.86107, 3.70161, 0.07681],
                    [-0.97619, 4.10996, 3.163, 0.08982],
                    [-1.11112, 4.35963, 2.69452, 0.10532],
                    [-1.24065, 4.60314, 2.69452, 0.10236],
                    [-1.37413, 4.84298, 2.69452, 0.10187],
                    [-1.5154, 5.07552, 2.69452, 0.10098],
                    [-1.66807, 5.29713, 2.69452, 0.09987],
                    [-1.83561, 5.50392, 2.69452, 0.09877],
                    [-2.02238, 5.69014, 2.77479, 0.09505],
                    [-2.22542, 5.8574, 2.77479, 0.0948],
                    [-2.44467, 6.0041, 2.83731, 0.09297],
                    [-2.67871, 6.1302, 3.07989, 0.08632],
                    [-2.92482, 6.23788, 3.30799, 0.08121],
                    [-3.18103, 6.32872, 3.57445, 0.07605],
                    [-3.44552, 6.40437, 3.71741, 0.074],
                    [-3.7172, 6.46526, 3.85471, 0.07223],
                    [-3.99493, 6.51172, 3.99632, 0.07046],
                    [-4.27737, 6.54416, 4.0, 0.07108],
                    [-4.56305, 6.56307, 4.0, 0.07158],
                    [-4.85046, 6.56892, 4.0, 0.07187],
                    [-5.13815, 6.5623, 3.72971, 0.07716],
                    [-5.42488, 6.54356, 3.42387, 0.08392],
                    [-5.7096, 6.51317, 3.09018, 0.09266],
                    [-5.99133, 6.47117, 2.81877, 0.10105],
                    [-6.26894, 6.41684, 2.42625, 0.11659],
                    [-6.54104, 6.34901, 2.42625, 0.11558],
                    [-6.8061, 6.26628, 2.42625, 0.11444],
                    [-7.06217, 6.16669, 2.42625, 0.11325],
                    [-7.30664, 6.04737, 2.42625, 0.11212],
                    [-7.53645, 5.90549, 2.42625, 0.11132],
                    [-7.74484, 5.73485, 2.69641, 0.09989],
                    [-7.93401, 5.54246, 2.90516, 0.09287],
                    [-8.10504, 5.33269, 3.06654, 0.08826],
                    [-8.25835, 5.10876, 3.25491, 0.08338],
                    [-8.39481, 4.87367, 3.3548, 0.08103],
                    [-8.51452, 4.62948, 3.33665, 0.0815],
                    [-8.61765, 4.37804, 3.14185, 0.0865],
                    [-8.70441, 4.12096, 2.91967, 0.09293],
                    [-8.77495, 3.85966, 2.62979, 0.10292],
                    [-8.82945, 3.5954, 2.35033, 0.1148],
                    [-8.86683, 3.32927, 2.09276, 0.12842],
                    [-8.88543, 3.06252, 2.09276, 0.12777],
                    [-8.88311, 2.79674, 2.09276, 0.127],
                    [-8.85701, 2.534, 2.09276, 0.12616],
                    [-8.80246, 2.27738, 2.09276, 0.12536],
                    [-8.71347, 2.03147, 2.09276, 0.12496],
                    [-8.58257, 1.80347, 2.61211, 0.10065],
                    [-8.42522, 1.58932, 2.89533, 0.09178],
                    [-8.24614, 1.38775, 3.1568, 0.08541],
                    [-8.04842, 1.19785, 3.48227, 0.07873],
                    [-7.835, 1.01845, 3.81514, 0.07308],
                    [-7.6082, 0.84854, 4.0, 0.07085],
                    [-7.37036, 0.68686, 4.0, 0.0719],
                    [-7.12355, 0.53217, 4.0, 0.07282],
                    [-6.86985, 0.383, 4.0, 0.07358],
                    [-6.61115, 0.23785, 4.0, 0.07416],
                    [-6.34901, 0.09535, 4.0, 0.07459],
                    [-6.08487, -0.04573, 4.0, 0.07486],
                    [-5.81899, -0.18713, 4.0, 0.07528],
                    [-5.55325, -0.32879, 4.0, 0.07529],
                    [-5.28763, -0.47071, 4.0, 0.07529],
                    [-5.02215, -0.61288, 4.0, 0.07529],
                    [-4.75678, -0.7553, 4.0, 0.07529],
                    [-4.49154, -0.89795, 4.0, 0.07529],
                    [-4.22641, -1.04083, 4.0, 0.07529],
                    [-3.9614, -1.18395, 4.0, 0.0753],
                    [-3.69651, -1.32729, 4.0, 0.0753],
                    [-3.43173, -1.47087, 4.0, 0.0753],
                    [-3.16707, -1.61468, 4.0, 0.0753],
                    [-2.90252, -1.75871, 4.0, 0.0753],
                    [-2.63809, -1.90298, 4.0, 0.07531],
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
        if speed_diff < -0.5 and get_track_direction(closest_index) != Direction.STRAIGHT:
            print("Unforgivable action speed difference on turn %f < -0.5" % speed_diff)
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

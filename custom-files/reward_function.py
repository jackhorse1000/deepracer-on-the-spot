import math
from enum import Enum

class PARAMS:
    prev_speed = None
    prev_optimum_speed = None
    prev_steering_angle = None
    prev_steps = None
    prev_direction_diff = None
    prev_normalized_distance_from_route = None

class Direction(Enum):
    RIGHT = 1
    LEFT = 2
    STRAIGHT = 3

class TrackInfo:
    MIN_SPEED = 2.2
    MAX_SPEED = 4
    STANDARD_TIME = 18
    FASTEST_TIME = 15

class Reward:
    def __init__(self, verbose=True):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        import math

        def should_break(optimum_speed):
            return optimum_speed < PARAMS.prev_optimum_speed

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

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):
            if start is not None and end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count - 1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time / current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
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
                        [3.437, -5.22566, 3.81111, 0.07846],
                        [3.70194, -5.36146, 3.46554, 0.08591],
                        [3.96726, -5.49295, 3.03411, 0.09759],
                        [4.23299, -5.6192, 2.67575, 0.10995],
                        [4.49915, -5.73913, 2.32472, 0.12558],
                        [4.76574, -5.85148, 2.0, 0.14465],
                        [5.03274, -5.95458, 2.0, 0.1431],
                        [5.30002, -6.04659, 2.0, 0.14134],
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
                        [8.34828, -4.801, 3.10259, 0.08567],
                        [8.46024, -4.55961, 2.83978, 0.0937],
                        [8.55704, -4.31177, 2.56736, 0.10364],
                        [8.63903, -4.05892, 2.28941, 0.11611],
                        [8.70662, -3.8023, 2.00034, 0.13266],
                        [8.7598, -3.54294, 2.00034, 0.13236],
                        [8.79815, -3.2818, 2.00034, 0.13195],
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
                        [2.38245, 0.40922, 3.84072, 0.07368],
                        [2.13471, 0.54439, 3.84072, 0.07348],
                        [1.89342, 0.68957, 3.84072, 0.07332],
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
                        [-0.4122, 3.12217, 3.70161, 0.07688],
                        [-0.55783, 3.3667, 3.163, 0.08998],
                        [-0.69993, 3.61319, 2.69452, 0.10559],
                        [-0.83921, 3.86107, 2.69452, 0.10552],
                        [-0.97619, 4.10996, 2.69452, 0.10543],
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
                        [-4.27737, 6.54416, 3.72971, 0.07623],
                        [-4.56305, 6.56307, 3.42387, 0.08362],
                        [-4.85046, 6.56892, 3.09018, 0.09303],
                        [-5.13815, 6.5623, 2.81877, 0.10209],
                        [-5.42488, 6.54356, 2.42625, 0.11843],
                        [-5.7096, 6.51317, 2.42625, 0.11801],
                        [-5.99133, 6.47117, 2.42625, 0.1174],
                        [-6.26894, 6.41684, 2.42625, 0.11659],
                        [-6.54104, 6.34901, 2.42625, 0.11558],
                        [-6.8061, 6.26628, 2.42625, 0.11444],
                        [-7.06217, 6.16669, 2.42625, 0.11325],
                        [-7.30664, 6.04737, 2.42625, 0.11212],
                        [-7.53645, 5.90549, 2.42625, 0.11132],
                        [-7.74484, 5.73485, 2.69641, 0.09989],
                        [-7.93401, 5.54246, 2.90516, 0.09287],
                        [-8.10504, 5.33269, 3.06654, 0.08826],
                        [-8.25835, 5.10876, 3.14185, 0.08638],
                        [-8.39481, 4.87367, 2.91967, 0.0931],
                        [-8.51452, 4.62948, 2.62979, 0.10341],
                        [-8.61765, 4.37804, 2.35033, 0.11563],
                        [-8.70441, 4.12096, 2.09276, 0.12965],
                        [-8.77495, 3.85966, 2.09276, 0.12933],
                        [-8.82945, 3.5954, 2.09276, 0.12893],
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

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Reinitialize previous parameters if it is a new episode
        if PARAMS.prev_steps is None or steps < PARAMS.prev_steps:
            PARAMS.prev_speed = None
            PARAMS.prev_steering_angle = None
            PARAMS.prev_direction_diff = None

        has_speed_dropped = False
        if PARAMS.prev_speed is not None:
            if PARAMS.prev_speed > speed:
                has_speed_dropped = True

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        # if self.verbose == True:
        #     self.first_racingpoint_index = 0 # this is just for testing purposes
        if self.first_racingpoint_index is None:
            print("At step %f Setting first_racingpoint_index" % steps)
            self.first_racingpoint_index = closest_index
        print("first_racingpoint_index is set to: ", self.first_racingpoint_index)

        def angle_between_vectors(u, v):
            dot_product = u[0] * v[0] + u[1] * v[1]
            determinant = u[0] * v[1] - u[1] * v[0]
            return math.degrees(math.atan2(determinant, dot_product))

        def track_lookahed_degree_turns(closest_index, lookahead=5):
            coords = []
            for i in range(0, lookahead):
                current_index = (closest_index + i) % len(racing_track)
                current_point = racing_track[current_index]
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

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist / (track_width * 0.5)))
        # reward += distance_reward * DISTANCE_MULTIPLE

        distance_reduction_bonus = 1
        if PARAMS.prev_normalized_distance_from_route is not None and PARAMS.prev_normalized_distance_from_route > dist:
            if abs(dist) > 0:
                distance_reduction_bonus = min(
                    abs(PARAMS.prev_normalized_distance_from_route / dist), 2)

        distance_reward = distance_reward * distance_reduction_bonus

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 0.5
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2] - speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff / (SPEED_DIFF_NO_REWARD)) ** 2) ** 2
        else:
            speed_reward = 0
        # reward += speed_reward * SPEED_MULTIPLE

        # Check if the speed has dropped
        has_speed_dropped = False
        is_turn_upcoming = get_track_direction(closest_index) != Direction.STRAIGHT
        if PARAMS.prev_speed is not None:
            if PARAMS.prev_speed > speed:
                has_speed_dropped = True
        # Penalize slowing down without good reason on straight portions
        if has_speed_dropped and not is_turn_upcoming:
            speed_maintain_bonus = min(speed / PARAMS.prev_speed, 1)

        speed_reward = speed_reward * speed_maintain_bonus

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 10
        STANDARD_TIME = 20
        FASTEST_TIME = 14
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 14 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME * (FASTEST_TIME) /
                                           (STANDARD_TIME - FASTEST_TIME)) * (
                                                steps_prediction - (STANDARD_TIME * 14 + 1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        # reward += steps_reward

        progress_reward = (progress ** 2) / steps
        if steps <= 5:
            progress_reward = 1  # ignore progress in the first 5 steps

        ################ DIRECTION DIFF ################

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)

        direction_diff_reward = 10 * abs(math.cos(direction_diff * math.pi / 20))

        has_steering_angle_changed = False
        if PARAMS.prev_steering_angle is not None:
            if not (math.isclose(PARAMS.prev_steering_angle, steering_angle)):
                has_steering_angle_changed = True
        steering_angle_maintain_bonus = 1
        # Not changing the steering angle is a good thing if heading in the right direction
        if dist < 20 and not has_steering_angle_changed:
            if abs(direction_diff) < 10:
                steering_angle_maintain_bonus *= 2
            if abs(direction_diff) < 5:
                steering_angle_maintain_bonus *= 2
            if PARAMS.prev_direction_diff is not None and abs(PARAMS.prev_direction_diff) > abs(direction_diff):
                steering_angle_maintain_bonus *= 2

        direction_diff_reward = direction_diff_reward * steering_angle_maintain_bonus

        reward = 10 * ((distance_reward + speed_reward) ** 2) + progress_reward + direction_diff_reward

        ################################

        if direction_diff > 20:
            reward = 1e-3

        if speed_diff > SPEED_DIFF_NO_REWARD:
            reward = 1e-3

        if dist > 0.2:
            reward = 1e-3

        # Zero reward of obviously too slow
        # speed_diff_zero = optimals[2]-speed
        # if speed_diff_zero > 0.5:
        #     reward = 1e-3

        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 3000  # should be adapted to track length and other rewards
        STANDARD_TIME = 18  # seconds (time that is easily done by model)
        FASTEST_TIME = 15  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                                       (15 * (STANDARD_TIME - FASTEST_TIME))) * (
                                            steps - STANDARD_TIME * 15)) + progress_reward
        else:
            finish_reward = 1e-3
        reward += finish_reward

        if progress == 75 or progress == 50 or progress == 25 or progress == 10:
            reward += (progress ** 3) / steps
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = finish_reward

        # Before returning reward, update the variables
        PARAMS.prev_speed = speed
        PARAMS.prev_steering_angle = steering_angle
        PARAMS.prev_direction_diff = direction_diff
        PARAMS.prev_steps = steps
        PARAMS.prev_normalized_distance_from_route = dist

        ####################### VERBOSE #######################
        if self.verbose == True:
            # print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f, reward =  %f" % (dist, distance_reward))
            # print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f, reward = %f" % (speed_diff, speed_reward))
            print("Direction difference: %f, reward = %f" % (direction_diff, direction_diff_reward))
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward =  %f, progress reward =  %f" % (steps_reward, progress_reward))
            print("=== Finish reward: %f ===" % finish_reward)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(max(1e-3, reward))


reward_object = Reward()  # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)


def get_test_params():
    return {
        'x': 0.7,
        'y': 1.05,
        'heading': 160.0,
        'track_width': 0.45,
        'is_reversed': False,
        'steering_angle': 0.0,
        'all_wheels_on_track': True,
        'progress': 100.0,
        'steps': 1,
        'distance_from_center': 0.0,
        'closest_waypoints': [0, 1, 2],
        'is_left_of_center': False,
        'speed': 1.0,
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
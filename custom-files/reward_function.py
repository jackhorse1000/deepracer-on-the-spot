import math

class Reward:
    def __init__(self, verbose=True):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        import math

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

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
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

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
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
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
                        [3.437, -5.22566, 4.0, 0.07475],
                        [3.70194, -5.36146, 4.0, 0.07443],
                        [3.96726, -5.49295, 4.0, 0.07403],
                        [4.23299, -5.6192, 4.0, 0.07355],
                        [4.49915, -5.73913, 4.0, 0.07298],
                        [4.76574, -5.85148, 3.48708, 0.08296],
                        [5.03274, -5.95458, 3.0, 0.0954],
                        [5.30002, -6.04659, 3.0, 0.09422],
                        [5.56737, -6.12538, 3.0, 0.09291],
                        [5.83441, -6.18828, 3.0, 0.09145],
                        [6.10058, -6.23276, 3.0, 0.08995],
                        [6.36485, -6.25427, 3.0, 0.08838],
                        [6.62551, -6.24763, 3.0, 0.08691],
                        [6.87934, -6.20546, 3.0, 0.08577],
                        [7.11948, -6.11752, 3.31134, 0.07723],
                        [7.3441, -5.99372, 3.60285, 0.07119],
                        [7.55237, -5.84081, 3.90609, 0.06615],
                        [7.74403, -5.66425, 4.0, 0.06515],
                        [7.91899, -5.46849, 4.0, 0.06564],
                        [8.07785, -5.25755, 4.0, 0.06602],
                        [8.22087, -5.03428, 4.0, 0.06629],
                        [8.34828, -4.801, 4.0, 0.06645],
                        [8.46024, -4.55961, 4.0, 0.06652],
                        [8.55704, -4.31177, 4.0, 0.06652],
                        [8.63903, -4.05892, 3.85104, 0.06902],
                        [8.70662, -3.8023, 3.43411, 0.07728],
                        [8.7598, -3.54294, 3.00051, 0.08824],
                        [8.79815, -3.2818, 3.00051, 0.08797],
                        [8.82097, -3.01987, 3.00051, 0.08763],
                        [8.82584, -2.75835, 3.00051, 0.08717],
                        [8.81026, -2.49895, 3.00051, 0.08661],
                        [8.7709, -2.24407, 3.00051, 0.08595],
                        [8.70355, -1.99724, 3.00051, 0.08527],
                        [8.60276, -1.76388, 3.00051, 0.08471],
                        [8.46122, -1.55346, 3.00934, 0.08427],
                        [8.28264, -1.37139, 3.32904, 0.07661],
                        [8.07617, -1.21647, 3.72274, 0.06934],
                        [7.84868, -1.08591, 4.0, 0.06557],
                        [7.60499, -0.97679, 4.0, 0.06675],
                        [7.3488, -0.88616, 4.0, 0.06794],
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
                        [1.65879, 0.84448, 4.0, 0.07029],
                        [1.4305, 1.00811, 4.0, 0.07022],
                        [1.2085, 1.18003, 4.0, 0.07019],
                        [0.99331, 1.36043, 4.0, 0.0702],
                        [0.78581, 1.54992, 4.0, 0.07025],
                        [0.58728, 1.74934, 4.0, 0.07035],
                        [0.39968, 1.95992, 4.0, 0.07051],
                        [0.22257, 2.18033, 4.0, 0.07069],
                        [0.05403, 2.40817, 4.0, 0.07085],
                        [-0.10721, 2.64192, 4.0, 0.07099],
                        [-0.26228, 2.88028, 4.0, 0.07109],
                        [-0.4122, 3.12217, 4.0, 0.07114],
                        [-0.55783, 3.3667, 4.0, 0.07115],
                        [-0.69993, 3.61319, 4.0, 0.07113],
                        [-0.83921, 3.86107, 4.0, 0.07108],
                        [-0.97619, 4.10996, 4.0, 0.07102],
                        [-1.11112, 4.35963, 4.0, 0.07095],
                        [-1.24065, 4.60314, 4.0, 0.06895],
                        [-1.37413, 4.84298, 4.0, 0.06862],
                        [-1.5154, 5.07552, 4.0, 0.06802],
                        [-1.66807, 5.29713, 4.0, 0.06728],
                        [-1.83561, 5.50392, 4.0, 0.06654],
                        [-2.02238, 5.69014, 4.0, 0.06594],
                        [-2.22542, 5.8574, 4.0, 0.06577],
                        [-2.44467, 6.0041, 4.0, 0.06595],
                        [-2.67871, 6.1302, 4.0, 0.06646],
                        [-2.92482, 6.23788, 4.0, 0.06716],
                        [-3.18103, 6.32872, 4.0, 0.06796],
                        [-3.44552, 6.40437, 4.0, 0.06877],
                        [-3.7172, 6.46526, 4.0, 0.06961],
                        [-3.99493, 6.51172, 4.0, 0.0704],
                        [-4.27737, 6.54416, 4.0, 0.07108],
                        [-4.56305, 6.56307, 4.0, 0.07158],
                        [-4.85046, 6.56892, 4.0, 0.07187],
                        [-5.13815, 6.5623, 4.0, 0.07194],
                        [-5.42488, 6.54356, 4.0, 0.07184],
                        [-5.7096, 6.51317, 3.63937, 0.07868],
                        [-5.99133, 6.47117, 3.63937, 0.07827],
                        [-6.26894, 6.41684, 3.63937, 0.07773],
                        [-6.54104, 6.34901, 3.63937, 0.07706],
                        [-6.8061, 6.26628, 3.63937, 0.0763],
                        [-7.06217, 6.16669, 3.63937, 0.0755],
                        [-7.30664, 6.04737, 3.63937, 0.07475],
                        [-7.53645, 5.90549, 3.63937, 0.07421],
                        [-7.74484, 5.73485, 4.0, 0.06734],
                        [-7.93401, 5.54246, 4.0, 0.06745],
                        [-8.10504, 5.33269, 4.0, 0.06766],
                        [-8.25835, 5.10876, 4.0, 0.06785],
                        [-8.39481, 4.87367, 4.0, 0.06796],
                        [-8.51452, 4.62948, 4.0, 0.06799],
                        [-8.61765, 4.37804, 3.94469, 0.0689],
                        [-8.70441, 4.12096, 3.52549, 0.07696],
                        [-8.77495, 3.85966, 3.13914, 0.08622],
                        [-8.82945, 3.5954, 3.13914, 0.08595],
                        [-8.86683, 3.32927, 3.13914, 0.08561],
                        [-8.88543, 3.06252, 3.13914, 0.08518],
                        [-8.88311, 2.79674, 3.13914, 0.08467],
                        [-8.85701, 2.534, 3.13914, 0.08411],
                        [-8.80246, 2.27738, 3.13914, 0.08358],
                        [-8.71347, 2.03147, 3.13914, 0.08331],
                        [-8.58257, 1.80347, 3.91816, 0.0671],
                        [-8.42522, 1.58932, 4.0, 0.06644],
                        [-8.24614, 1.38775, 4.0, 0.06741],
                        [-8.04842, 1.19785, 4.0, 0.06854],
                        [-7.835, 1.01845, 4.0, 0.0697],
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

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index
            print("first_racingpoint_index is set to: ", self.first_racingpoint_index)

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1 
        STANDARD_TIME = 20
        FASTEST_TIME = 16
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3
            
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        STANDARD_TIME = 20  # seconds (time that is easily done by model)
        FASTEST_TIME = 16  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################
        
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        
        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


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
        'steps': 0,
        'distance_from_center': 0.0,
        'closest_waypoints': [0, 1, 2],
        'is_left_of_center': False,
        'speed': 1.0,
        'is_offtrack' : False,
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
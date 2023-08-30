import math

class PARAMS:
    prev_speed = None
    prev_optimum_speed = None
    prev_steering_angle = None 
    prev_steps = None
    prev_direction_diff = None
    prev_normalized_distance_from_route = None

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
        racing_track = [[-0.26489, -3.56978, 4.0, 0.04281],
[-0.15094, -3.63234, 4.0, 0.0325],
[-0.03698, -3.6949, 4.0, 0.0325],
[0.11313, -3.77732, 4.0, 0.04281],
[0.3772, -3.92229, 4.0, 0.07531],
[0.64127, -4.06727, 4.0, 0.07531],
[0.90533, -4.21226, 4.0, 0.07531],
[1.1694, -4.35725, 4.0, 0.07531],
[1.43348, -4.50223, 4.0, 0.07532],
[1.69758, -4.6472, 4.0, 0.07532],
[1.96168, -4.79215, 4.0, 0.07532],
[2.22574, -4.93706, 4.0, 0.0753],
[2.48991, -5.08165, 4.0, 0.07529],
[2.75488, -5.22485, 4.0, 0.0753],
[3.02114, -5.36533, 3.91442, 0.07691],
[3.28912, -5.50154, 3.72491, 0.0807],
[3.55905, -5.63208, 3.51136, 0.08539],
[3.83108, -5.75543, 3.32592, 0.0898],
[4.10522, -5.86996, 3.1806, 0.09341],
[4.38138, -5.97401, 3.07367, 0.09601],
[4.65926, -6.06586, 3.00144, 0.09751],
[4.9384, -6.14376, 2.96111, 0.09787],
[5.21817, -6.20595, 2.94968, 0.09716],
[5.49773, -6.25083, 2.94968, 0.09599],
[5.77614, -6.27712, 2.94968, 0.0948],
[6.05222, -6.28318, 2.94968, 0.09362],
[6.32461, -6.26746, 2.94968, 0.0925],
[6.59181, -6.22882, 2.94968, 0.09153],
[6.85221, -6.16652, 2.94968, 0.09077],
[7.10415, -6.08026, 2.94968, 0.09028],
[7.34597, -5.9702, 2.94968, 0.09007],
[7.57603, -5.83693, 2.96326, 0.08972],
[7.7928, -5.68144, 2.99679, 0.08902],
[7.9948, -5.50507, 3.04414, 0.08809],
[8.1807, -5.30947, 3.0883, 0.08738],
[8.34926, -5.09654, 3.023, 0.08983],
[8.49931, -4.86836, 2.97498, 0.0918],
[8.6298, -4.62715, 2.97498, 0.09218],
[8.73972, -4.37522, 2.97498, 0.09239],
[8.82813, -4.11496, 2.95698, 0.09295],
[8.89413, -3.8488, 2.76618, 0.09913],
[8.93691, -3.57923, 2.5, 0.10918],
[8.95573, -3.30871, 2.5, 0.10847],
[8.94999, -3.03976, 2.5, 0.10761],
[8.91931, -2.77481, 2.5, 0.10669],
[8.86378, -2.51614, 2.5, 0.10583],
[8.78463, -2.26544, 2.5, 0.10516],
[8.683, -2.02411, 2.5, 0.10474],
[8.55888, -1.79392, 2.5, 0.10461],
[8.40999, -1.57825, 2.5, 0.10483],
[8.23225, -1.38324, 2.74907, 0.09598],
[8.03194, -1.20784, 2.97855, 0.08939],
[7.81323, -1.05088, 3.23475, 0.08322],
[7.57945, -0.91091, 3.39316, 0.0803],
[7.3324, -0.78754, 3.60914, 0.07651],
[7.07401, -0.6799, 3.79315, 0.0738],
[6.80574, -0.58743, 4.0, 0.07094],
[6.5294, -0.50878, 3.43773, 0.08358],
[6.24638, -0.44294, 3.26128, 0.0891],
[5.95801, -0.38877, 3.26128, 0.08997],
[5.66521, -0.34586, 3.26128, 0.09074],
[5.36875, -0.31423, 3.26128, 0.09142],
[5.06938, -0.2944, 3.26128, 0.092],
[4.76813, -0.27807, 3.26128, 0.09251],
[4.4879, -0.25668, 3.26128, 0.08618],
[4.21184, -0.22999, 3.26128, 0.08504],
[3.97774, -0.19073, 3.26128, 0.07278],
[3.70151, -0.12171, 3.68833, 0.07719],
[3.42873, -0.03368, 4.0, 0.07166],
[3.1599, 0.06296, 4.0, 0.07142],
[2.89556, 0.16874, 4.0, 0.07118],
[2.63624, 0.28406, 4.0, 0.07095],
[2.38245, 0.40922, 4.0, 0.07074],
[2.13471, 0.54439, 4.0, 0.07056],
[1.89342, 0.68957, 4.0, 0.0704],
[1.65879, 0.84448, 4.0, 0.07029],
[1.4305, 1.00811, 3.38458, 0.08299],
[1.2085, 1.18003, 3.38458, 0.08296],
[0.99331, 1.36043, 3.38458, 0.08297],
[0.78581, 1.54992, 3.38458, 0.08302],
[0.58728, 1.74934, 3.38458, 0.08314],
[0.39968, 1.95992, 3.38458, 0.08333],
[0.22257, 2.18033, 3.38458, 0.08354],
[0.05403, 2.40817, 3.38458, 0.08373],
[-0.10021, 2.64192, 3.38458, 0.08274],
[-0.23228, 2.88028, 3.92583, 0.06941],
[-0.3502, 3.12217, 3.7656, 0.07146],
[-0.45571, 3.34282, 3.65181, 0.06698],
[-0.58145, 3.61565, 3.57218, 0.0841],
[-0.68699, 3.88472, 3.52093, 0.08209],
[-0.80267, 4.14902, 3.49763, 0.08249],
[-0.92908, 4.40745, 3.49763, 0.08225],
[-1.06713, 4.65863, 3.49763, 0.08194],
[-1.2177, 4.90098, 3.49763, 0.08157],
[-1.38152, 5.13284, 3.49763, 0.08117],
[-1.55895, 5.35266, 3.49763, 0.08077],
[-1.75003, 5.55906, 3.49763, 0.08042],
[-1.95453, 5.75087, 3.49763, 0.08016],
[-2.17202, 5.92709, 3.49763, 0.08003],
[-2.40188, 6.08694, 3.50316, 0.07992],
[-2.64333, 6.22984, 3.53718, 0.07932],
[-2.89538, 6.35547, 3.59726, 0.07829],
[-3.15689, 6.46373, 3.67862, 0.07694],
[-3.42664, 6.55469, 3.77429, 0.07542],
[-3.70328, 6.62859, 3.87535, 0.07389],
[-3.98547, 6.68571, 3.81553, 0.07546],
[-4.27186, 6.72639, 3.71529, 0.07786],
[-4.56108, 6.7509, 3.62053, 0.08017],
[-4.85183, 6.75943, 3.53607, 0.08226],
[-5.14277, 6.75206, 3.46486, 0.084],
[-5.43262, 6.72875, 3.40804, 0.08532],
[-5.72004, 6.68932, 3.36511, 0.08621],
[-6.0037, 6.63352, 3.33422, 0.08671],
[-6.28222, 6.56105, 3.31241, 0.08688],
[-6.55418, 6.4716, 3.29602, 0.08686],
[-6.81815, 6.36493, 3.28121, 0.08677],
[-7.07266, 6.2409, 3.26454, 0.08673],
[-7.31624, 6.09953, 3.2436, 0.08683],
[-7.54745, 5.94107, 3.21751, 0.08712],
[-7.76486, 5.76595, 3.18723, 0.08759],
[-7.96712, 5.57489, 3.15608, 0.08816],
[-8.15293, 5.36881, 3.13578, 0.08849],
[-8.32107, 5.1489, 3.13578, 0.08828],
[-8.4704, 4.91654, 3.13578, 0.08808],
[-8.59985, 4.67328, 3.13578, 0.08787],
[-8.70845, 4.42085, 3.0966, 0.08874],
[-8.79531, 4.16108, 2.88275, 0.09502],
[-8.85966, 3.89589, 2.88275, 0.09466],
[-8.90084, 3.62723, 2.88275, 0.09429],
[-8.91838, 3.35706, 2.88275, 0.09392],
[-8.91209, 3.08727, 2.88275, 0.09361],
[-8.88269, 2.81955, 2.88275, 0.09343],
[-8.83189, 2.5551, 2.88275, 0.09341],
[-8.7587, 2.29544, 2.88275, 0.09358],
[-8.66196, 2.04236, 2.88275, 0.09399],
[-8.53832, 1.79897, 3.16558, 0.08624],
[-8.39259, 1.56536, 3.45538, 0.07968],
[-8.22842, 1.34114, 3.67867, 0.07554],
[-8.04802, 1.12614, 3.82171, 0.07344],
[-7.85261, 0.92057, 4.0, 0.07091],
[-7.64435, 0.72387, 4.0, 0.07162],
[-7.42475, 0.53578, 4.0, 0.07229],
[-7.19504, 0.35624, 4.0, 0.07289],
[-6.95674, 0.18507, 4.0, 0.07335],
[-6.71146, 0.02196, 4.0, 0.07364],
[-6.46124, -0.13419, 4.0, 0.07374],
[-6.20801, -0.28522, 4.0, 0.07371],
[-5.95298, -0.4326, 4.0, 0.07364],
[-5.69476, -0.57917, 4.0, 0.07423],
[-5.43236, -0.72672, 4.0, 0.07526],
[-5.16874, -0.87386, 4.0, 0.07548],
[-4.9046, -1.02041, 4.0, 0.07552],
[-4.64029, -1.16643, 4.0, 0.07549],
[-4.37594, -1.31213, 4.0, 0.07546],
[-4.11161, -1.45752, 4.0, 0.07542],
[-3.84783, -1.6023, 4.0, 0.07522],
[-3.58393, -1.74741, 4.0, 0.07529],
[-3.31985, -1.89254, 4.0, 0.07533],
[-3.05575, -2.03761, 4.0, 0.07533],
[-2.79163, -2.18261, 4.0, 0.07532],
[-2.52754, -2.32756, 4.0, 0.07531],
[-2.26347, -2.47251, 4.0, 0.07531],
[-1.99941, -2.61748, 4.0, 0.07531],
[-1.73535, -2.76246, 4.0, 0.07531],
[-1.47129, -2.90744, 4.0, 0.07531],
[-1.20722, -3.05243, 4.0, 0.07531],
[-0.94315, -3.19741, 4.0, 0.07531],
[-0.67907, -3.34238, 4.0, 0.07531],
[-0.41501, -3.48736, 4.0, 0.07531]]

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

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        # reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 0.5
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        # reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 10 
        STANDARD_TIME = 20
        FASTEST_TIME = 15
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        # reward += steps_reward


        progress_reward = (progress**2) / steps
        if steps <= 5:
            progress_reward = 1 #ignore progress in the first 5 steps



        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        
        direction_diff_reward = 10 * abs(math.cos(direction_diff * math.pi / 20))
        
        reward = 10 * ((distance_reward + speed_reward) ** 2) + progress_reward + direction_diff_reward
        
        if direction_diff > 20:
            reward = 1e-3
            
        if speed_diff > SPEED_DIFF_NO_REWARD:
            reward = 1e-3        
        
        if dist > 0.4:
            reward = 1e-3

        # Zero reward of obviously too slow
        # speed_diff_zero = optimals[2]-speed
        # if speed_diff_zero > 0.5:
        #     reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 3000 # should be adapted to track length and other rewards
        STANDARD_TIME = 18  # seconds (time that is easily done by model)
        FASTEST_TIME = 15 # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15)) + progress_reward
        else:
            finish_reward = 1e-3
        reward += finish_reward
        
        if progress == 75 or progress == 50.0 or progress == 25 or  progress == 10:
            reward += (progress**3) / steps
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = finish_reward


        # Before returning reward, update the variables
        PARAMS.prev_speed = speed
        PARAMS.prev_steering_angle = steering_angle
        PARAMS.prev_direction_diff = direction_diff
        PARAMS.prev_steps = steps

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
        'steps': 1,
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
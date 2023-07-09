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
                        [2.6438, -4.80153, 4.0, 0.07531],
                        [2.90788, -4.94601, 4.0, 0.07525],
                        [3.17212, -5.08915, 4.0, 0.07513],
                        [3.43661, -5.23009, 4.0, 0.07492],
                        [3.70144, -5.36793, 4.0, 0.07464],
                        [3.96669, -5.50172, 3.94271, 0.07535],
                        [4.23244, -5.63034, 3.58091, 0.08245],
                        [4.49871, -5.7526, 3.18687, 0.09194],
                        [4.76555, -5.8671, 2.84532, 0.10205],
                        [5.03291, -5.97234, 2.53899, 0.11317],
                        [5.30071, -6.06666, 2.25709, 0.12579],
                        [5.56876, -6.14761, 1.99408, 0.14042],
                        [5.83671, -6.21241, 1.77666, 0.15517],
                        [6.10394, -6.25774, 1.55117, 0.17474],
                        [6.36941, -6.27953, 1.55117, 0.17172],
                        [6.63133, -6.27256, 1.55117, 0.16891],
                        [6.88679, -6.23097, 1.55117, 0.16686],
                        [7.12987, -6.14585, 1.69343, 0.15209],
                        [7.35875, -6.02558, 1.86188, 0.13887],
                        [7.57308, -5.87699, 2.01191, 0.12963],
                        [7.77266, -5.70475, 2.1421, 0.12307],
                        [7.95716, -5.51235, 2.24714, 0.11862],
                        [8.12602, -5.30259, 2.33673, 0.11524],
                        [8.27866, -5.07801, 2.41, 0.11267],
                        [8.41445, -4.84104, 2.47198, 0.11049],
                        [8.53294, -4.59404, 2.52737, 0.10839],
                        [8.63387, -4.33929, 2.57732, 0.10632],
                        [8.71728, -4.07886, 2.60021, 0.10517],
                        [8.78339, -3.81459, 2.4902, 0.10939],
                        [8.8326, -3.54806, 2.38075, 0.11385],
                        [8.86464, -3.28058, 2.17912, 0.12362],
                        [8.87918, -3.01348, 1.99165, 0.13431],
                        [8.875, -2.74817, 1.76652, 0.15021],
                        [8.85086, -2.48622, 1.57613, 0.1669],
                        [8.80372, -2.22993, 1.5, 0.17373],
                        [8.73019, -1.98239, 1.5, 0.17215],
                        [8.6249, -1.74874, 1.5, 0.17085],
                        [8.48234, -1.53665, 1.5, 0.17037],
                        [8.30202, -1.35364, 1.65925, 0.15484],
                        [8.09344, -1.19852, 1.85696, 0.13998],
                        [7.86369, -1.0684, 2.07625, 0.12717],
                        [7.61779, -0.96022, 2.33092, 0.11525],
                        [7.35953, -0.8709, 2.62682, 0.10403],
                        [7.09181, -0.79744, 3.00092, 0.09251],
                        [6.817, -0.73681, 3.56786, 0.07888],
                        [6.53724, -0.68566, 4.0, 0.0711],
                        [6.25425, -0.64095, 4.0, 0.07163],
                        [5.96965, -0.59949, 4.0, 0.0719],
                        [5.68142, -0.55482, 4.0, 0.07292],
                        [5.39393, -0.50717, 4.0, 0.07285],
                        [5.10745, -0.45587, 4.0, 0.07276],
                        [4.82226, -0.40029, 4.0, 0.07264],
                        [4.53873, -0.33968, 3.9482, 0.07344],
                        [4.25723, -0.2733, 3.75928, 0.07693],
                        [3.97813, -0.20056, 3.61716, 0.07974],
                        [3.70191, -0.12067, 3.5106, 0.08191],
                        [3.4291, -0.03279, 3.43892, 0.08334],
                        [3.16026, 0.06372, 3.41093, 0.08374],
                        [2.8959, 0.16939, 3.41093, 0.08347],
                        [2.63651, 0.28458, 3.41093, 0.08321],
                        [2.38254, 0.40948, 3.41093, 0.08297],
                        [2.13431, 0.54403, 3.44367, 0.08199],
                        [1.8919, 0.68787, 3.45091, 0.08168],
                        [1.65521, 0.84032, 3.24121, 0.08686],
                        [1.42403, 1.00068, 2.93298, 0.09593],
                        [1.19877, 1.16914, 2.67884, 0.105],
                        [0.98012, 1.3462, 2.67884, 0.10502],
                        [0.76927, 1.53279, 2.67884, 0.1051],
                        [0.56838, 1.7307, 2.67884, 0.10527],
                        [0.38015, 1.94162, 2.90534, 0.0973],
                        [0.2033, 2.16307, 3.27561, 0.08652],
                        [0.03578, 2.39248, 3.605, 0.0788],
                        [-0.12378, 2.62824, 3.98961, 0.07136],
                        [-0.27666, 2.86898, 4.0, 0.07129],
                        [-0.42378, 3.11364, 4.0, 0.07137],
                        [-0.56612, 3.36121, 4.0, 0.07139],
                        [-0.70448, 3.6109, 4.0, 0.07137],
                        [-0.83962, 3.86207, 4.0, 0.0713],
                        [-0.97227, 4.11422, 3.33607, 0.0854],
                        [-1.10312, 4.36697, 2.68918, 0.10584],
                        [-1.2295, 4.61361, 2.29331, 0.12084],
                        [-1.35999, 4.85655, 1.98126, 0.13919],
                        [-1.49887, 5.09188, 1.98126, 0.13791],
                        [-1.65015, 5.31563, 1.98126, 0.13633],
                        [-1.81759, 5.5236, 1.98126, 0.13476],
                        [-2.00528, 5.71014, 2.08783, 0.12674],
                        [-2.20887, 5.87823, 2.08783, 0.12645],
                        [-2.42871, 6.02579, 2.10444, 0.12581],
                        [-2.66381, 6.15213, 2.32098, 0.11499],
                        [-2.91098, 6.26007, 2.46608, 0.10937],
                        [-3.1685, 6.35085, 2.63679, 0.10355],
                        [-3.43472, 6.42578, 2.8011, 0.09873],
                        [-3.70824, 6.48593, 2.91632, 0.09603],
                        [-3.98792, 6.53174, 3.03416, 0.0934],
                        [-4.27245, 6.5637, 3.16337, 0.09051],
                        [-4.56035, 6.58243, 3.26124, 0.08847],
                        [-4.85007, 6.58839, 3.30362, 0.08772],
                        [-5.14011, 6.5819, 3.19355, 0.09084],
                        [-5.42908, 6.56335, 3.03, 0.09556],
                        [-5.71585, 6.53319, 2.8214, 0.1022],
                        [-5.99926, 6.491, 2.59008, 0.11063],
                        [-6.2782, 6.43635, 2.39602, 0.11863],
                        [-6.55146, 6.36838, 2.07749, 0.13554],
                        [-6.81757, 6.2857, 1.82859, 0.15239],
                        [-7.07472, 6.18635, 1.82859, 0.15076],
                        [-7.32099, 6.06837, 1.82859, 0.14933],
                        [-7.55215, 5.92674, 1.82859, 0.14825],
                        [-7.7622, 5.75617, 2.02613, 0.13355],
                        [-7.95327, 5.5634, 2.16545, 0.12534],
                        [-8.12605, 5.35242, 2.3004, 0.11854],
                        [-8.28111, 5.12668, 2.42424, 0.11297],
                        [-8.41891, 4.88913, 2.54153, 0.10806],
                        [-8.54005, 4.64228, 2.60239, 0.10566],
                        [-8.64454, 4.38802, 2.63933, 0.10415],
                        [-8.73251, 4.12801, 2.51114, 0.10931],
                        [-8.80404, 3.86373, 2.36361, 0.11583],
                        [-8.85866, 3.59646, 2.17135, 0.12563],
                        [-8.89596, 3.32748, 1.9865, 0.1367],
                        [-8.91422, 3.0581, 1.76505, 0.15297],
                        [-8.91128, 2.79, 1.57658, 0.17006],
                        [-8.88381, 2.52541, 1.57658, 0.16873],
                        [-8.82795, 2.26733, 1.57658, 0.16748],
                        [-8.73745, 2.02045, 1.57658, 0.16678],
                        [-8.60513, 1.79183, 1.96922, 0.13414],
                        [-8.4465, 1.57721, 2.14673, 0.12432],
                        [-8.26555, 1.37573, 2.34382, 0.11554],
                        [-8.06553, 1.18644, 2.59698, 0.10604],
                        [-7.84964, 1.00803, 2.85123, 0.09823],
                        [-7.62028, 0.83942, 3.17847, 0.08956],
                        [-7.37988, 0.67932, 3.60332, 0.08016],
                        [-7.13079, 0.52624, 4.0, 0.07309],
                        [-6.87513, 0.37867, 4.0, 0.0738],
                        [-6.61472, 0.23511, 4.0, 0.07434],
                        [-6.35097, 0.09415, 4.0, 0.07476],
                        [-6.08487, -0.04573, 4.0, 0.07516],
                        [-5.81882, -0.18679, 4.0, 0.07528],
                        [-5.55307, -0.32844, 4.0, 0.07529],
                        [-5.28746, -0.47037, 4.0, 0.07529],
                        [-5.02199, -0.61257, 4.0, 0.07529],
                        [-4.75664, -0.75502, 4.0, 0.07529],
                        [-4.4914, -0.89768, 4.0, 0.07529],
                        [-4.22628, -1.04058, 4.0, 0.07529],
                        [-3.96128, -1.1837, 4.0, 0.0753],
                        [-3.69639, -1.32706, 4.0, 0.0753],
                        [-3.43161, -1.47064, 4.0, 0.0753],
                        [-3.16695, -1.61445, 4.0, 0.0753],
                        [-2.90241, -1.75849, 4.0, 0.0753],
                        [-2.63798, -1.90275, 4.0, 0.07531],
                        [-2.37366, -2.04724, 4.0, 0.07531],
                        [-2.10947, -2.19197, 4.0, 0.07531],
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
        STANDARD_TIME = 37
        FASTEST_TIME = 27
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
        STANDARD_TIME = 37  # seconds (time that is easily done by model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
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
    print("Params : " , params)
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
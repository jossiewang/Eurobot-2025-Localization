import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from obstacle_detector.msg import Obstacles

import numpy as np

import time

class LidarLocalization(Node): # inherit from Node

    def __init__(self):
        # super().__init__('lidar_localization_node')
        # Set the side (0 for blue and 1 for yellow); TODO: use param
        side = 1
        if side == 0:
            landmarks_map = [
                        np.array([-0.094, 0.052]),
                        np.array([-0.094, 1.948]),
                        np.array([3.094, 1.0])
                    ]
        elif side == 1:
            landmarks_map = [
                        np.array([3.094, 0.052]),
                        np.array([3.094, 1.948]),
                        np.array([-0.094, 1.0])
                    ]
        # set debug mode
        self.debug_mode = False

        # ros settings
        # self.lidar_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'lidar_pose', 10)
        # self.subscription = self.create_subscription(
        #     Obstacles,
        #     'raw_obstacles',
        #     self.obstacle_callback,
        #     10)
        # self.subscription = self.create_subscription(
        #     PoseWithCovarianceStamped, 
        #     'pred_pose',
        #     self.pred_pose_callback,
        #     10
        # )
        # self.subscription  # prevent unused variable warning

        # ros debug logger
        # self.get_logger().debug('Lidar Localization Node has been initialized')

        self.landmarks_map = landmarks_map
        self.init_landmarks_map(self.landmarks_map)
        self.robot_pose = []
        self.P_pred = []
        self.newPose = False
        self.R = np.array([[0.05**2, 0.0], [0.0, 0.05**2]]) # R: measurement noise; TODO: tune the value

    # def listener_callback(self, msg):
    #     self.get_logger().info('I heard: "%s"' % msg.data)
    #     # lidar_pose_msg
    #     lidar_pose_msg = PoseWithCovarianceStamped()
    #     lidar_pose_msg.header.stamp = self.get_clock().now().to_msg()
    #     lidar_pose_msg.header.frame_id = 'base_link'
    #     lidar_pose_msg.pose.pose.position.x = 1.0
    #     lidar_pose_msg.pose.pose.position.y = 2.0
    #     lidar_pose_msg.pose.pose.position.z = 3.0
    #     lidar_pose_msg.pose.pose.orientation.x = 0.0
    #     lidar_pose_msg.pose.pose.orientation.y = 0.0
    #     lidar_pose_msg.pose.pose.orientation.z = 0.0
    #     lidar_pose_msg.pose.pose.orientation.w = 1.0
    #     self.lidar_pose_pub.publish(lidar_pose_msg)
    
    def obstacle_callback(self, msg):
        # self.get_logger().debug('obstacle detected')
        # obstacle operation
        self.obs_raw = []
        for obs in msg.circles:
            self.obs_raw.append(np.array([obs.center.x, obs.center.y]))
        self.obs_time = msg.header.stamp
        # data processing
        if self.newPose == False: # Check if robot_pose or P_pred is empty
            self.print_debug("no new robot pose or P_pred")
            return
        self.landmarks_candidate = self.get_landmarks_candidate(self.landmarks_map, self.obs_raw, self.robot_pose, self.P_pred, self.R)
        self.landmarks_set = self.get_landmarks_set(self.landmarks_candidate)
        if len(self.landmarks_set) == 0:
            self.print_debug("empty landmarks set")
            return
        self.lidar_pose, self.P_post = self.get_lidar_pose(self.landmarks_set, self.landmarks_map)
        # clear used data
        self.clear_data()
    
    def pred_pose_callback(self, msg):
        # self.print_debug("Robot pose callback triggered")
        self.newPose = True
        orientation = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)[2] # raw, pitch, *yaw
        # check orientation range
        if orientation < 0:
            orientation += 2 * np.pi
        self.robot_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, orientation])
        self.P_pred = np.array([
            [msg.pose.covariance[0], 0, 0],
            [0, msg.pose.covariance[7], 0],
            [0, 0, msg.pose.covariance[35]]
        ])

    def init_landmarks_map(self, landmarks_map):
        self.landmarks_map = landmarks_map
        # calculate the geometry description of landmarks
        self.geometry_description_map = {}
        NUM_LANDMARKS = len(landmarks_map)
        for i in range(NUM_LANDMARKS):
            for j in range(i + 1, NUM_LANDMARKS):
                if i == j:
                    continue
                d_ij = np.linalg.norm(landmarks_map[i] - landmarks_map[j])
                self.geometry_description_map[(i, j)] = d_ij
                
    def clear_data(self):
        self.obs_raw = []
        self.robot_pose = np.array([])
        self.P_pred = np.array([])
        self.landmarks_candidate = []
        self.landmarks_set = []
        self.newPose = False

    def get_obs_candidate(self, robot_pose, P_pred, R, landmark, obs_raw):
        obs_candidates = []
        x_r, y_r, phi_r = robot_pose
        x_o, y_o = landmark
        r_prime = np.sqrt((x_o - x_r) ** 2 + (y_o - y_r) ** 2)
        # theta_rob = np.arctan2(
        temp = angle_limit_checking(np.arctan2(y_o - y_r, x_o - x_r)) # limit checking is not necessary right?
        theta_prime = angle_limit_checking(temp - phi_r)

        H = np.array([
            [-(x_o - x_r) / r_prime, -(y_o - y_r) / r_prime, 0],
            [(y_o - y_r) / r_prime ** 2, -(x_o - x_r) / r_prime ** 2, -1]
        ])
        S = H @ P_pred @ H.T + R
        S_inv = np.linalg.inv(S)
        S_det = np.linalg.det(S)
        normalizer = 1 / np.sqrt((2 * np.pi) ** 2 * S_det)

        likelihood_threshold = 0.1  # Define a threshold for likelihood

        for obs in obs_raw:
            r_z = np.sqrt(obs[0] ** 2 + obs[1] ** 2)
            theta_z = np.arctan2(obs[1], obs[0])
            y = np.array([r_z - r_prime, angle_limit_checking(theta_z - theta_prime)])
            di_square = y.T @ S_inv @ y
            likelihood = normalizer * np.exp(-0.5 * di_square)
            # normalize: max likelihood is for di_square = 0
            likelihood = likelihood / normalizer
            if likelihood > likelihood_threshold:
                obs_candidates.append({'position': obs, 'probability': likelihood})
        
        return obs_candidates

    def get_landmarks_candidate(self, landmarks_map, obs_raw, robot_pose, P_pred, R):
        landmarks_candidate = []
        for landmark in landmarks_map:
            candidate = {
                'landmark': landmark,
                'obs_candidates': self.get_obs_candidate(robot_pose, P_pred, R, landmark, obs_raw)
            }
            landmarks_candidate.append(candidate)
        # print landmarks_candidate for debug
        if self.debug_mode:
            for i, landmark in enumerate(landmarks_candidate):
                print(f"Landmark {i + 1}: {landmark['landmark']}")
                for j, obs_candidate in enumerate(landmark['obs_candidates']):
                    print(f"Obs {j + 1}: {obs_candidate['position']} with probability {obs_candidate['probability']}")
        return landmarks_candidate

    def get_landmarks_set(self, landmarks_candidate):
        landmarks_set = []
        # ## Permutations for two landmarks
        # landmark_indices = list(range(len(landmarks_candidate))) # Generate all combinations of landmarks
        # for comb in combinations(landmark_indices, 2):
        #     for i in range(len(landmarks_candidate[comb[0]]['obs_candidates'])):
        #         for j in range(len(landmarks_candidate[comb[1]]['obs_candidates'])):
        #             set = {
        #             'beacons': {
        #                 comb[0]: landmarks_candidate[comb[0]]['obs_candidates'][i]['position'],
        #                 comb[1]: landmarks_candidate[comb[1]]['obs_candidates'][j]['position']
        #                 }
        #             }
        #             # consistency of the set
        #             set['consistency'] = self.get_geometry_consistency(set['beacons'])
        #             if set['consistency'] < 0.99: #TODO: tune the value
        #                 self.get_logger().debug(f"Geometry consistency is less than 0.96: {set['consistency']}")
        #                 continue
        #             # probability of the set
        #             set['probability_set'] = landmarks_candidate[comb[0]]['obs_candidates'][i]['probability'] * landmarks_candidate[comb[1]]['obs_candidates'][j]['probability']
        #             landmarks_set.append(set)
        ## Permutations for three landmarks
        for i in range(len(landmarks_candidate[0]['obs_candidates'])):
            for j in range(len(landmarks_candidate[1]['obs_candidates'])):
                for k in range(len(landmarks_candidate[2]['obs_candidates'])):
                    set = {
                        'beacons': {
                            0: landmarks_candidate[0]['obs_candidates'][i]['position'],
                            1: landmarks_candidate[1]['obs_candidates'][j]['position'],
                            2: landmarks_candidate[2]['obs_candidates'][k]['position']
                        }
                    }
                    # consistency of the set
                    set['consistency'] = self.get_geometry_consistency(set['beacons'])
                    if set['consistency'] < 0.9:
                        # self.get_logger().debug(f"Geometry consistency is less than 0.9: {set['consistency']}")
                        continue
                    # probability of the set
                    set['probability_set'] = landmarks_candidate[0]['obs_candidates'][i]['probability'] * landmarks_candidate[1]['obs_candidates'][j]['probability'] * landmarks_candidate[2]['obs_candidates'][k]['probability']
                    landmarks_set.append(set)

        # print landmarks_set for debug
        if self.debug_mode:
            for i, set in enumerate(landmarks_set):
                print(f"Set {i + 1}:")
                print(f"Probability: {set['probability_set']}")
                print(f"Geometry Consistency: {set['consistency']}")

        return landmarks_set

    def get_lidar_pose(self, landmarks_set, landmarks_map):
        if not landmarks_set:
            raise ValueError("landmarks_set is empty")
        start_time = time.time()
        # prefer the set with more beacons
        landmarks_set = sorted(landmarks_set, key=lambda x: len(x['beacons']), reverse=True)
        # with the most beacon possible, prefer the set with the highest probability_set; TODO: better way to sort?
        max_likelihood = max(set['probability_set'] for set in landmarks_set)
        max_likelihood_idx = next(i for i, set in enumerate(landmarks_set) if set['probability_set'] == max_likelihood)
        print('Time cost for sorting: ', time.time() - start_time)

        lidar_pose = np.zeros(3)
        P_post = np.diag([0.05**2, 0.05**2, 0.05**2]) # what should the optimal value be?

        # If the most likely set has at least 3 beacons
        if len(landmarks_set[max_likelihood_idx]['beacons']) >= 3:
            start_time = time.time()
            beacons = [landmarks_set[max_likelihood_idx]['beacons'][i] for i in range(3)]
            A = np.zeros((2, 2))
            b = np.zeros(2)
            dist_beacon_robot = [np.linalg.norm(beacon) for beacon in beacons]

            A[0, 0] = 2 * (landmarks_map[0][0] - landmarks_map[2][0])
            A[0, 1] = 2 * (landmarks_map[0][1] - landmarks_map[2][1])
            A[1, 0] = 2 * (landmarks_map[1][0] - landmarks_map[2][0])
            A[1, 1] = 2 * (landmarks_map[1][1] - landmarks_map[2][1])

            b[0] = (landmarks_map[0][0]**2 - landmarks_map[2][0]**2) + (landmarks_map[0][1]**2 - landmarks_map[2][1]**2) + (dist_beacon_robot[2]**2 - dist_beacon_robot[0]**2)
            b[1] = (landmarks_map[1][0]**2 - landmarks_map[2][0]**2) + (landmarks_map[1][1]**2 - landmarks_map[2][1]**2) + (dist_beacon_robot[2]**2 - dist_beacon_robot[1]**2)

            try:
                X = np.linalg.solve(A.T @ A, A.T @ b)
                print('Time cost for solving X: ', time.time() - start_time)
                start_time = time.time()
                lidar_pose[0] = X[0]
                lidar_pose[1] = X[1]

                robot_sin = 0
                robot_cos = 0

                for i in range(3):
                    theta = angle_limit_checking(np.arctan2(landmarks_map[i][1] - lidar_pose[1], landmarks_map[i][0] - lidar_pose[0]) - np.arctan2(beacons[i][1], beacons[i][0]))
                    robot_sin += np.sin(theta)
                    robot_cos += np.cos(theta)

                    lidar_pose[2] = angle_limit_checking(np.arctan2(robot_sin, robot_cos))

                print('Time cost for find theta: ', time.time() - start_time)
                start_time = time.time()

                P_post[0, 0] /= max_likelihood
                P_post[1, 1] /= max_likelihood
                P_post[2, 2] /= max_likelihood
                print('Time cost for adjusting P_post: ', time.time() - start_time)
                start_time = time.time()
                # publish the lidar pose
                lidar_pose_msg = PoseWithCovarianceStamped()
                print('Time cost for creating lidar_pose_msg: ', time.time() - start_time)
                start_time = time.time()
                # self.lidar_pose_msg.header.stamp = self.get_clock().now().to_msg() # TODO: compensation
                lidar_pose_msg.header.frame_id = 'map' #TODO: param
                print('Time cost for setting header: ', time.time() - start_time)
                start_time = time.time()
                lidar_pose_msg.pose.pose.position.x = lidar_pose[0]
                print('Time cost for setting x: ', time.time() - start_time)
                lidar_pose_msg.pose.pose.position.y = lidar_pose[1]
                lidar_pose_msg.pose.pose.position.z = 0.0
                lidar_pose_msg.pose.pose.orientation.x = 0.0
                lidar_pose_msg.pose.pose.orientation.y = 0.0
                start_time = time.time()
                lidar_pose_msg.pose.pose.orientation.z = np.sin(lidar_pose[2] / 2)
                print('Time cost for setting z: ', time.time() - start_time)
                lidar_pose_msg.pose.pose.orientation.w = np.cos(lidar_pose[2] / 2)
                start_time = time.time()
                lidar_pose_msg.pose.covariance[0] = P_post[0, 0]    # TODO: diaganol shouldn't be 0?
                print('Time cost for setting covariance[0]: ', time.time() - start_time)
                lidar_pose_msg.pose.covariance[7] = P_post[1, 1]
                lidar_pose_msg.pose.covariance[35] = P_post[2, 2]
                # print('Time cost for preparing to publish: ', time.time() - start_time)
                # self.get_logger().debug(f"lidar_pose: {lidar_pose}")
                # self.lidar_pose_pub.publish(lidar_pose_msg)
                # self.get_logger().debug("Published lidar_pose message")

            except np.linalg.LinAlgError as e:
                # self.get_logger().warn("Linear algebra error: {}".format(e))
                print("Linear algebra error: {}".format(e))
        else:
            # self.get_logger().debug("not enough beacons")
            print("not enough beacons")

        return lidar_pose, P_post

    def get_geometry_consistency(self, beacons):
        geometry_description = {}
        consistency = 1.0
        lenB = len(beacons)

        # lenB can be 2, 3 or 4
        # use the index of the beacons to calculate the distance between them
        for i in beacons:
            for j in beacons:
                if i == j:
                    continue
                geometry_description[(i, j)] = np.linalg.norm(beacons[i] - beacons[j])
                # self.get_logger().debug(f"Beacon {i} to Beacon {j} distance: {geometry_description[(i, j)]}")
                if (i, j) in self.geometry_description_map:
                    expected_distance = self.geometry_description_map[(i, j)]
                    consistency *= 1 - np.abs(geometry_description[(i, j)] - expected_distance) / expected_distance
                # if the index is not found in map, it is probably on the lower triangle of the matrix

        return consistency

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk
    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss
    return q

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.atan2(t3, t4)
    return yaw_z # in radians

def angle_limit_checking(theta):
    while theta > np.pi:
        theta -= 2 * np.pi
    while theta <= -np.pi:
        theta += 2 * np.pi
    return theta

def main(args=None):
    rclpy.init(args=args)

    lidar_localization = LidarLocalization()

    rclpy.spin(lidar_localization)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_localization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
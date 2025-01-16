import unittest
import numpy as np
import time
from lidar_member_function import LidarLocalization

class TestLidarLocalization(unittest.TestCase):
    def setUp(self):
        self.beacon_prob = LidarLocalization()
        self.robot_pose = [1.5, 1, 0.785]
        self.P_pred = np.array([[0.05, 0, 0], [0, 0.05, 0], [0, 0, 0.01]])
        self.R = np.array([[0.001, 0], [0, 0.001]])
        self.landmarks_map = [np.array([0.0, 0.0]), np.array([0.0, 2.0]), np.array([3.0, 1.0])]
        self.beacon_prob.init_landmarks_map(self.landmarks_map)
        # obs_raw_map: 0,0; 0.1,0; 0.2,0; 3,1; 3.1,1; 0,2
        obs_raw_map = [np.array([0.0, 0.0]), np.array([0.1, 0.0]), np.array([0.2, 0.0]), np.array([3.0, 1.0]), np.array([3.1, 1.0]), np.array([0.0, 2.0])]
        # change into robot frame
        rotation_matrix = np.array([[np.cos(self.robot_pose[2]), np.sin(self.robot_pose[2])], [-1 * np.sin(self.robot_pose[2]), np.cos(self.robot_pose[2])]])
        self.obs_raw = []
            # obs_raw_robot = []
        for obs_raw in obs_raw_map:
            relative_vector = np.array([obs_raw[0] - self.robot_pose[0], obs_raw[1] - self.robot_pose[1]])
            self.obs_raw.append(np.dot(rotation_matrix, relative_vector))
        # give more fake data to mess around
        obs_raw_mess = [np.array([0.5, 0.7]), np.array([0.6, 0.2]), np.array([0.4, 0.3]), np.array([3.2, 1.6]), np.array([2.5, 2]), np.array([2.3, 1.0]), np.array([0.3, 1.0]), np.array([1.1, 1.1]), np.array([0.2, 0.0])]
        self.obs_raw.extend(obs_raw_mess)
        # print time to evaluate the time cost
        start_time = time.time()
        landmarks_candidate = self.beacon_prob.get_landmarks_candidate(self.landmarks_map, self.obs_raw, self.robot_pose, self.P_pred, self.R)
        print('Time cost for landmarks_candidate: ', time.time() - start_time)
        start_time = time.time()
        self.landmarks_candidate = landmarks_candidate
        print('Time cost for landmarks_candidate: ', time.time() - start_time)

    def test_get_robot_pose(self):
        start_time = time.time()
        landmarks_set = self.beacon_prob.get_landmarks_set(self.landmarks_candidate)
        print('Time cost for landmarks_set: ', time.time() - start_time)
        start_time = time.time()
        self.lidar_pose, self.P_post = self.beacon_prob.get_lidar_pose(landmarks_set, self.landmarks_map)
        print('Time cost for lidar_pose: ', time.time() - start_time)
        self.assertEqual(len(landmarks_set), 6)  # 2 * 2 * 2 = 8 permutations
        for set in landmarks_set:
            self.assertIn('beacons', set)
            self.assertIn('geometry_description', set)
            self.assertIn('probability_set', set)
            self.assertEqual(len(set['beacons']), 3)
            self.assertIsInstance(set['probability_set'], float)

    # def test_get_landmarks_set_empty_candidates(self):
    #     landmarks_candidate = [
    #         {'landmark': np.array([1.0, 2.0]), 'obs_candidates': []},
    #         {'landmark': np.array([3.0, 4.0]), 'obs_candidates': []},
    #         {'landmark': np.array([5.0, 6.0]), 'obs_candidates': []}
    #     ]
    #     landmarks_set = self.beacon_prob.get_landmarks_set(landmarks_candidate)
    #     self.assertEqual(len(landmarks_set), 0)

if __name__ == '__main__':
    unittest.main()
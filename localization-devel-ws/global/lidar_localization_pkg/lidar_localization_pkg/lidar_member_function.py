import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from obstacle_detector.msg import Obstacles

import numpy as np

class LidarLocalization(Node): # inherit from Node

    def __init__(self):
        super().__init__('lidar_localization_node') # name of the node
        # publisher
        self.lidar_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'lidar_pose', 10)
        # subscriber
        self.subscription = self.create_subscription(
            Obstacles,
            'raw_obstacles',
            self.obstacle_callback,
            10)
        self.subscription  # prevent unused variable warning

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
        self.get_logger().info('obstacle detected')
        # obstacle operation
        self.obs_raw = []
        for obs in msg.circles:
            self.obs_raw.append(np.array([obs.center.x, obs.center.y]))
        # use log to print what it get
        self.get_logger().info('obs_raw: %s' % self.obs_raw)

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
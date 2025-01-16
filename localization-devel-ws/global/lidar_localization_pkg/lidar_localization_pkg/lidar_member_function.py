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

        # ros debug logger
        self.get_logger().debug('Lidar Localization Node has been initialized')

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
        self.get_logger().debug('obstacle detected')
        # obstacle operation
        self.obs_raw = []
        for obs in msg.circles:
            self.obs_raw.append(np.array([obs.center.x, obs.center.y]))
        # use log to print what it get
        self.get_logger().debug('obs_raw: %s' % self.obs_raw)

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
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
    
def euler_from_quaternion(self, x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z # in radians

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
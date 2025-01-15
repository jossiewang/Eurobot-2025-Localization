import rclpy
from rclpy.node import Node

from std_msgs.msg import String # add to package.xml
from geometry_msgs.msg import PoseWithCovarianceStamped

class LidarLocalization(Node): # inherit from Node

    def __init__(self):
        super().__init__('lidar_localization_node') # name of the node
        # publisher
        self.lidar_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'lidar_pose', 10)
        # subscriber
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        msg = String()
        # lidar_pose_msg
        lidar_pose_msg = PoseWithCovarianceStamped()
        lidar_pose_msg.header.stamp = self.get_clock().now().to_msg()
        lidar_pose_msg.header.frame_id = 'base_link'
        lidar_pose_msg.pose.pose.position.x = 1.0
        lidar_pose_msg.pose.pose.position.y = 2.0
        lidar_pose_msg.pose.pose.position.z = 3.0
        lidar_pose_msg.pose.pose.orientation.x = 0.0
        lidar_pose_msg.pose.pose.orientation.y = 0.0
        lidar_pose_msg.pose.pose.orientation.z = 0.0
        lidar_pose_msg.pose.pose.orientation.w = 1.0
        self.lidar_pose_pub.publish(lidar_pose_msg)

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
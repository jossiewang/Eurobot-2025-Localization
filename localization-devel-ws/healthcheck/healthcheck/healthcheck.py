import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# from datetime import datetime  # Import for date and time
# import os  # Import for file operations

class HealthCheckNode(Node):
    def __init__(self):
        super().__init__('healthcheck_node')
        
        # Parameters
        self.declare_parameter('robot_frame_id', 'base_footprint')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('rival_frame_id', 'rival/base_footprint')
        self.declare_parameter('lidar_frame_id', 'laser')

        self.p_robot_frame_id = self.get_parameter('robot_frame_id').get_parameter_value().string_value
        self.p_map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value
        self.p_rival_frame_id = self.get_parameter('rival_frame_id').get_parameter_value().string_value
        self.p_lidar_frame_id = self.get_parameter('lidar_frame_id').get_parameter_value().string_value

        # Subscribers: final_pose, local_filter, lidar_pose, imu/data_cov, odom2map, rival/final_pose
        self.subscription = self.create_subscription(
            PoseStamped,
            'odom2map',
            self.odom2map_callback,
            10
        )
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'lidar_pose',
            self.lidar_pose_callback,
            10
        )
        self.subscription # prevent unused variable warning

        # TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # # Create health report file
        # self.create_health_report_file()

        self.check_localization_ok()

        # Timer for health check (3 seconds interval)
        self.timer = self.create_timer(3.0, self.health_check_timer_callback)

        self.wheel_slip_first = True

    # def create_health_report_file(self):
    #     # Generate the filename based on the current date and time
    #     now = datetime.now()
    #     filename = now.strftime("%Y-%m-%d_%H-%M-%S_health_report.txt")
    #     report_dir = '/user/localization/localization_ws/src/localization-devel-ws/healthcheck/report'

    #     # Ensure the directory exists
    #     os.makedirs(report_dir, exist_ok=True)

    #     # Full path to the report file
    #     self.report_file_path = os.path.join(report_dir, filename)

    #     # Create the file and write the header
    #     with open(self.report_file_path, 'w') as file:
    #         file.write("Health Report\n")
    #         file.write(f"Generated on: {now.strftime('%Y-%m-%d %H:%M:%S')}\n")
    #         file.write("=" * 40 + "\n")
    #     self.get_logger().info(f"Health report file created: {self.report_file_path}")

    def check_localization_ok(self):
        # Conditions to satisfy for localization ok
        # 1. TF is published and without error; base_footprint and rival/base_footprint are published
        self.check_tf_ok()
        # 2. local_filter, odom2map and imu/data_cov are published (TODO) (what;s the difference oddom2map and local_filter? can they be merged?)
        # 3. lidar_pose is published and agree with either initial pose or camera pose (TODO)
        # 4. warn if camera_pose is missing (TODO)
        # if localization_ok, response to main (a service?), and ekf, lidar param set to 'running' (TODO)
        return True
    
    def health_check_timer_callback(self):
        self.dead_wheel_slip_estimation()
        # self.check_lidar_delay()
        # self.check_final_pose()

    def dead_wheel_slip_estimation(self):
        # Check for dead wheel slip estimation
        # Check the availability of the odom2map and lidar_pose
        if not hasattr(self, 'odom2map') or not hasattr(self, 'lidar_pose'):
            self.get_logger().warn("odom2map or lidar_pose not available")
            return False
        if not self.wheel_slip_first:
            # compare the displacement of the odometry and the lidar pose
            odom_displacement_x = self.odom2map.pose.position.x - self.odom_x_prev
            odom_displacement_y = self.odom2map.pose.position.y - self.odom_y_prev
            lidar_displacement_x = self.lidar_pose.pose.pose.position.x - self.lidar_x_prev
            lidar_displacement_y = self.lidar_pose.pose.pose.position.y - self.lidar_y_prev
            slip_x = abs(odom_displacement_x - lidar_displacement_x)
            slip_y = abs(odom_displacement_y - lidar_displacement_y)
            self.get_logger().info(f"Slip X: {slip_x}, Slip Y: {slip_y}")
            # the checking frequency cannot be too high, otherwise it will be affected by lidar's large noise,
            # additionally, the noise of lidar should be within 1 cm,
            # so in 3 seconds, maybe the slip could be within 3 cm
            
            # # Append slip data to the health report file
            # with open(self.report_file_path, 'a') as file:
            #     file.write(f"Slip X: {slip_x}, Slip Y: {slip_y}\n")

            if slip_x > 0.03 or slip_y > 0.03:
                self.get_logger().warn(f"Dead wheel slip detected! Slip X: {slip_x}, Slip Y: {slip_y}")
                # a service to warn lidar_localization
                return False
        self.odom_x_prev = self.odom2map.pose.position.x
        self.odom_y_prev = self.odom2map.pose.position.y
        self.lidar_x_prev = self.lidar_pose.pose.pose.position.x
        self.lidar_y_prev = self.lidar_pose.pose.pose.position.y
        self.wheel_slip_first = False
        return True
    

    def check_tf_ok(self):
        tf_retry_count = 0
        while rclpy.ok(): # is it safe to use while loop? it is a blocking function
            tf_retry_count += 1
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))

            tf_ok = True

            # check /base_footprint, /laser, /rival/base_footprint
            try:
                self.tf_buffer.can_transform(
                    self.p_robot_frame_id,
                    self.p_map_frame_id,
                    rclpy.time.Time()
                )
            except Exception as e:
                tf_ok = False
                self.get_logger().warn(f"TF lookup failed: {e}")
            try:
                self.tf_buffer.can_transform(

                    self.p_rival_frame_id,
                    self.p_map_frame_id,
                    rclpy.time.Time()
                )
            except Exception as e:
                tf_ok = False
                self.get_logger().warn(f"TF lookup failed: {e}")

            if tf_ok:
                return True

            self.get_logger().warn("[Lidar Localization]: TF not OK")

            if tf_retry_count % 20 == 0:
                self.get_logger().error(
                    f"[Lidar Localization]: TF error after retry {tf_retry_count} times"
                )

        return False
    
    # def check_lidar_delay(self):
    #     # Check delay time for lidar, 
    #     # compare the trend of orientation because that is the most obvoius and stable data

    #     # Initialize a list to store the latest 50 orientation z data
    #     if not hasattr(self, 'orientation_z_data'):
    #         self.orientation_z_data = []

    #     # Append the latest orientation z data
    #     self.orientation_z_data.append(self.lidar_pose.pose.pose.orientation.z)

    #     # Ensure the list only keeps the latest 50 entries
    #     if len(self.orientation_z_data) > 50:
    #         self.orientation_z_data.pop(0)

    #     # Example: Log the orientation data for debugging
    #     self.get_logger().info(f"Latest orientation z data: {self.orientation_z_data}")
    #     return True

    # def check_final_pose(self):
    #     # compare odom2map, lidar_pose and camera_pose
    #     # warn if any one of them has a large difference

    def odom2map_callback(self, msg):
        self.odom2map = msg

    def lidar_pose_callback(self, msg):
        self.lidar_pose = msg

def main(args=None):
    rclpy.init(args=args)
    node = HealthCheckNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import PoseStamped , PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import csv
import time

#!/usr/bin/env python


class ResidualTest:
    def __init__(self):
        rospy.init_node('timestamp_data', anonymous=True)
        
        rospy.Subscriber('/robot/imu/data', Imu, self.IMUcallback)
        rospy.Subscriber('/robot/scan', LaserScan, self.LScallback)
        rospy.Subscriber('/robot/lidar_bonbonbon', PoseWithCovarianceStamped, self.LDcallback)
        rospy.Subscriber('/robot/Toposition', Odometry, self.TPcallback)
        rospy.Subscriber('/robot/final_pose', Odometry, self.FPcallback)
        
        self.csv_file = open('/home/user/bag_info/scripts/timestamp_data.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time', 'topic'])
        
        self.residual = None
        self.covariance = None
    
    def IMUcallback(self, data):
        current_time = time.time()
        self.csv_writer.writerow([current_time, '/imu'])
        if self.csv_file:
            self.csv_file.flush()

    def LScallback(self, data):
        current_time = time.time()
        self.csv_writer.writerow([current_time, '/scan'])
        if self.csv_file:
            self.csv_file.flush()
    
    def LDcallback(self, data):
        

    def write_to_csv(self):
        if self.residual is not None and self.covariance is not None:
            residual_x = self.residual.x
            residual_y = self.residual.y
            residual_theta = self.residual.z
            
            variance_x = math.sqrt(self.covariance[0])
            variance_y = math.sqrt(self.covariance[7])
            variance_theta = math.sqrt(self.covariance[35])
            
            self.csv_writer.writerow([residual_x, residual_y, residual_theta, 
                                      3 * variance_x, 3 * variance_y, 3 * variance_theta])
            self.residual = None
            self.covariance = None

    def run(self):
        rospy.spin()
        self.csv_file.close()

if __name__ == '__main__':
    try:
        residual_test = ResidualTest()
        residual_test.run()
    except rospy.ROSInterruptException:
        pass
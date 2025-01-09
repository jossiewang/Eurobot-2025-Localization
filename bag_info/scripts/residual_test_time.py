import rospy
import csv
import math
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped

#!/usr/bin/env python


class ResidualTest:
    def __init__(self):
        rospy.init_node('residual_test', anonymous=True)
        
        self.residual_sub = rospy.Subscriber('/robot/residual', Vector3, self.residual_callback)
        self.cov_sub = rospy.Subscriber('/robot/ekf_prediction_pose', PoseWithCovarianceStamped, self.cov_callback)
        
        self.csv_file = open('/home/user/bag_info/scripts/residual_data.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time', 'residual_x', 'residual_y', 'residual_theta', 
                                  '3x_variance_x', '3x_variance_y', '3x_variance_theta'])
        
        self.residual = None
        self.covariance = None
        self.time = None

    def residual_callback(self, msg):
        self.residual = msg
        self.write_to_csv()

    def cov_callback(self, msg):
        self.covariance = msg.pose.covariance
        self.time = msg.header.stamp.to_sec()
        self.write_to_csv()

    def write_to_csv(self):
        if self.residual is not None and self.covariance is not None and self.time is not None:
            residual_x = self.residual.x
            residual_y = self.residual.y
            residual_theta = self.residual.z
            
            variance_x = math.sqrt(self.covariance[0])
            variance_y = math.sqrt(self.covariance[7])
            variance_theta = math.sqrt(self.covariance[35])
            
            self.csv_writer.writerow([self.time, residual_x, residual_y, residual_theta, 
                                      3 * variance_x, 3 * variance_y, 3 * variance_theta])
            self.residual = None
            self.covariance = None
            self.time = None

    def run(self):
        rospy.spin()
        self.csv_file.close()

if __name__ == '__main__':
    try:
        residual_test = ResidualTest()
        residual_test.run()
    except rospy.ROSInterruptException:
        pass
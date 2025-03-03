#include "draw_path/draw_path.h"

Draw::Draw () : Node("draw_path_node"){

    rclcpp::Clock clock;

    lidar_Pose_.header.frame_id = "robot/map";
    lidar_Pose_.header.stamp = clock.now();

    draw_lidar_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("pose", 10, std::bind(&Draw::lidar_callback, this, _1));
    draw_lidar_pub_ = this->create_publisher<nav_msgs::msg::Path>("lidar_Pose", 10);
}

void Draw::lidar_callback(const nav_msgs::msg::Odometry::ConstPtr& msg){

    geometry_msgs::msg::PoseStamped lidar_PoseStamped;

    lidar_PoseStamped.pose = msg->pose.pose;
    lidar_PoseStamped.header.frame_id = "robot/map";
    lidar_PoseStamped.header.stamp = msg->header.stamp;

    lidar_Pose_.poses.push_back(lidar_PoseStamped);

    draw_lidar_pub_->publish(lidar_Pose_);
}

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Draw>());
    rclcpp::shutdown();

    return 0;
}
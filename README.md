# Eurobot-2025-Localization

## How to Use
### Run on the Robots
#### w/ ready_signal (TODO)
    docker compose -f docker-run-rs.yaml up
#### w/o ready_signal (TODO)
    # choose the side
    docker compose -f docker-run-blue.yaml up
    docker compose -f docker-run-yellow.yaml up
### Run on Local Machine
#### develop w/o sensors w/ ros1
    cd docker/local-ros1
    docker compose up
#### develop w/ sensors w/ ros2
    cd docker/local-ros2
    docker compose up
#### simulation on local machine (TODO)
##### ros1 version
##### ros2 version

## Branches
1. main: tested and ready to run with compose-run.yaml
2. devel: use this to do experiment and test feature integration
3. ft.XXX: use this to test specific feature

## GitHub & Container Workspace Structure

    Eurobot-2025-Localization/
    ├── docker/
    │   ├── gui/
    │   │   ├── Dockerfile
    │   │   └── docker-compose.yaml
    │   ├── local/
    │   │   ├── Dockerfile
    │   │   └── docker-compose.yaml
    │   ├── Robert/
    │   │   └── docker-compose.yaml
    │   └── Robinson/
    │       └── docker-compose.yaml
    └── localization-ws/src/
        ├── sensor-ws                    --- only in container, built in image ---
        │   ├── phidgets_drivers/              
        │   ├── obstacle_detector              
        │   └── rplidar_ros_driver/            
        └── localization-devel-ws        --- mount start from here ---
            ├── usb.sh
            ├── local/                    ### 相對定位 (odom->base_footprint)
            │   ├── local_filter/              # IMU + Wheel Odom，發 topic & TF
            │   │   └── src/
            │   ├── imu/
            │   │   └── imu_drive/
            │   └── wheel_odom/
            │       ├── odom_drive/            # covariance 處理
            │       └── communication/         # 與 STM32 的 microROS 通訊
            ├── global/                   ### 絕對定位
            │   ├── lidar_localization/        # FindBeacon, triangulation
            │   └── SearchBeacon/              # 急診室 (odom + aruco + lidar)
            │       └── src/
            ├── fusion/                   ### EKF
            │   └── src/
            └── rival/                    ### 敵機定位
                └── rival_localization/        # ArUco + LiDAR
                    └── src/
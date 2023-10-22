#include "fusion.h"
#include "my_camera.h"
#include "my_lidar.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include<iostream>
#include <thread>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fusion_of_camera_and_lidar_node");
    ros::NodeHandle n;

    fusion my_fusion(n);

std::cout<<"the lidar_pt in main="<<my_fusion.car_lidar.get_cloud_ptr()<<std::endl;         

// return 0;
    std::thread subscribe_thread(&fusion::test_spin, &my_fusion);
    std::thread publish_thread(&fusion::publish_thread, &my_fusion);
    // my_fusion.test_spin();
    subscribe_thread.join();
    publish_thread.join();

    // ros::spin();
    return 0;
}
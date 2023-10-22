#pragma once

#include "my_config.h"
#include "my_lidar.h"
#include "my_camera.h"

#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h> /* /usr/include/pcl-1.7/pcl */ /* 依赖/usr/include/eigen3/Eigen/ */
#include <pcl/point_types.h>

class fusion
{
private:
    my_config fusion_config;

    std::vector<cv::Point3f> points3d; //用OpenCV存储点云

    std::vector<cv::Point2f> projectedPoints; //存储投影后的二维点云，放到图片中的点云信息
    std::vector<cv::Scalar> dis_color;        //用于存储点云的颜色信息（根据距离赋值）

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud; //存储坐标变换后的点云（原本坐标是相对于雷达本身坐标系的，变换后是相对于相机的也就是图片中显示的点对应的三维坐标位置）

    cv::Mat fused_image; //存储融合后的图像

    ros::NodeHandle n;

    sensor_msgs::ImagePtr fused_image_msg; //融合后的图像消息

    ros::Publisher fused_image_pub; //用于发布图像的节点

    ros::Subscriber lidar_sub;
    ros::Subscriber camera_sub;

    //存储颜色信息
    int color[21][3]; //= {{255, 0, 0}, {255, 69, 0}, {255, 99, 71}, {255, 140, 0}, {255, 165, 0}, {238, 173, 14}, {255, 193, 37}, {255, 255, 0}, {255, 236, 139}, {202, 255, 112}, {0, 255, 0}, {84, 255, 159}, {127, 255, 212}, {0, 229, 238}, {152, 245, 255}, {178, 223, 238}, {126, 192, 238}, {28, 134, 238}, {0, 0, 255}, {72, 118, 255}, {122, 103, 238}};
    float color_dis;  // = 1.2;
    bool init_Points3f();       //将ca_lidar中的pcl点云转换为points3d
    void init_color();     //根据点云的z坐标生成每个点云的rgb
    bool point_to_image(); //将点云在图像中圈出来
    // pcl::transformPointCloud(*input_cloud_ptr, *transformed_cloud, transform); // lidar coordinate(forward x+, left y+, up z+)

public:
    my_lidar car_lidar;
    my_camera car_camera;
    fusion(ros::NodeHandle n);
    bool projection();          //生成融合图
    void publish_fused_image(); //发布融合图消息
    void publish_thread();
    void show_fused_image();    //并用OpenCV窗口显示融合图
    void test_spin();
    ~fusion();
};

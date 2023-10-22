#pragma once

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h> /* /opt/ros/kinetic/include/cv_bridge */

#include <opencv2/opencv.hpp>

class my_camera
{
    //实现相机回调函数
private:
    cv_bridge::CvImagePtr cv_image_ptr; //存储转换后的相机图像数据
    // cv_bridge::Cv
    bool called; //判断是否被调用过回调函数
    // cv::Mat image;

public:
    my_camera(); //初始化相机图像
    bool iscalled();
    cv_bridge::CvImagePtr get_image_ptr();
    void cameraCallback(const sensor_msgs::ImageConstPtr &);
    ~my_camera();
};
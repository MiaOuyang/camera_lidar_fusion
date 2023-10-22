#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <string>
class my_config
{
    //读取配置文件，并配置投影所需要的各种矩阵
private:
    std::string file_path;
    //下面存储投影所需矩阵，以供使用

public:
    cv::Mat extrinsic_mat, camera_mat, dist_coeff;
    cv::Mat rotate_mat, transform_vec;

    Eigen::Matrix4d transform, inv_transform; //用于存储雷达点云坐标变换矩阵，这里因为那个坐标变换的函数没有Matrix4d的重载，就用4f吧，应该差不多

    my_config();            //初始化配置文件路径，获取当前位置下的配置文件
    my_config(std::string); //获取指定位置下的配置文件
    void read_config();
    ~my_config();
};
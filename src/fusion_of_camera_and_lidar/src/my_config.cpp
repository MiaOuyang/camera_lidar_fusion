#include "my_config.h"

#include <string>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

my_config::my_config()
{
    this->file_path = "./src/fusion_of_camera_and_lidar/config/calibration_out.yml"; //默认是在ros_ws下运行程序，那么配置文件的相对路径就是这样的
    ROS_INFO("my_config inited!");
}
my_config::my_config(std::string file_path)
{
    this->file_path = file_path;
}
void my_config::read_config()
{
    cv::FileStorage fs_read(this->file_path, cv::FileStorage::READ);

    ROS_INFO("open .yml file!");
    fs_read["CameraExtrinsicMat"] >> this->extrinsic_mat;
    fs_read["CameraMat"] >> this->camera_mat;
    fs_read["DistCoeff"] >> this->dist_coeff;
    fs_read.release();
    ROS_INFO("close .yml file!");

    this->rotate_mat = cv::Mat(3, 3, cv::DataType<double>::type);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            this->rotate_mat.at<double>(i, j) = this->extrinsic_mat.at<double>(j, i);
        }
    }
    this->transform_vec = cv::Mat(3, 1, cv::DataType<double>::type);
    this->transform_vec.at<double>(0) = this->extrinsic_mat.at<double>(1, 3);
    this->transform_vec.at<double>(1) = this->extrinsic_mat.at<double>(2, 3);
    this->transform_vec.at<double>(2) = this->extrinsic_mat.at<double>(0, 3);
    this->transform(0, 0) = this->extrinsic_mat.at<double>(0, 0);
    this->transform(0, 1) = this->extrinsic_mat.at<double>(0, 1);
    this->transform(0, 2) = this->extrinsic_mat.at<double>(0, 2);
    this->transform(0, 3) = this->extrinsic_mat.at<double>(0, 3);
    this->transform(1, 0) = this->extrinsic_mat.at<double>(1, 0);
    this->transform(1, 1) = this->extrinsic_mat.at<double>(1, 1);
    this->transform(1, 2) = this->extrinsic_mat.at<double>(1, 2);
    this->transform(1, 3) = this->extrinsic_mat.at<double>(1, 3);
    this->transform(2, 0) = this->extrinsic_mat.at<double>(2, 0);
    this->transform(2, 1) = this->extrinsic_mat.at<double>(2, 1);
    this->transform(2, 2) = this->extrinsic_mat.at<double>(2, 2);
    this->transform(2, 3) = this->extrinsic_mat.at<double>(2, 3);
    this->transform(3, 0) = this->extrinsic_mat.at<double>(3, 0);
    this->transform(3, 1) = this->extrinsic_mat.at<double>(3, 1);
    this->transform(3, 2) = this->extrinsic_mat.at<double>(3, 2);
    this->transform(3, 3) = this->extrinsic_mat.at<double>(3, 3);
    this->inv_transform = this->transform.inverse();
    ROS_INFO("the matrixs generated!");
}

my_config::~my_config()
{
    ROS_INFO("config completed!");
}
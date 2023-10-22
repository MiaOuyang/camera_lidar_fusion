#include "fusion.h"
#include "my_config.h"
#include "my_lidar.h"
#include "my_camera.h"

#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h> /* /usr/include/pcl-1.7/pcl */ /* 依赖/usr/include/eigen3/Eigen/ */
#include <pcl/point_types.h>
#include <pcl/common/transforms.h> //用于pcl点云坐标变换

fusion::fusion(ros::NodeHandle n)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    this->transformed_cloud = temp_ptr; //指针要初始化
    this->n = n;
    this->fused_image_pub = n.advertise<sensor_msgs::Image>("/fusion_image", 100);
    this->lidar_sub = n.subscribe("/rslidar_points", 10, &my_lidar::lidarCallback, &(this->car_lidar));
    this->camera_sub = n.subscribe("/camera/color/image_raw", 10, &my_camera::cameraCallback, &(this->car_camera));
    int temp_color[21][3] = {{255, 0, 0}, {255, 69, 0}, {255, 99, 71}, {255, 140, 0}, {255, 165, 0}, {238, 173, 14}, {255, 193, 37}, {255, 255, 0}, {255, 236, 139}, {202, 255, 112}, {0, 255, 0}, {84, 255, 159}, {127, 255, 212}, {0, 229, 238}, {152, 245, 255}, {178, 223, 238}, {126, 192, 238}, {28, 134, 238}, {0, 0, 255}, {72, 118, 255}, {122, 103, 238}};
    // this->color = temp_color;
    std::copy(&temp_color[0][0], &temp_color[0][0] + 21 * 3, &this->color[0][0]);
    this->color_dis = 1.2;
    this->fusion_config.read_config(); //读取配置文件，初始化投影以及坐标变换必要矩阵
    ROS_INFO("fusion inited");
}
bool fusion::init_Points3f()
{
    //将点云插入到OpenCV的三维点数据中

    if (!this->car_lidar.iscalled())
    {
        return false;
    }
    this->points3d.clear();                                              //清空点云内容
    this->points3d.reserve(this->car_lidar.get_cloud_ptr()->size() + 1); //重新分配空间
    cv::Point3f point;
    for (size_t i = 0; i < this->car_lidar.get_cloud_ptr()->size(); i++)
    {
        point.x = this->car_lidar.get_cloud_ptr()->points[i].x;
        point.y = this->car_lidar.get_cloud_ptr()->points[i].y;
        point.z = this->car_lidar.get_cloud_ptr()->points[i].z;
        points3d.push_back(point); //将点云插入到OpenCV的三维点数据中
    }
    return true;
}

void fusion::init_color()
{
    //获取每个点云在相机坐标系的坐标，放到整个逻辑中，
    // pcl::transformPointCloud(*(this->car_lidar.get_cloud_ptr()), *(this->transformed_cloud), this->fusion_config.transform); // lidar coordinate(forward x+, left y+, up z+)
    // camera coordiante(right x+, down y+, forward z+) (3D-3D)
    // using the extrinsic matrix between this two coordinate system

    //这段初始化颜色信息的前提是获得了激光雷达点云坐标变换后的点
    this->dis_color.clear(); //给存颜色信息的vector重置
    this->dis_color.reserve(this->car_lidar.get_cloud_ptr()->size() + 1);
    for (size_t i = 0; i < this->transformed_cloud->points.size(); i++)
    {
        if (this->transformed_cloud->points[i].z > 0) //渲染显示照片正面的点云
        {
            int color_order = int(this->transformed_cloud->points[i].z / this->color_dis);
            if (color_order > 20)
            {
                color_order = 20;
            }
            this->dis_color.push_back(cv::Scalar(this->color[color_order][2], this->color[color_order][1], this->color[color_order][0]));
        }
    } //存储点云的颜色信息
    return;
}

bool fusion::point_to_image()
{
    if (!this->car_camera.iscalled())
    {
        return false;
    }
    //在圈出来之前，融合图应该初始化为相机图像。
    //并且在圈出来之前，点云应当投影到相机图像上（生成二维点云）
    for (size_t i = 0; i < this->projectedPoints.size(); i++)
    {
        cv::Point2f p = this->projectedPoints[i];
        // ROS_INFO("start to circle color");
        if (p.y < 480 && p.y >= 0 && p.x < 640 && p.x >= 0 && this->transformed_cloud->points[i].z > 0)
        {
            //把在照片正面的，照片尺寸之内的点云在图像中圈出来。
            cv::circle(this->fused_image, p, 1, this->dis_color[i], 1, 8, 0);
        }
    }
    return true;
}

bool fusion::projection()
{
    //一步一步融合图像

    //应当从雷达中获取到了点云消息
    // 将点云消息转到用OpenCV存储的格式
    if (!this->init_Points3f())
    {
        ROS_INFO("no points topic!");
        return false;
    }

    //使用pcl库将原始的点云进行坐标变换，得到相机坐标系下的点云坐标
    ROS_INFO("start to transform cloud");
    pcl::transformPointCloud(*(this->car_lidar.get_cloud_ptr()), *(this->transformed_cloud), this->fusion_config.transform); // lidar coordinate(forward x+, left y+, up z+)

    //根据相机坐标系下的点云坐标初始化每个点应该渲染的颜色
    this->init_color();

    //初始化融合后的图像fused_image为原始的相机图片
    this->fused_image = this->car_camera.get_image_ptr()->image;

    //在画圈之前必须要先把点云投影到二维，这样才知道该在哪里画圈
    this->projectedPoints.clear(); //清空之前的点
    cv::projectPoints(this->points3d, this->fusion_config.rotate_mat, this->fusion_config.transform_vec, this->fusion_config.camera_mat, this->fusion_config.dist_coeff, this->projectedPoints);

    //根据前面给每个点赋予的颜色对投影到相机图片上的点进行渲染（通过画圈的方式）
    if (!this->point_to_image())
    {
        ROS_INFO("No image topic found! waiting subscribe");
        return false;
    }

    //至此，图片融合数据处理部分已经完成！
    ROS_INFO("fused image successfully!");
    return true;
}

void fusion::publish_fused_image()
{
    //把融合图像发布出去
    this->fused_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->fused_image).toImageMsg();
    this->fused_image_pub.publish(this->fused_image_msg);
    ROS_INFO("fusion image published!");
    return;
}

void fusion::show_fused_image()
{
    ROS_INFO("To be continued!");
    return;
}

void fusion::publish_thread()
{
    static ros::Rate rate(10);
    while (ros::ok())
    {
        if (!this->projection())
        {
            continue;
        }
        ROS_INFO("publish fused image!");
        this->publish_fused_image();
        rate.sleep();
    }
    return;
}

void fusion::test_spin()
{
    ROS_INFO("start spin");
    ros::spin();
    ROS_INFO("end spin");
}

fusion::~fusion()
{
    ROS_INFO("fusion deleted!");
}
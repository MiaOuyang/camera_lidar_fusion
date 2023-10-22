#pragma once

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h> /* /usr/include/pcl-1.7/pcl */ /* 依赖/usr/include/eigen3/Eigen/ */
#include <pcl/point_types.h>                                //提供各种点云数据类型

class my_lidar
{
    //实现雷达的回调函数
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_cloud_ptr; //存储转换后的雷达点云
    bool called;

public:

    my_lidar(); //初始化获取pcl点云指针
    bool iscalled();
    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &);
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_cloud_ptr();
    ~my_lidar();
};

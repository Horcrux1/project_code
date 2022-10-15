// Copyright: Pouyan B. Navard @ PCVLab @ OSU 

#include <string>
#include <iostream>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>


class PointCloudRegistration{
    public:
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_point_cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_point_cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_point_cloud;
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    public:
        void setSourcePointCloud(std::string pc_path);
        void setTargetPointCloud(std::string pc_path);
        bool initICP();
        bool alignPointClouds();
        void saveAlignedPointCloud(std::string save_path);
    private: 
        void readPointCloud(std::string pc_path, pcl::PointCloud<pcl::PointXYZRGB>& point_cloud);
    public:
        PointCloudRegistration()
        :source_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
        target_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
        aligned_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
        {}
};

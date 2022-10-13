#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>


class VinsMonoParser{
    public:
        VinsMonoParser();
        void loadPointCloudBag(std::string pc_bag_path);
        void loadOdometryBag(std::string odo_bag_path);
        void parsePointCloud();
        void parseTrajectory();
        bool savePointCloud(std::string path);
    private:
        void moveFromRosMsgToPclPointCloud2(sensor_msgs::PointCloud&, pcl::PointCloud<pcl::PointXYZ>&);
    public: 
        rosbag::Bag point_cloud_bag;
        rosbag::Bag trajectory_bag;
        // container to store pointcloud messages 
        std::vector<pcl::PointCloud<pcl::PointXYZ>> point_cloud_messages;
};

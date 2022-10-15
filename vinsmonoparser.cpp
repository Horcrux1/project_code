// Copyright: Pouyan B. Navard @ PCLab @ OSU 

#include "vinsmonoparser.h"

#define LOG(X) std::cout << X << std::endl

VinsMonoParser::VinsMonoParser(){
    LOG("Vins Mono Parser takes care of parsing rosbag files generated from VINS-Mono software!");
}

void VinsMonoParser::loadPointCloudBag(std::string pc_bag_path){
    if(!boost::filesystem::exists(pc_bag_path)){
        LOG("Could not find the point cloud!");
    }
    else{
        // open the point cloud bagfile 
        try{
            point_cloud_bag.open(pc_bag_path);
        }
        catch(rosbag::BagException){
            LOG("Error opening the bag file!");
        }
    }
}

void VinsMonoParser::loadOdometryBag(std::string odo_bag_path){
    if(!boost::filesystem::exists(odo_bag_path)){
        LOG("Could not find the point cloud!");
    }
    else{
        // open the point cloud bagfile 
        try{
            trajectory_bag.open(odo_bag_path);
        }
        catch(rosbag::BagException){
            LOG("Error opening the bag file!");
        }
    }
}

void VinsMonoParser::moveFromRosMsgToPclPointCloud2(sensor_msgs::PointCloud& ros_pc_msg, pcl::PointCloud<pcl::PointXYZ>& point_cloud){\
    /*
        convert ROS pointcloud message to PCL standard template point cloud. It migrates ROS pointcloud1 message to pointcloud2 and then to PCL standard template pointcloud.
    */

    // converting PointCloud message to PointCloud2 message which is compatible with PCL library!
    sensor_msgs::PointCloud2 ros_pc2_msg; 
    sensor_msgs::convertPointCloudToPointCloud2(ros_pc_msg, ros_pc2_msg);
    // move from ROS message to PCL template point cloud 
    pcl::moveFromROSMsg(ros_pc2_msg, point_cloud);
}

void VinsMonoParser::parsePointCloud(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // iterate through point cloud messages generated from frames
    for(rosbag::MessageInstance const m: rosbag::View(point_cloud_bag)){
        sensor_msgs::PointCloudPtr ros_pc1_msg =  m.instantiate<sensor_msgs::PointCloud>();
        if(ros_pc1_msg != NULL) {
            moveFromRosMsgToPclPointCloud2(*ros_pc1_msg, *point_cloud);
            if((*point_cloud).size() != 0)
                point_cloud_messages.push_back(*point_cloud);
            else
                LOG((boost::format("sequence-%1% of the point cloud message was empty!") % (*point_cloud).header.seq));
        }
    }
}

bool VinsMonoParser::savePointCloud(std::string path){
    // check if the directory exists 
    if(!boost::filesystem::exists(path)){
        LOG("Could not save the point cloud! Directory does not exist!");
        return false;
    }
    else {
        LOG(boost::format("Saving pointcloud to %1%") % path); 
        // save point cloud to the disk with label    
        boost::format fmtr(path + "/pointcloud-%1%.pcd");
        for(pcl::PointCloud<pcl::PointXYZ> pc: point_cloud_messages){
            fmtr % pc.header.seq;
            pcl::io::savePCDFileASCII(fmtr.str(), pc);
        }
        LOG("Done!");
        return true;
    }
}

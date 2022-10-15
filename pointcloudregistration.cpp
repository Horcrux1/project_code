// Copyright: Pouyan B. Navard @ PCVLab @ OSU 

#include "pointcloudregistration.h"

#define LOG(X) std::cout << X << std::endl

void PointCloudRegistration::setSourcePointCloud(std::string pc_path){
    readPointCloud(pc_path, *source_point_cloud);
}

void PointCloudRegistration::setTargetPointCloud(std::string pc_path){
    readPointCloud(pc_path, *target_point_cloud);
}

void PointCloudRegistration::readPointCloud(std::string pc_path, pcl::PointCloud<pcl::PointXYZRGB>& point_cloud){
    if(!boost::filesystem::exists(pc_path)){
        LOG("Could not find the point cloud!");
    }
    else{
        // get extension of the point cloud 
        std::string pc_extension = boost::filesystem::extension(pc_path);
        if(pc_extension == ".pcd"){
            pcl::io::loadPCDFile<pcl::PointXYZRGB>(pc_path, point_cloud);
        } else if(pc_extension == ".ply"){
            pcl::PLYReader plyreader;
            plyreader.read(pc_path, point_cloud);
        }
        else{
            LOG("Cannot read the point cloud! Format not supported!");
        }
    }
}

bool PointCloudRegistration::initICP(){
    if ((*source_point_cloud).size() == 0){
        LOG("source point cloud was empty! Use PointCloudRegistration::setSourcePointCloud() to initialize it!");
        return false;
    } else if ((*target_point_cloud).size() == 0){
        LOG("traget point cloud was empty! Use PointCloudRegistration::setTargetPointCloud() to initialize it!");
        return false;
    }else{
        LOG("initializing ICP registration variables!");        
        icp.setInputSource(source_point_cloud);
        icp.setInputTarget(target_point_cloud);
        // Set the max correspondence distance to 5cm 
        icp.setMaxCorrespondenceDistance (0.05);
        // Set the maximum number of iterations (criterion 1)
        icp.setMaximumIterations (50);
        // Set the transformation epsilon (criterion 2)
        icp.setTransformationEpsilon (1e-8);
        // Set the euclidean distance difference epsilon (criterion 3)
        icp.setEuclideanFitnessEpsilon (1);
        LOG("Done!");
        return true;
    }
}

bool PointCloudRegistration::alignPointClouds(){
    LOG("aligning the point clouds!");
    icp.align(*aligned_point_cloud);
    if(icp.hasConverged()){
        LOG("Registration converged successfuly!");
        LOG(boost::format("Fitness score: %1%") % icp.getFitnessScore());
        return true;
    } else {
        LOG("Convergence failed!");
        return false;
    }
}

void PointCloudRegistration::saveAlignedPointCloud(std::string save_path){

    LOG(boost::format("Saving the aligned pointcloud to %1%") % save_path); 
    // get extension of the point cloud 
    std::string pc_extension = boost::filesystem::extension(save_path);
    if(save_path == ".pcd"){
        // save point cloud to the disk with label    
        pcl::io::savePCDFileASCII(save_path, *aligned_point_cloud);
        LOG("Done!");
    } else if(save_path == ".ply"){
        pcl::PLYWriter plywriter;
        plywriter.write(save_path, *aligned_point_cloud);
    }
    else{
        LOG("Cannot save the registered point cloud! Format not supported!");
    }

}

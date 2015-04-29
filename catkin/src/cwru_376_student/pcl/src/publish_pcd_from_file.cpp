// process_pcd_dev.cpp: wsn, April, 2015
// example code to acquire a pointcloud from disk, then perform various processing steps interactively.
// processing is invoked by point-cloud selections in rviz, as well as "mode" settings via a service
// e.g.:  rosservice call process_mode 0 induces processing in mode zero (plane fitting)
// adding more code for cylinder registration


#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> 
#include <geometry_msgs/PointStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/features/normal_3d.h>

#include <cwru_srv/simple_int_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>

//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud; // can use this for short-hand

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace pcl::io;

//pcl::PointCloud<pcl::PointXYZ>::Ptr g_pclKinect(new PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud_from_disk(new pcl::PointCloud<pcl::PointXYZ>); //this one is read from PCD file on disk

//more globals--to share info on planes and patches
Eigen::Vector4f g_plane_params;
Eigen::Vector3f g_patch_centroid;
Eigen::Vector3f g_plane_normal;
Eigen::Vector3f g_plane_origin;
Eigen::Affine3f g_A_plane;
double g_z_plane_nom;
std::vector<int> g_indices_of_plane; //indices of patch that do not contain outliers 


int main(int argc, char** argv) {
    // Do some initialization here
    ros::init(argc, argv, "publish_pcd");
    ros::NodeHandle nh;
    ros::Rate rate(2);
    ros::Publisher pubPcdCloud = nh.advertise<sensor_msgs::PointCloud2> ("/kinect/depth/points", 1);

    //load a pointcloud from file: 
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *g_cloud_from_disk) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
            << g_cloud_from_disk->width * g_cloud_from_disk->height
            << " data points from test_pcd.pcd  " << std::endl;

    g_cloud_from_disk->header.frame_id = "world"; //looks like PCD does not encode the reference frame id
 
    while (ros::ok()) {
        pubPcdCloud.publish(g_cloud_from_disk); //keep displaying the original scene

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
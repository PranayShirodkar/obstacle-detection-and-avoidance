#include <ros/ros.h>
// PCL specific includes
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher voxel;
ros::Publisher planeinliers;
ros::Publisher planeoutliers;
ros::Publisher cloudinliers;
ros::Publisher cloudoutliers;

using namespace sensor_msgs;

void cloud_cb(const sensor_msgs::PointCloud2Ptr& cloud_msg)
{
    //formatting the input
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr initial_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Zlimited_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);//Zlimited and voxel done
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_inliers(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_outliers(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outliers(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::fromROSMsg(*cloud_msg, *initial_cloud);

    //PROCESSING (Limit Z, Voxel, RANSAC, Extract inliers & outliers)

    // Limit Z
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(initial_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 25.0);
    //pass.setFilterLimitsNegative(true);
    pass.filter(*Zlimited_cloud);

    // Voxelgrid
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(Zlimited_cloud);
    vox.setLeafSize(0.01f, 0.01f, 0.01f); // 0.01f // 0.5f
    // 0.01f = 1cm is the distance between voxelgrid points. Setting to 0.1 decreases the number of points. Setting to 0.001 increases the number of points
    vox.filter(*cloud);

    // RANSAC for a plane
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;  // Create the segmentation object
    seg.setOptimizeCoefficients(true);  // Optional
    seg.setModelType(pcl::SACMODEL_PLANE);  // Mandatory
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.08f); // 0.08f // 1.0f //all point cloud points within this distance from the modelled plane are considered inliers
    seg.setMaxIterations(300); //number of times RANSAC iterates. Higher -> more accurate plane model coefficients, but more processing time

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
    }
    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
    << coefficients->values[1] << " "
    << coefficients->values[2] << " " 
    << coefficients->values[3] << std::endl;
    std::cerr << "Total points: " << cloud->points.size() << std::endl;
    std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;

    // Extract inliers
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);           // Extract the inliers
    extract.filter(*plane_inliers);       // cloud_inliers contains the plane

    // Extract outliers
    //extract.setInputCloud(cloud);       // Already done
    //extract.setIndices(inliers);            // Already done
    extract.setNegative(true);                // Extract the outliers
    extract.filter(*plane_outliers);      // cloud_outliers contains everything but the plane

    // StatisticalOutlierRemoval from plane_outliers
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(plane_outliers);
    sor.setMeanK(100); // 50
    sor.setStddevMulThresh(1.0);
    sor.setNegative(false);
    sor.filter(*cloud_inliers);

    sor.setNegative(true);
    sor.filter(*cloud_outliers);

    //formatting for output
    sensor_msgs::PointCloud2 cloud_output;
    sensor_msgs::PointCloud2 plane_inliers_output;
    sensor_msgs::PointCloud2 plane_outliers_output;
    sensor_msgs::PointCloud2 cloud_inliers_output;
    sensor_msgs::PointCloud2 cloud_outliers_output;

    pcl::toROSMsg(*cloud, cloud_output);
    pcl::toROSMsg(*plane_inliers, plane_inliers_output);
    pcl::toROSMsg(*plane_outliers, plane_outliers_output);
    pcl::toROSMsg(*cloud_inliers, cloud_inliers_output);
    pcl::toROSMsg(*cloud_outliers, cloud_outliers_output);

    voxel.publish(cloud_output);
    planeinliers.publish(plane_inliers_output);
    planeoutliers.publish(plane_outliers_output);
    cloudinliers.publish(cloud_inliers_output);
    cloudoutliers.publish(cloud_outliers_output);
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pointcloudprocessing");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("stereo/points2", 1, cloud_cb);

    // Create a ROS publisher for the output model coefficients
    voxel = nh.advertise<sensor_msgs::PointCloud2>("voxel_output", 1);
    planeinliers = nh.advertise<sensor_msgs::PointCloud2>("plane_inliers_output", 1);
    planeoutliers = nh.advertise<sensor_msgs::PointCloud2>("plane_outliers_output", 1);
    cloudinliers = nh.advertise<sensor_msgs::PointCloud2>("cloud_inliers_output", 1);
    cloudoutliers = nh.advertise<sensor_msgs::PointCloud2>("cloud_outliers_output", 1);
    // Spin
    ros::spin();
}
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

using namespace sensor_msgs;

void cloud_cb(const sensor_msgs::PointCloud2Ptr& cloud_msg)
{
    sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);

    sensor_msgs::PointCloud2Iterator<float> iter_xi(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_yi(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_zi(*cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_ri(*cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_gi(*cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_bi(*cloud_msg, "b");

    //project onto 2D map the y = 1 plane, convert to white
    //pcd_modifer.size() = 786432 = 1024x768
    for (size_t i = 0; i < pcd_modifier.size(); ++i, ++iter_xi, ++iter_yi, ++iter_zi, ++iter_ri, ++iter_gi, ++iter_bi)
    {
        if ((!std::isnan(*iter_xi)) || (!std::isnan(*iter_yi)) || (!std::isnan(*iter_zi)))
        {
        *iter_yi = 1;
        *iter_ri = 255;
        *iter_gi = 255;
        *iter_bi = 255;
        }
    }
    pub.publish(cloud_msg);
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "obstaclemapnode");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("cloud_inliers_output", 1, cloud_cb);
    // Create a ROS publisher for the output model coefficients
    pub = nh.advertise<sensor_msgs::PointCloud2>("obstaclemap", 1);
    // Spin
    ros::spin();
}

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <tf/transform_listener.h>

ros::Publisher octomap_pub;
tf::TransformListener* tf_listener;

void sensorDataCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
 {
    // Lookup the transform between the sensor frame and the global map frame
    tf::StampedTransform transform;
    try {
        tf_listener->waitForTransform("map", cloud_msg->header.frame_id, cloud_msg->header.stamp, ros::Duration(1.0));
        tf_listener->lookupTransform("map", cloud_msg->header.frame_id, cloud_msg->header.stamp, transform);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return;
    }

    // Transform the sensor data (point cloud) to the global map frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);
    pcl_ros::transformPointCloud(*pcl_cloud, *pcl_cloud, transform);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "transform");
    ros::NodeHandle nh;

    tf_listener = new tf::TransformListener();

    ros::Subscriber sensor_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, sensorDataCallback);
    octomap_pub = nh.advertise<octomap_msgs::Octomap>("/octomap", 1);

    ros::spin();

    delete tf_listener;
    return 0;
}


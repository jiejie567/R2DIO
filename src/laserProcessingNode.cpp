// Author of SSL_SLAM3: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "utils.h"
#include "param.h"
#include "laserProcessingClass.h"

LaserProcessingClass laserProcessing;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;

ros::Publisher pubLinePoints;
ros::Publisher pubPlanePoints;

ros::Publisher pubLaserCloudFiltered;

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();

}
void RGBDHandler(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{

    //read data
//    laserProcessing.frame_count++;
//    if(laserProcessing.frame_count%3!=0)
//        return;
    cv_bridge::CvImagePtr color_ptr, depth_ptr;
    cv::Mat color_pic, depth_pic;
    color_ptr = cv_bridge::toCvCopy(msgRGB, sensor_msgs::image_encodings::BGR8);
    color_pic = color_ptr->image;
    depth_ptr = cv_bridge::toCvCopy(msgD, sensor_msgs::image_encodings::TYPE_32FC1);
    depth_pic = depth_ptr->image;

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pointcloud_line(new pcl::PointCloud<pcl::PointXYZRGBL>());
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGBL>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter ( new pcl::PointCloud<pcl::PointXYZRGB> ());
    static TicToc timer("laser processing");
    timer.tic();
    laserProcessing.featureExtraction(color_pic, depth_pic, pointcloud_line, cloud_plane, cloud_filter);
    timer.toc(100);

    ros::Time pointcloud_time = msgRGB->header.stamp;
    sensor_msgs::PointCloud2 laserCloudFilteredMsg;
    pcl::toROSMsg(*cloud_filter, laserCloudFilteredMsg);
    laserCloudFilteredMsg.header.stamp = pointcloud_time;
    laserCloudFilteredMsg.header.frame_id = "camera_depth_optical_frame";
    pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

    sensor_msgs::PointCloud2 LinePointsMsg;
    pcl::toROSMsg(*pointcloud_line, LinePointsMsg);
    LinePointsMsg.header.stamp = pointcloud_time;
    LinePointsMsg.header.frame_id = "camera_depth_optical_frame";
    pubLinePoints.publish(LinePointsMsg);

    sensor_msgs::PointCloud2 PlanePointsMsg;
    pcl::toROSMsg(*cloud_plane, PlanePointsMsg);
    PlanePointsMsg.header.stamp = pointcloud_time;
    PlanePointsMsg.header.frame_id = "camera_depth_optical_frame";
    pubPlanePoints.publish(PlanePointsMsg);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    std::string file_path;
    nh.getParam("/file_path", file_path);
    laserProcessing.init(file_path);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, laserProcessing.lidar_param.rgb_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, laserProcessing.lidar_param.depth_topic, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&RGBDHandler, _1, _2));

    pubLaserCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_filtered", 100);
    pubLinePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_line", 100);
    pubPlanePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_plane", 100);

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

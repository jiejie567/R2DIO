//
// Created by frog on 2022/12/7.
//
//
// Created by ywl on 22-11-20.
// 记录保存mct_loam轨迹
// rosrun r2dio trajectory_listener
// rosservice call /recordPath "{}"
// 记录保存其他轨迹
// rosrun r2dio trajectory_listener /topicName
// rosservice call /recordPath "{}"

#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <queue>
#include <std_srvs/Trigger.h>
#include <nav_msgs/Odometry.h>


std::queue<nav_msgs::PathConstPtr> qPath;
std::fstream f;

void addToQueueCallback(const nav_msgs::PathConstPtr & msg)
{
    qPath.push(msg);
    if (qPath.size()>100)
        qPath.pop();
}


bool recordPathCallback(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response &res)
{
    nav_msgs::PathConstPtr path = qPath.back();
    for (auto &pose: path->poses)
    {
        geometry_msgs::Point position;
        position = pose.pose.position;
        geometry_msgs::Quaternion orientation;
        orientation = pose.pose.orientation;
        double time = pose.header.stamp.toSec();
        f << time << " " << position.x << " " << position.y << " " << position.z << " " << orientation.x <<
          " " << orientation.y << " " << orientation.z << " " << orientation.w << std::endl;
    }
    f.close();
    res.success = true;
    res.message = "Finish";
    ROS_INFO("FINISH");
    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_listener");
    ros::NodeHandle nh;
    ros::Subscriber myListener;
    nh.setParam("/use_sim_time", true);

    f.open("trajectory.txt", std::ios::out);
    f.precision(9);
    f.setf(std::ios::fixed);

    std::string topicName;
    if (argc > 1)
        topicName = std::string(argv[1]);
    else
        topicName = "/path";

    if (topicName.empty())
    {
        ROS_ERROR("Given a wrong topic name");
        ros::shutdown();
        return -1;
    }

    ROS_INFO("Listening to %s", topicName.c_str());
    myListener = nh.subscribe<nav_msgs::Path>(topicName, 100, addToQueueCallback);
    ros::ServiceServer recordPathService = nh.advertiseService("/recordPath", recordPathCallback);

    ros::spin();
}
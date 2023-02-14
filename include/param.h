// Author of SSL_SLAM3: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _PARAM_H_
#define _PARAM_H_

// opencv
#include <opencv2/opencv.hpp>

// eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
// define lidar parameter
class CommonParam{
    public:
        CommonParam(){};
        void loadParam(std::string& path);
        int getCoreNum(void);
        int getInitFrame(void);
        int getNearbyFrame(void);
        Eigen::Isometry3d getTbl(void);
    private:
        int core_num;
        int init_frame;
        int nearby_frame;
        Eigen::Isometry3d Tbl;
};

// define lidar parameter
class LidarParam{
    public:
        LidarParam(){};
        void loadParam(std::string& path);
        double getMapResolution(void);
        Eigen::Matrix<double, 6, 1> getOdomN(void);
        double getEdgeN();
        double getPlaneN();
        double getLocalMapSize(void);
        double getLocalMapResolution(void);
        double getMapCellWidth(void);
        double getMapCellHeight(void);
        double getMapCellDepth(void);
        int getMapCellWidthRange(void);
        int getMapCellHeightRange(void);
        int getMapCellDepthRange(void);
        string rgb_topic;
        string depth_topic;
        double camera_factor;
        double camera_cx;
        double camera_cy;
        double camera_fx;
        double camera_fy;
        double min_distance;
        double max_distance;
        int gap_plane;
        int gap_line;
    private:
        int frequency;
        double horizontal_angle;
        double map_resolution;
        Eigen::Matrix<double, 6, 1> odom_n;
        double edge_n;
        double plane_n;
        double local_map_resolution;
        double local_map_size;
        double map_cell_width;
        double map_cell_height;
        double map_cell_depth;
        int map_cell_width_range;
        int map_cell_height_range;
        int map_cell_depth_range;
};

class ImuParam{
    public:
        ImuParam(){};
        void loadParam(std::string& path);
        int getFrequency(void);
        double getAccN(void);
        double getGyrN(void);
        double getAccW(void);
        double getGyrW(void);
        string imu_topic;
private:
        int frequency;
        double acc_n;
        double gyr_n;
        double acc_w;
        double gyr_w;
};


#endif // _PARAM_H_


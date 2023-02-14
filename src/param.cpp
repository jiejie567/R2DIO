// Author of SSL_SLAM3: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "param.h"

std::string readString(cv::FileNode& file_node, const std::string& name, const std::string& default_value){
    cv::FileNode node = file_node[name];
    if(node.empty())
        std::cerr << name << " required parameter does not exist, using default value"<< default_value << std::endl;
    else if(!node.isString())
        std::cerr << name << " parameter must be a string, using default value"<< default_value << std::endl;
    else
        return node.string();
    return default_value;
}
int readInt(cv::FileNode& file_node, const std::string& name, const int& default_value){
    cv::FileNode node = file_node[name];
    if(node.empty())
        std::cerr << name << " required parameter does not exist, using default value"<< default_value << std::endl;
    else if(!node.isInt())
        std::cerr << name << " parameter must be an integer, using default value"<< default_value << std::endl;
    else
        return node.operator int();
    return default_value;
}
double readDouble(cv::FileNode& file_node, const std::string& name, const double& default_value){
    cv::FileNode node = file_node[name];
    if(node.empty())
        std::cerr << name << " required parameter does not exist, using default value"<< default_value << std::endl;
    else if(!node.isReal())
        std::cerr << name << " parameter must be a double, using default value"<< default_value << std::endl;
    else
        return node.real();
    return default_value;
}
Eigen::Isometry3d readMatrix(cv::FileNode& file_node, const std::string& name, const Eigen::Isometry3d& default_value){
    cv::FileNode node = file_node[name];
    if(node.empty())
        std::cerr << name << " required parameter does not exist, using default value: rotation["<< default_value.linear() <<"] translation"<< default_value.translation()<< std::endl;
    else{
        cv::Mat mat_in = node.mat();
        if(mat_in.empty() || mat_in.rows != 4 || mat_in.cols != 4)
            std::cerr << name << " required parameter does not exist, using default value: rotation["<< default_value.linear() <<"] translation"<< default_value.translation()<< std::endl;
        else{
            Eigen::Isometry3d mat_out = Eigen::Isometry3d::Identity();
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    mat_out.linear()(i,j) = mat_in.at<double>(i, j);
                }
            }
            mat_out.translation().x() = mat_in.at<double>(0, 3);
            mat_out.translation().y() = mat_in.at<double>(1, 3);
            mat_out.translation().z() = mat_in.at<double>(2, 3);
            return mat_out; 
        }
    }
    return default_value;
}

void CommonParam::loadParam(std::string& path){
    //init qr detection 
    cv::FileStorage fsSettings(path, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
        cerr << "ERROR: Wrong path to settings" << endl;
    cv::FileNode node = fsSettings["common"]; 
    core_num = readInt(node,"core_num",1);
    init_frame = readInt(node,"init_frame",20);
    nearby_frame = readInt(node,"nearby_frame",15);
    Tbl = readMatrix(node,"Tbl",Eigen::Isometry3d::Identity());
    fsSettings.release();
}
int CommonParam::getCoreNum(){
    return core_num;
}
int CommonParam::getInitFrame(){
    return init_frame;
}
int CommonParam::getNearbyFrame(){
    return nearby_frame;
}
Eigen::Isometry3d CommonParam::getTbl(){
    return Tbl;
}

void LidarParam::loadParam(std::string& path){
    //init qr detection 
    cv::FileStorage fsSettings(path, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
        cerr << "ERROR: Wrong path to settings" << endl;
    cv::FileNode node = fsSettings["lidar"];
    rgb_topic = readString(node,"rgb_topic","/camera/color/image_raw");
    depth_topic = readString(node,"depth_topic","/camera/aligned_depth_to_color/image_raw");
    camera_factor = readDouble(node,"camera_factor",1000);
    camera_cx = readDouble(node,"camera_cx",333.891);
    camera_cy = readDouble(node,"camera_cy",254.687);
    camera_fx = readDouble(node,"camera_fx",597.53);
    camera_fy = readDouble(node,"camera_fy",597.795);
    min_distance = readDouble(node,"min_distance",0.25);
    max_distance = readDouble(node,"max_distance",9.0);
    map_resolution = readDouble(node,"map_resolution",0.05);
    frequency = readInt(node,"frequency",30);

    double odom_n_rpy = readDouble(node,"odom_n_rpy",1e-3);
    double odom_n_xyz = readDouble(node,"odom_n_xyz",1e-3);
    for (int i = 0; i < 3; ++i){
        odom_n(i) = odom_n_rpy;
        odom_n(i+3) = odom_n_xyz;
    }
    edge_n = readDouble(node,"edge_n",1e-3);
    plane_n = readDouble(node, "plane_n", 1e-3);
    local_map_size = readDouble(node,"local_map_size",10.0);
    local_map_resolution = readDouble(node,"local_map_resolution",0.05);

    map_cell_width = readDouble(node,"map_cell_width",50.0);
    map_cell_height = readDouble(node,"map_cell_height",50.0);
    map_cell_depth = readDouble(node,"map_cell_depth",50.0);
    map_cell_width_range = readInt(node,"map_cell_width_range",2);
    map_cell_height_range = readInt(node,"map_cell_height_range",2);
    map_cell_depth_range = readInt(node,"map_cell_depth_range",2);

    gap_line = readInt(node,"gap_line",3);
    gap_plane = readInt(node,"gap_plane",20);

    fsSettings.release();
}

double LidarParam::getMapResolution(){
    return map_resolution;
}
Eigen::Matrix<double, 6, 1> LidarParam::getOdomN(){
    return odom_n;
}
double LidarParam::getEdgeN(){
    return edge_n;
}
double LidarParam::getPlaneN(){
    return plane_n;
}

double LidarParam::getLocalMapResolution(){
    return local_map_resolution;
}
double LidarParam::getLocalMapSize(){
    return local_map_size;
}
double LidarParam::getMapCellWidth(){
    return map_cell_width;
}
double LidarParam::getMapCellHeight(){
    return map_cell_height;
}
double LidarParam::getMapCellDepth(){
    return map_cell_depth;
}
int LidarParam::getMapCellWidthRange(){
    return map_cell_width_range;
}
int LidarParam::getMapCellHeightRange(){
    return map_cell_height_range;
}
int LidarParam::getMapCellDepthRange(){
    return map_cell_depth_range;
}

void ImuParam::loadParam(std::string& path){
    //init qr detection 
    cv::FileStorage fsSettings(path, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
        cerr << "ERROR: Wrong path to settings" << endl;
    cv::FileNode node = fsSettings["imu"]; 
    frequency = readInt(node,"frequency",200);
    double freq_sr = sqrt((double)frequency);
    imu_topic = readString(node,"imu_topic","/camera/imu");
    double acc_n_in = readDouble(node,"acc_n",1e-2);
    double gyr_n_in = readDouble(node,"gyr_n",1e-3);
    double acc_w_in = readDouble(node,"acc_w",1e-3);
    double gyr_w_in = readDouble(node,"gyr_w",1e-4);
    acc_n = acc_n_in * freq_sr;
    gyr_n = gyr_n_in * freq_sr;
    acc_w = acc_w_in / freq_sr;
    gyr_w = gyr_w_in / freq_sr;
    fsSettings.release();
}
int ImuParam::getFrequency(){
    return frequency;
}
double ImuParam::getAccN(){
    return acc_n;
}
double ImuParam::getGyrN(){
    return gyr_n;
}
double ImuParam::getAccW(){
    return acc_w;
}
double ImuParam::getGyrW(){
    return gyr_w;
}

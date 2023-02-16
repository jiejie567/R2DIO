// Author of SSL_SLAM3: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "odomEstimationClass.h"

OdomEstimationClass::OdomEstimationClass(){
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose_r_arr.reserve(1000);
    pose_t_arr.reserve(1000);
    pose_v_arr.reserve(1000);
    pose_b_a_arr.reserve(1000);
    pose_b_g_arr.reserve(1000);
    imu_preintegrator_arr.reserve(1000);

    pose_r_arr.push_back(Utils::RToso3(pose.linear()));
    pose_t_arr.push_back(pose.translation());
    pose_v_arr.push_back(Eigen::Vector3d::Zero());
    pose_b_a_arr.push_back(Eigen::Vector3d::Zero());
    pose_b_g_arr.push_back(Eigen::Vector3d::Zero());
    imu_preintegrator_arr.clear();
    is_initialized = false;

    edge_map = pcl::PointCloud<pcl::PointXYZRGBL>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBL>());
    plane_map = pcl::PointCloud<pcl::PointXYZRGBL>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBL>());
    current_edge_points = pcl::PointCloud<pcl::PointXYZRGBL>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBL>());
    current_edge_points->reserve(1000);
    current_plane_points = pcl::PointCloud<pcl::PointXYZRGBL>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBL>());
    current_plane_points->reserve(1000);

    edge_map->reserve(100000);
    plane_map->reserve(100000);


    current_plane_num=0;
    pm_plane_info = new std::map<int,Eigen::Vector4d>();
    pm_line_info = new std::map<int,Eigen::Vector4d>();

}

void OdomEstimationClass::init(std::string& file_path){
    common_param.loadParam(file_path);
    lidar_param.loadParam(file_path);
    imu_param.loadParam(file_path);

    if(common_param.getNearbyFrame()>POSE_BUFFER) std::cerr<<"please set POSE_BUFFER = common.nearby_frame! "<<std::endl;
    double map_resolution = lidar_param.getLocalMapResolution();
    //downsampling size
    edge_downsize_filter.setLeafSize(map_resolution, map_resolution, map_resolution);
    plane_downsize_filter.setLeafSize(map_resolution * 2, map_resolution * 2, map_resolution * 2);

}

void OdomEstimationClass::initMapWithPoints(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr edge_in,
        const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr plane_in){
    last_pose = Eigen::Isometry3d::Identity();
    addLidarFeature(edge_in, plane_in);
    *edge_map += *current_edge_points;
    *plane_map += *current_plane_points;
    edge_kd_tree.setInputCloud(edge_map);
    plane_kd_tree.setInputCloud(plane_map);

    for(const auto& it:m_current_plane_info)
        (*pm_plane_info)[it.first] = it.second;
    for(const auto& it:m_current_line_info)
        (*pm_line_info)[it.first] = it.second;
}

bool OdomEstimationClass::initialize(void){
    if(pose_r_arr.size()<common_param.getInitFrame())
        return is_initialized;
    const int start_id = pose_r_arr.size() - common_param.getInitFrame();
    const int end_id = pose_r_arr.size() - 1;

    Eigen::Vector3d acc_mean(0.0,0.0,0.0);
    Eigen::Vector3d gyr_mean(0.0,0.0,0.0);
    int acc_count =0;

    // mean and std of IMU acc
    for(int i=start_id; i<end_id; i++){
        int discarded_imu =0;
        std::vector<Eigen::Vector3d> acc_buf = imu_preintegrator_arr[i].getAcc();
        std::vector<Eigen::Vector3d> gyr_buf = imu_preintegrator_arr[i].getGyr();

        for (int j = 0; j < acc_buf.size(); j++){
            acc_mean+=acc_buf[j];
            gyr_mean+=gyr_buf[j];
            acc_count++;
        }
    }
    acc_mean = acc_mean / acc_count;
    gyr_mean = gyr_mean / acc_count;

    for(int i=start_id; i<end_id;i++){
        imu_preintegrator_arr[i].update(Eigen::Vector3d::Zero(),gyr_mean);
        lidar_odom_arr[i] = Eigen::Isometry3d::Identity();
    }

    if(fabs(Utils::gravity.norm() - acc_mean.norm())>0.02)
    {
        ROS_WARN("the gravity is wrong! measured gravity = %f", acc_mean.norm());
        ROS_WARN("Use the measured gravity temporarily");
        Utils::gravity = acc_mean;
    }
    else
        Utils::gravity = acc_mean;

    ROS_INFO("gravity= %f = %f,%f,%f",Utils::gravity.norm(), Utils::gravity.x(),Utils::gravity.y(),Utils::gravity.z());
    ROS_INFO("gyr bias %f, %f, %f",gyr_mean.x(),gyr_mean.y(),gyr_mean.z());
   
    is_initialized = true;
    return is_initialized;
}

void OdomEstimationClass::addImuPreintegration(std::vector<double> dt_arr, std::vector<Eigen::Vector3d> acc_arr, std::vector<Eigen::Vector3d> gyr_arr){
    ImuPreintegrationClass imu_preintegrator(pose_b_a_arr.back(),pose_b_g_arr.back(), imu_param.getAccN(), imu_param.getGyrN(), imu_param.getAccW(), imu_param.getGyrW());
    for (int i = 0; i < dt_arr.size(); ++i){
        imu_preintegrator.addImuData(dt_arr[i], acc_arr[i], gyr_arr[i]);
    }
    imu_preintegrator_arr.push_back(imu_preintegrator);

    //add pose states
    Eigen::Matrix3d last_R = Utils::so3ToR(pose_r_arr.back());
    if(is_initialized == true){
        pose_r_arr.push_back(Utils::RToso3(last_R * imu_preintegrator.delta_R));
        pose_t_arr.push_back(pose_t_arr.back() - 0.5 * Utils::gravity * imu_preintegrator.sum_dt * imu_preintegrator.sum_dt + pose_v_arr.back() * imu_preintegrator.sum_dt + last_R * imu_preintegrator.delta_p);
        pose_v_arr.push_back(pose_v_arr.back() - Utils::gravity * imu_preintegrator.sum_dt + last_R * imu_preintegrator.delta_v);
    }else{
        pose_r_arr.push_back(Eigen::Vector3d::Zero());
        pose_t_arr.push_back(Eigen::Vector3d::Zero());
        pose_v_arr.push_back(pose_v_arr.back());
    }

    Eigen::Vector3d b_a_hat = pose_b_a_arr.back();
    pose_b_a_arr.push_back(b_a_hat);
    Eigen::Vector3d b_g_hat = pose_b_g_arr.back();
    pose_b_g_arr.push_back(b_g_hat);

    lidar_odom_arr.push_back(Eigen::Isometry3d::Identity());
}

void OdomEstimationClass::addLidarFeature(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr edge_in,
        const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr plane_in){
    Eigen::Isometry3d T_bl = common_param.getTbl();
    // plane decode
    current_plane_num = static_cast<int>(plane_in->at(0).x);
    m_current_plane_info.clear();
    std::vector<int> indexs;
    for(auto i = 1;i<current_plane_num+1;i++){
        indexs.push_back(i);
    }
    for (auto i : indexs)
    {
        Eigen::Vector4d plane(plane_in->at(i).x, plane_in->at(i).y, plane_in->at(i).z, plane_in->at(i).data[3]);
        // plane = T_bl.matrix().transpose().inverse()*plane;
        m_current_plane_info[plane_in->at(i).label] = plane;
    }
    plane_in->erase(plane_in->begin(), plane_in->begin() + current_plane_num + 1);

    // line decode
    current_line_num = static_cast<int>(edge_in->at(0).x);
    std::vector<int> indexs_line;
    m_current_line_info.clear();
    for(auto i = 1;i<current_line_num+1;i++){
        indexs_line.push_back(i);
    }
    for (auto i : indexs_line)
    {
        Eigen::Vector4d line_direction(edge_in->at(i).x,edge_in->at(i).y,edge_in->at(i).z,0.f);
        // line_direction = T_bl.matrix()*line_direction;
        m_current_line_info[edge_in->at(i).label] = line_direction;
    }
    edge_in->erase(edge_in->begin(),edge_in->begin()+current_line_num+1);
    current_edge_points = edge_in;
    current_plane_points = plane_in;
    // pcl::transformPointCloud(*edge_in, *current_edge_points, T_bl.cast<float>());
    // pcl::transformPointCloud(*plane_in, *current_plane_points, T_bl.cast<float>());
}

void OdomEstimationClass::addEdgeCost(ceres::Problem& problem, ceres::LossFunction *loss_function, double* pose, int cnt){
    int edge_num=0;
    Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
    T_wb.linear() = Utils::so3ToR(Eigen::Vector3d(pose[0],pose[1],pose[2]));
    T_wb.translation() = Eigen::Vector3d(pose[3],pose[4],pose[5]);
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr transformed_edge = pcl::PointCloud<pcl::PointXYZRGBL>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBL>());
    pcl::transformPointCloud(*current_edge_points, *transformed_edge, T_wb.cast<float>());

    std::map<int,Eigen::Vector4d> tmp_current_line_info;
    for (const auto &it:m_current_line_info)
        tmp_current_line_info[it.first]=T_wb.matrix() * it.second;

    for (int i = 0; i < (int)transformed_edge->points.size(); i++){
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        bool lineValid = true;
        edge_kd_tree.nearestKSearch(transformed_edge->points[i], 5, pointSearchInd, pointSearchSqDis);

        if (pointSearchSqDis[4] < 0.5)
        {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++){
                Eigen::Vector3d tmp(edge_map->points[pointSearchInd[j]].x,
                                    edge_map->points[pointSearchInd[j]].y,
                                    edge_map->points[pointSearchInd[j]].z);
                center = center + tmp;
                nearCorners.push_back(tmp);
                double two_normal_dot = tmp_current_line_info[transformed_edge->points[i].label].head(3).dot(
                        (pm_line_info->at(edge_map->points[pointSearchInd[j]].label).head(3)));

                if(abs(two_normal_dot)<cos((60. - cnt*15. )* M_PI / 180.))
                {
                    lineValid = false;
                    break;
                }
            }

            center = center / 5.0;

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < 5; j++){
                Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d curr_point(current_edge_points->points[i].x, current_edge_points->points[i].y, current_edge_points->points[i].z);
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1] && lineValid){
                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;

                ceres::CostFunction *cost_function = new LidarEdgeFactor(curr_point, point_a, point_b, lidar_param.getEdgeN());
//                ceres::CostFunction *cost_function = new LidarEdgeFactor(curr_point, point_a, point_b,
//                        lidar_param.getEdgeN()/current_edge_points->points[i].label);

                problem.AddResidualBlock(cost_function, loss_function, pose);
                edge_num++;
            }
        }
    }
    if(edge_num<20){
        std::cout<<"not enough correct edge points"<<std::endl;
    }

}

void OdomEstimationClass::addPlaneCost(ceres::Problem& problem, ceres::LossFunction *loss_function, double* pose, int cnt){
    int plane_num=0;
    Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
    T_wb.linear() = Utils::so3ToR(Eigen::Vector3d(pose[0],pose[1],pose[2]));
    T_wb.translation() = Eigen::Vector3d(pose[3],pose[4],pose[5]);
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr transformed_plane = pcl::PointCloud<pcl::PointXYZRGBL>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBL>());
    pcl::transformPointCloud(*current_plane_points, *transformed_plane, T_wb.cast<float>());
    std::map<int,Eigen::Vector4d> tmp_current_plane_info;
    for (const auto &it:m_current_plane_info)
        tmp_current_plane_info[it.first]=T_wb.matrix().transpose().inverse() * it.second;
    for (int i = 0; i < (int) transformed_plane->points.size(); i++){
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        plane_kd_tree.nearestKSearch(transformed_plane->points[i], 5, pointSearchInd, pointSearchSqDis);
        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        bool planeValid = true;
        if (pointSearchSqDis[4] < 0.5){
            for (int j = 0; j < 5; j++){
                matA0(j, 0) = plane_map->points[pointSearchInd[j]].x;
                matA0(j, 1) = plane_map->points[pointSearchInd[j]].y;
                matA0(j, 2) = plane_map->points[pointSearchInd[j]].z;
            }
            // find the norm of plane
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double negative_OA_dot_norm = 1 / norm.norm();
            norm.normalize();


            for (int j = 0; j < 5; j++)
            {
                // if OX * n > 0.2, then plane is not fit well
                if (fabs(norm(0) * plane_map->points[pointSearchInd[j]].x +
                         norm(1) * plane_map->points[pointSearchInd[j]].y +
                         norm(2) * plane_map->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2 )
                {
                    planeValid = false;
                    break;
                }
                double two_normal_dot = tmp_current_plane_info[transformed_plane->points[i].label].head(3).dot(
                        (pm_plane_info->at(plane_map->points[pointSearchInd[j]].label).head(3)));
                if(abs(two_normal_dot)<cos((40. - cnt*15. )* M_PI / 180.))
                {
                    planeValid = false;
                    break;
                }
            }
            Eigen::Vector3d curr_point(current_plane_points->points[i].x, current_plane_points->points[i].y, current_plane_points->points[i].z);
            if (planeValid)
            {
                ceres::CostFunction *cost_function = new LidarPlaneFactor(curr_point, norm, negative_OA_dot_norm, lidar_param.getPlaneN());
                problem.AddResidualBlock(cost_function, loss_function, pose);
                plane_num++;
            }
        }

    }
    if(plane_num<10){
        std::cout<<"not enough correct plane points"<<std::endl;
    }
}
void OdomEstimationClass::addImuCost(ImuPreintegrationClass& imu_integrator, ceres::Problem& problem, ceres::LossFunction *loss_function, double* pose1, double* pose2){
    auto* imu_factor = new ImuPreintegrationFactor(imu_integrator);
    problem.AddResidualBlock(imu_factor, loss_function, pose1, pose2);
}

void OdomEstimationClass::addOdometryCost(const Eigen::Isometry3d& odom, ceres::Problem& problem, ceres::LossFunction *loss_function, double* pose1, double* pose2){
    LidarOdometryFactor* odom_factor = new LidarOdometryFactor(odom, lidar_param.getOdomN());
    problem.AddResidualBlock(odom_factor, loss_function, pose1, pose2);
}

void OdomEstimationClass::optimize(){
    if(imu_preintegrator_arr.size()!= lidar_odom_arr.size() || lidar_odom_arr.size() != pose_r_arr.size()-1)
        ROS_WARN("pose num and imu num are not properly aligned");

    const int pose_size = pose_r_arr.size()>common_param.getNearbyFrame()?common_param.getNearbyFrame():pose_r_arr.size();
    const int start_id = pose_r_arr.size() - pose_size;
    const int end_id = pose_r_arr.size() - 1;

    double pose[POSE_BUFFER][15];
    for(int i = start_id; i <= end_id; i++){
        const int pose_id = i - start_id;
        pose[pose_id][0] = pose_r_arr[i].x();
        pose[pose_id][1] = pose_r_arr[i].y();
        pose[pose_id][2] = pose_r_arr[i].z();
        pose[pose_id][3] = pose_t_arr[i].x();
        pose[pose_id][4] = pose_t_arr[i].y();
        pose[pose_id][5] = pose_t_arr[i].z();
        pose[pose_id][6] = pose_v_arr[i].x();
        pose[pose_id][7] = pose_v_arr[i].y();
        pose[pose_id][8] = pose_v_arr[i].z();
        pose[pose_id][9] = pose_b_a_arr[i].x();
        pose[pose_id][10] = pose_b_a_arr[i].y();
        pose[pose_id][11] = pose_b_a_arr[i].z();
        pose[pose_id][12] = pose_b_g_arr[i].x();
        pose[pose_id][13] = pose_b_g_arr[i].y();
        pose[pose_id][14] = pose_b_g_arr[i].z();
    }
    for (int iterCount = 0; iterCount < 3; iterCount++){
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.5);
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);

        for(int i = start_id; i <= end_id; i++){
            const int pose_id = i - start_id;
            if(pose_id == 0)
                problem.AddParameterBlock(pose[pose_id], 15, new ConstantPoseParameterization());
            else
                problem.AddParameterBlock(pose[pose_id], 15, new PoseParameterization());
        }

        //add imu cost factor
        for (int i = end_id-1; i < end_id; i++){
            const int pose_id = i - start_id;
            addImuCost(imu_preintegrator_arr[i], problem, loss_function, pose[pose_id], pose[pose_id+1]);
        }
        addEdgeCost(problem, loss_function, pose[pose_size-1],iterCount);
        addPlaneCost(problem, loss_function, pose[pose_size - 1], iterCount);

        // add odometry cost factor
        for (int i = start_id; i < end_id - 1; i++){
            const int pose_id = i - start_id;
//            addOdometryCost(lidar_odom_arr[i], problem, loss_function, pose[pose_id], pose[pose_id+1]);
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 6;
        options.gradient_check_relative_precision = 1e-4;
        options.max_solver_time_in_seconds = 0.03;
        options.num_threads = common_param.getCoreNum();
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
    }
    for(int i = start_id; i<= end_id; i++){
        const int pose_id = i - start_id;
        pose_r_arr[i].x() = pose[pose_id][0];
        pose_r_arr[i].y() = pose[pose_id][1];
        pose_r_arr[i].z() = pose[pose_id][2];
        pose_t_arr[i].x() = pose[pose_id][3];
        pose_t_arr[i].y() = pose[pose_id][4];
        pose_t_arr[i].z() = pose[pose_id][5];
        pose_v_arr[i].x() = pose[pose_id][6];
        pose_v_arr[i].y() = pose[pose_id][7];
        pose_v_arr[i].z() = pose[pose_id][8];
        pose_b_a_arr[i].x() = pose[pose_id][9];
        pose_b_a_arr[i].y() = pose[pose_id][10];
        pose_b_a_arr[i].z() = pose[pose_id][11];
        pose_b_g_arr[i].x() = pose[pose_id][12];
        pose_b_g_arr[i].y() = pose[pose_id][13];
        pose_b_g_arr[i].z() = pose[pose_id][14];
    }
    
    for(int i = start_id; i < end_id; i++){
        imu_preintegrator_arr[i].update(pose_b_a_arr[i], pose_b_g_arr[i]);
    }
    // update odom
    for(int i = end_id - 1; i < end_id; i++){
        Eigen::Matrix3d last_R = Utils::so3ToR(pose_r_arr[i]);
        lidar_odom_arr[i].linear() = last_R.transpose() * Utils::so3ToR(pose_r_arr[i+1]);
        lidar_odom_arr[i].translation() = last_R.transpose() * (pose_t_arr[i+1] - pose_t_arr[i]);
    }
    current_pose = lidar_odom_arr[end_id];
    // update map
    Eigen::Isometry3d current_pose_tmp = Eigen::Isometry3d::Identity();
    current_pose_tmp.linear() = Utils::so3ToR(pose_r_arr.back());
    current_pose_tmp.translation() = pose_t_arr.back();
    updateLocalMap(current_pose_tmp);
}


void OdomEstimationClass::updateLocalMap(Eigen::Isometry3d& transform){

    for (auto &it:m_current_plane_info){
        it.second = transform.matrix().transpose().inverse()*it.second;
    }
    for (auto &it:m_current_line_info){
        it.second = transform.matrix()*it.second;
    }

    for(const auto& it:m_current_plane_info)
        (*pm_plane_info)[it.first] = it.second;
    for(const auto& it:m_current_line_info)
        (*pm_line_info)[it.first] = it.second;

    Eigen::Isometry3d delta_transform = last_pose.inverse() * current_pose;
    double x_min = transform.translation().x() - lidar_param.getLocalMapSize();
    double y_min = transform.translation().y() - lidar_param.getLocalMapSize();
    double z_min = transform.translation().z() - lidar_param.getLocalMapSize();
    double x_max = transform.translation().x() + lidar_param.getLocalMapSize();
    double y_max = transform.translation().y() + lidar_param.getLocalMapSize();
    double z_max = transform.translation().z() + lidar_param.getLocalMapSize();

    pcl::CropBox<pcl::PointXYZRGBL> crop_box_filter;
    crop_box_filter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
    crop_box_filter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
    crop_box_filter.setNegative(false);

    crop_box_filter.setInputCloud(edge_map);
    crop_box_filter.filter(*edge_map);
    crop_box_filter.setInputCloud(plane_map);
    crop_box_filter.filter(*plane_map);

    pcl::transformPointCloud(*current_edge_points, *current_edge_points, transform.cast<float>());
    pcl::transformPointCloud(*current_plane_points, *current_plane_points, transform.cast<float>());

    edge_downsize_filter.setInputCloud(edge_map);
    edge_downsize_filter.filter(*edge_map);
    plane_downsize_filter.setInputCloud(plane_map);
    plane_downsize_filter.filter(*plane_map);


    last_pose = current_pose;
    *edge_map += *current_edge_points;
    *plane_map += *current_plane_points;
    edge_kd_tree.setInputCloud(edge_map);
    plane_kd_tree.setInputCloud(plane_map);
}

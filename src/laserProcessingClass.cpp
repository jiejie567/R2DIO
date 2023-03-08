// Author of SSL_SLAM3: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#include "laserProcessingClass.h"
#include <thread>
#include "omp.h"


void LaserProcessingClass::init(std::string& file_path){


    lidar_param.loadParam(file_path);
    common_param.loadParam(file_path);
    num_of_plane=0;
    num_of_line=0;
    gap_line = lidar_param.gap_line;
    gap_plane = lidar_param.gap_plane;
}

 void LaserProcessingClass::lineFilter(cv::Mat& color_im,cv::Mat_<cv::Vec3f>& cloud_peac,pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& pc_out_line)
{

    int length_threshold = 30;
    int distance_threshold = 2;
    int canny_th1 = 50;
    int canny_th2 = 50;
    int canny_aperture_size = 3;
    bool do_merge = true;

    cv::Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(length_threshold,
                                                                                       distance_threshold, canny_th1, canny_th2, canny_aperture_size,
                                                                                       do_merge);
    cv::Mat image_gray(color_im.rows, color_im.cols, CV_8U);
    cvtColor(color_im, image_gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::Vec4f> lines_fld;
    fld->detect(image_gray, lines_fld);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_all_line(new pcl::PointCloud<pcl::PointXYZRGBL>);
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr line_info_cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
    int line_cnt = 0;
    for(auto &  l: lines_fld)
    {
        cv::LineIterator lit(color_im, cv::Point(l[0],l[1]), cv::Point(l[2],l[3]));//todo Note
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
        cloud->reserve(100);
        int gap_tmp_line = gap_line;
        if(lit.count < gap_tmp_line * 5)
            gap_tmp_line = gap_tmp_line / 2;
        for(int  j = 0; j < lit.count; ++j, ++lit)
        {
            if(j % gap_tmp_line == 0) {
                int col = lit.pos().x;
                int row = lit.pos().y;
                pcl::PointXYZRGBL pt;
                pt.x = cloud_peac[row][col][0] / 1000.; //mm -> m
                pt.y = cloud_peac[row][col][1] / 1000.;
                pt.z = cloud_peac[row][col][2] / 1000.;
                if(pt.z==0.)
                    continue;
                pt.b = color_im.ptr<uchar>(row)[col * 3];
                pt.g = color_im.ptr<uchar>(row)[col * 3 + 1];
                pt.r = color_im.ptr<uchar>(row)[col * 3 + 2];
                pt.label = num_of_line;
                cloud->emplace_back(pt);
            }
        }

        if(cloud->size()<3)
            continue;

        //-----------------------------拟合直线-----------------------------
        pcl::SampleConsensusModelLine<pcl::PointXYZRGBL>::Ptr model_line(new pcl::SampleConsensusModelLine<pcl::PointXYZRGBL>(cloud));
        pcl::RandomSampleConsensus<pcl::PointXYZRGBL> ransac(model_line);
        ransac.setDistanceThreshold(0.01);	//内点到模型的最大距离
        ransac.setMaxIterations(1000);		//最大迭代次数
        ransac.computeModel();				//直线拟合
        //--------------------------根据索引提取内点------------------------
        vector<int> inliers;
        ransac.getInliers(inliers);
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZRGBL>);
        pcl::copyPointCloud<pcl::PointXYZRGBL>(*cloud, inliers, *cloud_line);
        *cloud_all_line += *cloud_line;

        Eigen::VectorXf coef;
        ransac.getModelCoefficients(coef);
        //tramsform to IMU
        Eigen::Vector4d tmp(coef[3],coef[4],coef[5],0.0);
        tmp = common_param.getTbl().matrix()*tmp;

        pcl::PointXYZRGBL line_direction_info;
        line_direction_info.x = tmp[0]; //nx
        line_direction_info.y = tmp[1]; //ny
        line_direction_info.z = tmp[2]; //nz
        line_direction_info.label = num_of_line;



        line_info_cloud->push_back(line_direction_info);

        num_of_line++;
        line_cnt++;
    }
    pcl::PointXYZRGBL line_num;
    line_num.x = static_cast<float>(line_cnt);
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr line_num_cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
    line_num_cloud->push_back(line_num);
    //transform to IMU
    pcl::transformPointCloud(*cloud_all_line, *cloud_all_line, common_param.getTbl().cast<float>());
    *pc_out_line = *line_num_cloud + *line_info_cloud + *cloud_all_line;
}

void LaserProcessingClass::featureExtraction(cv::Mat& color_im, cv::Mat& depth_im, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& pc_out_line,
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& pc_out_plane, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_filter){
    struct OrganizedImage3D {
        const cv::Mat_<cv::Vec3f>& cloud_peac;
        //note: ahc::PlaneFitter assumes mm as unit!!!
        OrganizedImage3D(const cv::Mat_<cv::Vec3f>& c): cloud_peac(c) {}
        inline int width() const { return cloud_peac.cols; }
        inline int height() const { return cloud_peac.rows; }
        inline bool get(const int row, const int col, double& x, double& y, double& z) const {
            const cv::Vec3f& p = cloud_peac.at<cv::Vec3f>(row,col);
            x = p[0];
            y = p[1];
            z = p[2];
            return z > 0 && isnan(z)==0; //return false if current depth is NaN
        }
    };
    typedef ahc::PlaneFitter< OrganizedImage3D > PlaneFitter;
    cv::Mat_<cv::Vec3f> cloud_peac(depth_im.rows, depth_im.cols);
    cloud_filter->resize(depth_im.rows*depth_im.cols);
    omp_set_num_threads(2);
#pragma omp parallel for
    for (int r = 0; r < depth_im.rows; r++) {
        const float *depth_ptr = depth_im.ptr<float>(r);
        cv::Vec3f *pt_ptr = cloud_peac.ptr<cv::Vec3f>(r);
        for (int c = 0; c < depth_im.cols; c++) {
            float z = (float) depth_ptr[c] / lidar_param.camera_factor;
            if (z > lidar_param.max_distance || z < lidar_param.min_distance || isnan(z)) {
                z = 0.0;
                depth_im.at<float>(r, c) = 0;
            }
            pcl::PointXYZRGB &p = cloud_filter->points[r * depth_im.cols + c];
            p.z = z;
            p.x = (c - lidar_param.camera_cx) * p.z / lidar_param.camera_fx;
            p.y = (r - lidar_param.camera_cy) * p.z / lidar_param.camera_fy;

            p.b = color_im.ptr<uchar>(r)[c * 3];
            p.g = color_im.ptr<uchar>(r)[c * 3 + 1];
            p.r = color_im.ptr<uchar>(r)[c * 3 + 2];

            pt_ptr[c][0] = p.x * lidar_param.camera_factor;//m->mm
            pt_ptr[c][1] = p.y * lidar_param.camera_factor;//m->mm
            pt_ptr[c][2] = z * lidar_param.camera_factor;//m->mm
        }
    }
    // transform to IMU
    Eigen::Isometry3d T_bl = common_param.getTbl();
    pcl::transformPointCloud(*cloud_filter, *cloud_filter, T_bl.cast<float>());
    // line filter
    thread th2(&LaserProcessingClass::lineFilter, this, std::ref(color_im), std::ref(cloud_peac), std::ref(pc_out_line));

    // plane filter

    PlaneFitter pf;
    pf.minSupport = 300;
    pf.windowWidth = 8;
    pf.windowHeight = 8;
    pf.doRefine = true;

    cv::Mat seg(depth_im.rows, depth_im.cols, CV_8UC3);
    std::vector<std::vector<int>> vSeg;
    OrganizedImage3D Ixyz(cloud_peac);
    pf.run(&Ixyz, &vSeg, &seg);
    int gap_tmp = gap_plane;

    // pcl 拟合平面
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_all_plane(new pcl::PointCloud<pcl::PointXYZRGBL>);
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr plane_info_cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
    int plane_cnt = 0;

    for(auto idx_plane = 0; idx_plane<vSeg.size();idx_plane++) {
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
        cloud->reserve(1000);
        for (auto idx_idx = 0; idx_idx < vSeg[idx_plane].size(); idx_idx++) {
            pcl::PointXYZRGBL pt;
            int pt_idx = vSeg[idx_plane].at(idx_idx);
            int row = pt_idx / depth_im.cols;
            int col = pt_idx % depth_im.cols;
            if(row%gap_tmp==0 && col%gap_tmp==0)
            {
                pt.x = cloud_peac[row][col][0] / 1000.; //mm -> m
                pt.y = cloud_peac[row][col][1] / 1000.;
                pt.z = cloud_peac[row][col][2] / 1000.;

                pt.b = seg.ptr<uchar>(row)[col * 3];
                pt.g = seg.ptr<uchar>(row)[col * 3 + 1];
                pt.r = seg.ptr<uchar>(row)[col * 3 + 2];
                pt.label = num_of_plane;
                cloud->emplace_back(pt);
            }
        }

        if(cloud->size()<3) {
            continue;
        }

        //--------------------------RANSAC拟合平面--------------------------
        pcl::SampleConsensusModelPlane<pcl::PointXYZRGBL>::Ptr model_plane(
                new pcl::SampleConsensusModelPlane<pcl::PointXYZRGBL>(cloud));
        pcl::RandomSampleConsensus<pcl::PointXYZRGBL> ransac(model_plane);
        ransac.setDistanceThreshold(0.02);    //设置距离阈值，与平面距离小于0.1的点作为内点
        ransac.computeModel();                //执行模型估计
        //-------------------------根据索引提取内点--------------------------
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGBL>);
        std::vector<int> inliers;                //存储内点索引的容器
        ransac.getInliers(inliers);            //提取内点索引
        pcl::copyPointCloud<pcl::PointXYZRGBL>(*cloud, inliers, *cloud_plane);

        *cloud_all_plane += *cloud_plane;

        //----------------------------输出模型参数---------------------------
        Eigen::VectorXf coefficient;
        ransac.getModelCoefficients(coefficient);
        if(coefficient[3]<0)
        {
            coefficient = -coefficient;
        }
        // transfrom to IMU
        Eigen::Vector4d tmp(coefficient[0],coefficient[1],coefficient[2],coefficient[3]);
        tmp = T_bl.matrix().transpose().inverse()*tmp;

        pcl::PointXYZRGBL plane_info;
        plane_info.x = tmp[0];
        plane_info.y = tmp[1];
        plane_info.z = tmp[2];
        plane_info.data[3] = tmp[3];
        plane_info.rgb = cloud_plane->size();
        plane_info.label = num_of_plane;
        plane_info_cloud->push_back(plane_info);

        num_of_plane++;
        plane_cnt++;
    }
    pcl::PointXYZRGBL plane_num;
    plane_num.x = static_cast<float>(plane_cnt);
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr plane_num_cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
    plane_num_cloud->push_back(plane_num);
    //tramsform to IMU
    pcl::transformPointCloud(*cloud_all_plane, *cloud_all_plane, T_bl.cast<float>());
    *pc_out_plane = *plane_num_cloud + *plane_info_cloud + *cloud_all_plane;//num of plane + plane info + plane points
    th2.join();
}

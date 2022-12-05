// Author of SSL_SLAM3: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _LIDAR_OPTIMIZATION_FACTOR_H_
#define _LIDAR_OPTIMIZATION_FACTOR_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "utils.h"
#include <ros/ros.h>
class LidarOdometryFactor : public ceres::SizedCostFunction<6, 15, 15>{
public:
    LidarOdometryFactor(Eigen::Isometry3d odom_in, Eigen::Matrix<double, 6, 1> covariance_in);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    Eigen::Isometry3d odom;
    Eigen::Matrix<double, 6, 6> sqrt_info;
};

class LidarEdgeFactor : public ceres::SizedCostFunction<1, 15> {
public:
	LidarEdgeFactor(Eigen::Vector3d curr_point_in, Eigen::Vector3d last_point_a_in, Eigen::Vector3d last_point_b_in, double covariance_in);
	virtual ~LidarEdgeFactor() {}
	virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

	Eigen::Vector3d curr_point;
    Eigen::Vector3d last_point_a;
	Eigen::Vector3d last_point_b;
	double sqrt_info;
};


class LidarPlaneFactor : public ceres::SizedCostFunction<1, 15> {
public:
    LidarPlaneFactor(Eigen::Vector3d curr_point_in, Eigen::Vector3d plane_unit_norm_in, double negative_OA_dot_norm_in, double covariance_in);
    virtual ~LidarPlaneFactor() {}
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    Eigen::Vector3d curr_point;
    Eigen::Vector3d plane_unit_norm;
    double negative_OA_dot_norm;
    double sqrt_info;
};
struct NumericDiffCostFunctor {
    Eigen::Vector4d current_plane_hessian_;
    Eigen::Vector4d target_plane_hessian_;
    double sqrt_info;
    int quantity_plane_matched_;

    NumericDiffCostFunctor(Eigen::Vector4d current_plane_hessian, Eigen::Vector4d traget_plane_hessian, double quantity_plane_matched, double covariance_in){
        current_plane_hessian_ = current_plane_hessian;
        target_plane_hessian_ = traget_plane_hessian;
        quantity_plane_matched_ = quantity_plane_matched;
        sqrt_info = 1.0 /covariance_in;
    }
    bool operator()(const double* const parameters, double *residuals) const {
        Eigen::Vector3d ri(parameters[0], parameters[1], parameters[2]);
        Eigen::Vector3d Pi(parameters[3], parameters[4], parameters[5]);
        Eigen::Matrix3d Ri = Utils::so3ToR(ri);
        Eigen::Vector3d plane_n = current_plane_hessian_.head(3);
        double plane_d= current_plane_hessian_[3];
        Eigen::Vector3d plane_n_transed = Ri*plane_n;
        auto pi_transepose = Pi.transpose();
        double plane_d_transed = -pi_transepose*plane_n_transed+plane_d;
        if(plane_d_transed<0)
        {
            plane_d_transed = -plane_d_transed;
            plane_n_transed = -plane_n_transed;
        }
        Eigen::Vector3d current_plane_closest_point = plane_n_transed*plane_d_transed;
        Eigen::Vector3d target_plane_closest_point = target_plane_hessian_.head(3)*target_plane_hessian_[3];

        Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
        T_wb.linear() = Utils::so3ToR(Eigen::Vector3d(parameters[0],parameters[1],parameters[2]));
        T_wb.translation() = Eigen::Vector3d(parameters[3],parameters[4],parameters[5]);

//        residuals[0] = sqrt_info*quantity_plane_matched_*(T_wb.matrix().transpose().inverse()*current_plane_hessian_-target_plane_hessian_).norm();

//        residuals[0] = sqrt_info*quantity_plane_matched_*(current_plane_closest_point-target_plane_closest_point).norm();
        Eigen::Map<Eigen::Matrix<double, 1, 1> > residuals_i(residuals);
        residuals_i[0] =  sqrt_info*quantity_plane_matched_*(current_plane_closest_point-target_plane_closest_point).norm();
        return true;
    }
};
struct NumericDiffCostLineFunctor {
    Eigen::Vector3d curr_end_pt;
    Eigen::Vector3d last_point_a;
    Eigen::Vector3d last_point_b;
    double sqrt_info;
    double weight_;

    NumericDiffCostLineFunctor(Eigen::Vector4d curr_end_, Eigen::Vector4d last_point_a_in,
            Eigen::Vector4d last_point_b_in, double covariance_in,
                               double weight){
        curr_end_pt = curr_end_.head(3);
        last_point_a = last_point_a_in.head(3);
        last_point_b = last_point_b_in.head(3);
        sqrt_info = 1.0 / covariance_in;
        weight_ = weight;
    }
    bool operator()(const double* const parameters, double *residuals) const {
        Eigen::Vector3d ri(parameters[0], parameters[1], parameters[2]);
        Eigen::Vector3d Pi(parameters[3], parameters[4], parameters[5]);
        Eigen::Matrix3d Ri = Utils::so3ToR(ri);

        Eigen::Vector3d lp = Ri * curr_end_pt + Pi;
        Eigen::Vector3d nu = (lp - last_point_a).cross(lp - last_point_b);
        Eigen::Vector3d de = last_point_a - last_point_b;
        residuals[0] = weight_*sqrt_info * nu.norm() / de.norm();
        return true;
    }
};
#endif // _LIDAR_OPTIMIZATION_FACTOR_H_


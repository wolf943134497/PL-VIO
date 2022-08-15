/*
 * SPL_VIO
 * Copyright (C) 2022-2023 Yanyu Zhang
 * Copyright (C) 2022-2023 Wei Ren
 */

#ifndef OV_CORE_LINE_GEOMETRY_H
#define OV_CORE_LINE_GEOMETRY_H

#include <Eigen/Eigen>
#include "utils/quat_ops.h"

namespace ov_core {

inline Eigen::Matrix<double,6,1> endpoints_2_plk(const Eigen::Matrix<double, 3, 1> &p_f1, const Eigen::Matrix<double, 3, 1> &p_f2) {
    Eigen::Matrix<double, 3, 1> n_l = skew_x(p_f1) * p_f2;
    Eigen::Matrix<double, 3, 1> v_l = p_f2 - p_f1;
    Eigen::Matrix<double, 6, 1> plk;
    plk << n_l(0), n_l(1), n_l(2), v_l(0), v_l(1), v_l(2);
    return plk;
}

inline Eigen::Matrix3d plk_2_rot(const Eigen::Matrix<double,6,1> &plk) {
    Eigen::Matrix<double, 3, 1> n_l = plk.head(3);
    Eigen::Matrix<double, 3, 1> v_l = plk.tail(3);

    Eigen::Matrix<double, 3, 1> n_e = n_l/n_l.norm();
    Eigen::Matrix<double, 3, 1> v_e = v_l/v_l.norm();
    Eigen::Matrix<double, 3, 1> ne_ve = skew_x(n_e)*v_e;
    
    Eigen::Matrix3d rot;
    rot.block(0, 0, 3, 1) = n_e;
    rot.block(0, 1, 3, 1) = v_e;
    rot.block(0, 2, 3, 1) = ne_ve;
    return rot;
}

inline double plk_2_d(const Eigen::Matrix<double,6,1> &plk) {
    Eigen::Matrix<double, 3, 1> n_l = plk.head(3);
    Eigen::Matrix<double, 3, 1> v_l = plk.tail(3);

    double d = n_l.norm()/v_l.norm();
    return d;
}

inline Eigen::Matrix<double,4,1> quat_2_cp(const Eigen::Matrix<double, 4, 1> &quat, const double &d) {
    Eigen::Matrix<double,4,1> result = d * quat;
    return result;
}

inline double cp_2_depth(const Eigen::Matrix<double,4,1> &cp) {
    double depth = cp.norm();
    return depth;
}

inline Eigen::Matrix3d cp_2_rot(const Eigen::Matrix<double,4,1> &cp) {
    double depth = cp_2_depth(cp);
    Eigen::Matrix<double,4,1> quat = cp/depth;
    Eigen::Matrix<double, 3, 3> rot = quat_2_Rot(quat);
    return rot;
}

// ======================================================================================
// Not unique solution: here assume v_l is unit vector
// ======================================================================================
inline Eigen::Matrix<double,6,1> rot_to_plk(const Eigen::Matrix3d &rot, const double &d) {
    Eigen::Matrix<double, 3, 1> n_l = rot.block(0, 0, 3, 1)*d;
    Eigen::Matrix<double, 3, 1> v_l = rot.block(0, 1, 3, 1);
    Eigen::Matrix<double, 6, 1> plk;
    plk << n_l(0), n_l(1), n_l(2), v_l(0), v_l(1), v_l(2);
    return plk;
}

// ======================================================================================
// This function needs to double check
// ======================================================================================
inline Eigen::Matrix<double,6,1> plk_2_endpoints(const Eigen::Matrix<double,6,1> &plk) {
    Eigen::Matrix<double, 3, 1> n_l = plk.head(3);
    Eigen::Matrix<double, 3, 1> v_l = plk.tail(3);
    
    Eigen::Matrix<double, 3, 3> p_f1_rot = ((v_l*v_l.transpose()).inverse()*v_l*n_l.transpose()).transpose();
    Eigen::Matrix<double, 3, 1> p_f1 = vee(p_f1_rot);
    Eigen::Matrix<double, 3, 1> p_f2 = p_f1 + v_l;
    
    Eigen::Matrix<double, 6, 1> endpoints;
    endpoints << p_f1(0), p_f1(1), p_f1(2), p_f2(0), p_f2(1), p_f2(2);
    return endpoints;
}
// ======================================================================================

inline Eigen::Vector4d ppp_2_plane(Eigen::Vector3d &x1, Eigen::Vector3d &x2, Eigen::Vector3d &x3) {
    Eigen::Vector4d plane;
    plane << (x1 - x3).cross(x2 - x3), -x3.dot(x1.cross(x2)); 
    // d = - x3.dot( (x1-x3).cross( x2-x3 ) ) = - x3.dot( x1.cross( x2 ) )

    return plane;
}

inline Eigen::Matrix<double,6,1> pipi_plk( Eigen::Vector4d pi1, Eigen::Vector4d pi2){
    Eigen::Matrix<double,6,1> plk;
    Eigen::Matrix4d dp = pi1 * pi2.transpose() - pi2 * pi1.transpose();

    plk << dp(0,3), dp(1,3), dp(2,3), - dp(1,2), dp(0,2), - dp(0,1);
    return plk;
}

inline Eigen::Vector4d pi_from_ppp(Eigen::Vector3d x1, Eigen::Vector3d x2, Eigen::Vector3d x3) {
    Eigen::Vector4d pi;
    pi << ( x1 - x3 ).cross( x2 - x3 ), - x3.dot( x1.cross( x2 ) ); // d = - x3.dot( (x1-x3).cross( x2-x3 ) ) = - x3.dot( x1.cross( x2 ) )

    return pi;
}


} // namespace ov_core

#endif  /* OV_CORE_LINE_GEOMETRY_H */

/*
 * SPL_VIO
 * Copyright (C) 2022-2023 Yanyu Zhang
 * Copyright (C) 2022-2023 Wei Ren
 */

#include "LineInitializer.h"
#include <Eigen/Eigen>
#include <fstream>

using namespace ov_core;

bool LineInitializer::line_triangulation(Line *line, std::unordered_map<size_t, std::unordered_map<double, ClonePose>> &clonesCAM) {
    std::ofstream myfile4;
    myfile4.open("/home/zhangyanyu/catkin_ws_ov/src/open_vins/Debug/sim_line_est.txt", std::ofstream::app);


    // Total number of measurements
    // Also set the first measurement to be the anchor frame
    int total_meas = 0;
    size_t anchor_most_meas = 0;
    size_t most_meas = 0;
    for (auto const &pair : line->timestamps) {
        total_meas += (int)pair.second.size();
        if (pair.second.size() > most_meas) {
            anchor_most_meas = pair.first;
            most_meas = pair.second.size();
        }
    }
  
    line->anchor_cam_id = anchor_most_meas;
    line->anchor_clone_timestamp = line->timestamps.at(line->anchor_cam_id).back();

    // Our linear system matrices
    Eigen::Matrix3d N = Eigen::Matrix3d::Zero();
    
    // Get the position of the anchor pose
    ClonePose anchorclone = clonesCAM.at(line->anchor_cam_id).at(line->anchor_clone_timestamp);
    const Eigen::Matrix<double, 3, 3> &R_GtoA = anchorclone.Rot();
    const Eigen::Matrix<double, 3, 1> &p_AinG = anchorclone.pos();

    Eigen::Matrix<double, 3, 1> n_e1_C1;

    // Loop through each camera for this feature
    for (auto const &pair : line->timestamps) {
    
        // Add CAM_I features
        for (size_t m = 0; m < line->timestamps.at(pair.first).size(); m++) {

            // Get the position of this clone in the global
            const Eigen::Matrix<double, 3, 3> &R_GtoCi = clonesCAM.at(pair.first).at(line->timestamps.at(pair.first).at(m)).Rot();

            // Convert current position relative to anchor
            Eigen::Matrix<double, 3, 3> R_AtoCi;
            R_AtoCi.noalias() = R_GtoCi * R_GtoA.transpose();

            // Get the line endpoints
            Eigen::Matrix<double, 3, 1> startpoint, endpoint;
                 
            startpoint << line->startpoint_norm.at(pair.first).at(m)(0), line->startpoint_norm.at(pair.first).at(m)(1), 1;
            endpoint << line->endpoint_norm.at(pair.first).at(m)(0), line->endpoint_norm.at(pair.first).at(m)(1), 1;
            Eigen::Matrix<double, 3, 1> n_ei_Ci = plk_2_rot(endpoints_2_plk(startpoint, endpoint)).block(0, 0, 3, 1);
//myfile4 << "[Estimation] n_ei_Ci: " << "\n"<< n_ei_Ci <<"\n";
//myfile4 << "[Estimation] R_AtoCi: " << "\n"<< R_AtoCi <<"\n";
            if (m == (line->timestamps.at(pair.first).size()-1) && (pair.first == anchor_most_meas)) {
                n_e1_C1 = n_ei_Ci;
            }

            Eigen::Matrix<double, 1, 3> N_cur;
            N_cur = n_ei_Ci.transpose() * R_AtoCi;           
            N += N_cur.transpose() * N_cur;
        }
    }

    // Solve the linear system
    Eigen::Matrix<double, 3, 1> v_e1_A;
    Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3>> svd(N, Eigen::ComputeFullU | Eigen::ComputeFullV);
    int svd_id = -1;

    //PRINT_DEBUG("[breakpoint 1]: %d \n", svd.singularValues().size());
    int a = svd.singularValues().size()-1;
    if (svd.singularValues()(a) < 0.0001) {
        return false;
    }
/*
    for (int a = (svd.singularValues().size()-1); a >= 0; a--) {
        if (svd.singularValues()(a) > 0.0001) {
            svd_id = a;
            break;
        }
    }
    //PRINT_DEBUG("[breakpoint 2]: %d \n", svd_id);
    if (svd_id == -1) {
        return false;
    }
*/
    Eigen::Matrix<double, 3, 3> v = svd.matrixV();

    v_e1_A = v.block(0, 2, 3, 1);

    double A = 0;
    double B = 0;
    
    // Loop through each camera for this feature
    for (auto const &pair : line->timestamps) {
    
        // Add CAM_I features
        for (size_t n = 0; n < line->timestamps.at(pair.first).size(); n++) {

            // Get the position of this clone in the global
            const Eigen::Matrix<double, 3, 3> &R_GtoCi = clonesCAM.at(pair.first).at(line->timestamps.at(pair.first).at(n)).Rot();
            const Eigen::Matrix<double, 3, 1> &p_CiinG = clonesCAM.at(pair.first).at(line->timestamps.at(pair.first).at(n)).pos();


            // Convert current position relative to anchor
            Eigen::Matrix<double, 3, 3> R_AtoCi;
      	    R_AtoCi.noalias() = R_GtoCi * R_GtoA.transpose();
            Eigen::Matrix<double, 3, 1> p_CiinA;
            p_CiinA.noalias() = R_GtoA * (p_CiinG - p_AinG);

            // Get the line endpoints

            Eigen::Matrix<double, 3, 1> startpoint, endpoint;
            startpoint << line->startpoint_norm.at(pair.first).at(n)(0), line->startpoint_norm.at(pair.first).at(n)(1), 1;
            endpoint << line->endpoint_norm.at(pair.first).at(n)(0), line->endpoint_norm.at(pair.first).at(n)(1), 1;


        
            Eigen::Matrix<double, 3, 1> n_ei_Ci = plk_2_rot(endpoints_2_plk(startpoint, endpoint)).block(0, 0, 3, 1);

            Eigen::Matrix<double, 3, 1> v_ei_Ci = plk_2_rot(endpoints_2_plk(startpoint, endpoint)).block(0, 1, 3, 1);

            Eigen::Matrix<double, 3, 1> bi = skew_x(v_e1_A) * R_AtoCi.transpose() * n_ei_Ci;

            double cur_A = bi.transpose() * n_e1_C1;
            double cur_B = bi.transpose() * skew_x(p_CiinA) * R_AtoCi.transpose() * v_ei_Ci;

            A += cur_A * cur_A;
            B += cur_A * cur_B;
            
        }
    }
 
    // Solve the linear system
    double d_l_C1 = abs(B/A);

    if (std::abs(d_l_C1) < 1 || std::abs(d_l_C1) > 100) {
        return false;
    }

    /*
    if (std::abs(d_l_C1) < 1 || std::abs(d_l_C1) > 10) {
        return false;
    }
    */

    Eigen::Matrix<double, 6, 1> plk_A;
    plk_A << n_e1_C1(0), n_e1_C1(1), n_e1_C1(2), v_e1_A(0), v_e1_A(1), v_e1_A(2);

    Eigen::Matrix<double, 4, 1> p_LinA = d_l_C1 * rot_2_quat(plk_2_rot(plk_A));

    line->p_LinA = p_LinA;

    Eigen::Matrix<double, 3, 1> n_G = R_GtoA.transpose() * d_l_C1 * plk_A.head(3) + skew_x(p_AinG) * R_GtoA.transpose() * plk_A.tail(3);
    double d_l_G = n_G.norm();
/*
    if (std::abs(d_l_G) > 10) {
        return false;
    }
*/
    Eigen::Matrix<double, 3, 1> n_e1_G = n_G / d_l_G;

    Eigen::Matrix<double, 3, 1> v_e1_G = R_GtoA.transpose() * v_e1_A;

    Eigen::Matrix<double, 6, 1> plk_G;
    plk_G << n_e1_G(0), n_e1_G(1), n_e1_G(2), v_e1_G(0), v_e1_G(1), v_e1_G(2);
    Eigen::Matrix<double, 4, 1> p_LinG = d_l_G * rot_2_quat(plk_2_rot(plk_G));

    line->p_LinG = p_LinG;
    
    //std::ofstream myfile4;
    //myfile4.open("/home/zhangyanyu/catkin_ws_ov/src/open_vins/Debug/sim_line_est.txt", std::ofstream::app);
    myfile4 << "[Estimation] n_e1_in_A: " << int(line->lineid) << " -> "<< n_e1_C1(0)<<" "<< n_e1_C1(1)<<" "<< n_e1_C1(2)<<"\n";
    myfile4 << "[Estimation] v_e1_in_A: " << int(line->lineid) << " -> "<< v_e1_A(0)<<" "<< v_e1_A(1)<<" "<< v_e1_A(2)<<"\n";
    myfile4 << "[Estimation] line ID: " << int(line->lineid) << " line depth in A-> " << d_l_C1 << "\n";

    myfile4 << "[Estimation] n_e1_in_G: " << int(line->lineid) << " -> "<<n_e1_G(0)<<" "<<n_e1_G(1)<<" "<<n_e1_G(2)<<"\n";
    myfile4 << "[Estimation] v_e1_in_G: " << int(line->lineid) << " -> "<<v_e1_G(0)<<" "<<v_e1_G(1)<<" "<<v_e1_G(2)<<"\n";
    myfile4 << "[Estimation] line ID: " << int(line->lineid) << " line depth in G -> " << d_l_G << "\n";
    myfile4 << "===============================================================================\n";
    myfile4.close();
    
    return true;
}







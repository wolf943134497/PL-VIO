#include "LineUpdaterHelper.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

void LineUpdaterHelper::get_line_jacobian_representation(std::shared_ptr<State> state, UpdaterHelperLine &line, Eigen::MatrixXd &H_f, std::vector<Eigen::MatrixXd> &H_x, std::vector<std::shared_ptr<Type>> &x_order) {

    // Anchor pose orientation and position, and camera calibration for our anchor camera
/*
    Eigen::Matrix3d R_ItoC = state->_calib_IMUtoCAM.at(line.anchor_cam_id)->Rot();
    Eigen::Vector3d p_IinC = state->_calib_IMUtoCAM.at(line.anchor_cam_id)->pos();
    Eigen::Matrix3d R_GtoI = state->_clones_IMU.at(line.anchor_clone_timestamp)->Rot();
    Eigen::Vector3d p_IinG = state->_clones_IMU.at(line.anchor_clone_timestamp)->pos(); 

    Eigen::Vector3d n_einA = cp_2_rot(line.p_LinA).block(0, 0, 3, 1);
    Eigen::Vector3d v_einA = cp_2_rot(line.p_LinA).block(0, 1, 3, 1);
    double d_linA = cp_2_depth(line.p_LinA);
*/
    Eigen::Vector3d n_einG = cp_2_rot(line.p_LinG).block(0, 0, 3, 1);
    Eigen::Vector3d v_einG = cp_2_rot(line.p_LinG).block(0, 1, 3, 1);
    double d_linG = cp_2_depth(line.p_LinG);

    if (state->_options.do_fej) {
        Eigen::Vector3d n_einG = cp_2_rot(line.p_LinG_fej).block(0, 0, 3, 1);
        Eigen::Vector3d v_einG = cp_2_rot(line.p_LinG_fej).block(0, 1, 3, 1);
        double d_linG = cp_2_depth(line.p_LinG_fej);
    }
/*
    Eigen::Vector3d n_einI = R_ItoC.transpose() * n_einA - R_ItoC.transpose() * skew_x(p_IinC) * v_einA / d_linA;
    Eigen::Vector3d v_einI = R_ItoC.transpose() * v_einA;
    Eigen::Vector3d n_einG_best = R_GtoI.transpose() * n_einI + skew_x(p_IinG) * R_GtoI.transpose() * v_einI / d_linA;
    Eigen::Vector3d v_einG_best = R_GtoI.transpose() * v_einI;

    // If I am doing FEJ, I should FEJ the anchor states (should we fej calibration???)
    // Also get the FEJ position of the feature if we are
    if (state->_options.do_fej) {
        // "Best" feature in the global frame

        // Transform the best into our anchor frame using FEJ
        R_GtoI = state->_clones_IMU.at(line.anchor_clone_timestamp)->Rot_fej();
        p_IinG = state->_clones_IMU.at(line.anchor_clone_timestamp)->pos_fej();

        Eigen::Vector3d n_einI = R_GtoI * n_einG_best - R_GtoI * skew_x(p_IinG) * v_einG_best / d_linA;
        Eigen::Vector3d v_einI = R_GtoI * v_einG_best;
        Eigen::Vector3d n_einA = R_ItoC * n_einI + skew_x(p_IinC) * R_ItoC * v_einI / d_linA;
        Eigen::Vector3d v_einA = R_ItoC * v_einI;
    }

    // Jacobian for our anchor pose
    Eigen::Matrix<double, 6, 6> H_anc;

    Eigen::Matrix<double, 6, 3> d_Li_d_pIinG;
    d_Li_d_pIinG.block(0, 0, 3, 3) = R_GtoI * skew_x(v_einG_best);
    d_Li_d_pIinG.block(3, 0, 3, 3) = Eigen::MatrixXd::Zero(3, 3);
    
    Eigen::Matrix<double, 6, 3> d_Li_d_thetaI;
    d_Li_d_thetaI.block(0, 0, 3, 3) = d_linA * skew_x(R_GtoI * n_einG_best) - skew_x(R_GtoI * skew_x(p_IinG) * v_einG_best);
    d_Li_d_thetaI.block(3, 0, 3, 3) = skew_x(R_GtoI * v_einG_best);

    H_anc.block(0, 0, 6, 3) = d_Li_d_thetaI;
    H_anc.block(0, 3, 6, 3) = d_Li_d_pIinG;

    // Add anchor Jacobians to our return vector
    x_order.push_back(state->_clones_IMU.at(line.anchor_clone_timestamp));
    H_x.push_back(H_anc);
*/

/*
    // Get calibration Jacobians (for anchor clone)
    if (state->_options.do_calib_camera_pose) {
        Eigen::Matrix<double, 6, 6> d_Lc_d_calib = Eigen::Matrix<double, 6, 6>::Identity();

        x_order.push_back(state->_calib_IMUtoCAM.at(line.anchor_cam_id));
        H_x.push_back(d_Lc_d_calib);
    }
*/
    Eigen::Vector3d e1_v, e2_v;
    e1_v << 1, 0, 0;
    e2_v << 0, 1, 0;

    Eigen::Matrix<double, 6, 4> d_LG_d_theta_dl;
    Eigen::Matrix<double, 6, 1> plk;
    plk << n_einG(0), n_einG(1), n_einG(2), v_einG(0), v_einG(1), v_einG(2);
    d_LG_d_theta_dl.block(0, 0, 3, 3) = d_linG * skew_x(plk_2_rot(plk) * e1_v);
    d_LG_d_theta_dl.block(0, 3, 3, 1) = plk_2_rot(plk) * e1_v;
    d_LG_d_theta_dl.block(3, 0, 3, 3) = skew_x(plk_2_rot(plk) * e2_v);
    d_LG_d_theta_dl.block(3, 3, 3, 1) = Eigen::MatrixXd::Zero(3, 1);

    Eigen::Matrix<double, 4, 4> d_theta_dl_d_plinG;
    Eigen::Matrix<double, 4, 1> quat = rot_2_quat(plk_2_rot(plk));
    d_theta_dl_d_plinG.block(0, 0, 3, 3) = 2/d_linG*(quat(3) * Eigen::Matrix<double, 3, 3>::Identity() - skew_x(quat.block(0, 0, 3, 1)));
    d_theta_dl_d_plinG.block(3, 0, 1, 3) = quat.block(0, 0, 3, 1).transpose();
    d_theta_dl_d_plinG.block(0, 3, 3, 1) = -2 / d_linG * quat.block(0, 0, 3, 1);
    d_theta_dl_d_plinG(3, 3) = quat(3);

    H_f = d_LG_d_theta_dl * d_theta_dl_d_plinG;
    return;
}

void LineUpdaterHelper::get_line_jacobian_full(std::shared_ptr<State> state, UpdaterHelperLine &line, Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res, std::vector<std::shared_ptr<ov_type::Type>> &x_order) {
    

std::ofstream line_debug;
line_debug.open("/home/zhangyanyu/catkin_ws_ov/src/open_vins/Debug/line_debug.txt", std::ofstream::app);

    // Total number of measurements for this line
    int total_meas = 0;
    for (auto const &pair : line.timestamps) {
        total_meas += (int)pair.second.size();
    }
    
    // Compute the size of the states involved with this feature
    int total_hx = 0;
    std::unordered_map<std::shared_ptr<Type>, size_t> map_hx;
    for (auto const &pair : line.timestamps) {

        // Our extrinsics and intrinsics
        std::shared_ptr<PoseJPL> calibration = state->_calib_IMUtoCAM.at(pair.first);
        // std::shared_ptr<Vec> distortion = state->_cam_intrinsics.at(pair.first);

        // If doing calibration extrinsics
        if (state->_options.do_calib_camera_pose) {
            map_hx.insert({calibration, total_hx});
            x_order.push_back(calibration);
            total_hx += calibration->size();
        }

        // If doing calibration intrinsics
        /*
        if (state->_options.do_calib_camera_intrinsics) {
            map_hx.insert({distortion, total_hx});
            x_order.push_back(distortion);
            total_hx += distortion->size();
        }
        */

        // Loop through all measurements for this specific camera
        for (size_t m = 0; m < line.timestamps[pair.first].size(); m++) {

            // Add this clone if it is not added already
            std::shared_ptr<PoseJPL> clone_Ci = state->_clones_IMU.at(line.timestamps[pair.first].at(m));

            if (map_hx.find(clone_Ci) == map_hx.end()) {
                map_hx.insert({clone_Ci, total_hx});
                x_order.push_back(clone_Ci);
                total_hx += clone_Ci->size();
            }
        }
    }

    /*
    // If we are using an anchored representation, make sure that the anchor is also added
    if (Line_Landmark_Rep::is_relative_representation(line.line_representation)) {

        // Assert we have a clone
        assert(line.anchor_cam_id != -1);

        // Add this anchor if it is not added already
        std::shared_ptr<PoseJPL> clone_Ai = state->_clones_IMU.at(line.anchor_clone_timestamp);
        if (map_hx.find(clone_Ai) == map_hx.end()) {
            map_hx.insert({clone_Ai, total_hx});
            x_order.push_back(clone_Ai);
            total_hx += clone_Ai->size();
        }

        // Also add its calibration if we are doing calibration
        if (state->_options.do_calib_camera_pose) {
            // Add this anchor if it is not added already
            std::shared_ptr<PoseJPL> clone_calib = state->_calib_IMUtoCAM.at(line.anchor_cam_id);
            if (map_hx.find(clone_calib) == map_hx.end()) {
                map_hx.insert({clone_calib, total_hx});
                x_order.push_back(clone_calib);
                total_hx += clone_calib->size();
            }
        }
    }
    */
    //=========================================================================
    //=========================================================================

    // Calculate the position of this feature in the global frame
    // If anchored, then we need to calculate the position of the feature in the global
    Eigen::Vector3d n_einG = cp_2_rot(line.p_LinG).block(0, 0, 3, 1);
    Eigen::Vector3d v_einG = cp_2_rot(line.p_LinG).block(0, 1, 3, 1);
    double d_linG = cp_2_depth(line.p_LinG);

line_debug << "[Starting] ID: " << " -> " << int(line.lineid) << "\n";
line_debug << "[Starting] n_einG: " << " -> (" << n_einG(0)<<" "<<n_einG(1)<<" "<<n_einG(2)<<")\n";
line_debug << "[Starting] v_einG: " << " -> (" << v_einG(0)<<" "<<v_einG(1)<<" "<<v_einG(2)<<")\n";
line_debug << "[Starting] d_linG: " << " -> " << d_linG << "\n";
    /*
    if (Line_Landmark_Rep::is_relative_representation(line.line_representation)) {
        // Assert that we have an anchor pose for this line
        assert(line.anchor_cam_id != -1);
        // Get calibration for our anchor camera
        Eigen::Matrix3d R_ItoC = state->_calib_IMUtoCAM.at(line.anchor_cam_id)->Rot();
        Eigen::Vector3d p_IinC = state->_calib_IMUtoCAM.at(line.anchor_cam_id)->pos();
        // Anchor pose orientation and position
        Eigen::Matrix3d R_GtoI = state->_clones_IMU.at(line.anchor_clone_timestamp)->Rot();
        Eigen::Vector3d p_IinG = state->_clones_IMU.at(line.anchor_clone_timestamp)->pos();
        // Feature in the global frame

        n_einG = R_GtoI.transpose() * R_ItoC.transpose() * (n_einA - p_IinG) + p_IinC;
        v_einG = R_GtoI.transpose() * R_ItoC.transpose() * (v_einA - p_IinG) + p_IinC;
    }
    */
    // Calculate the position of this feature in the global frame FEJ
    // If anchored, then we can use the "best" p_FinG since the value of p_FinA does not matter

    Eigen::Vector3d n_einG_fej = cp_2_rot(line.p_LinG_fej).block(0, 0, 3, 1);
    Eigen::Vector3d v_einG_fej = cp_2_rot(line.p_LinG_fej).block(0, 1, 3, 1);
    double d_linG_fej = cp_2_depth(line.p_LinG_fej);
    
    /*
    if (Line_Landmark_Rep::is_relative_representation(line.line_representation)) {
        Eigen::Vector3d n_einG_fej = n_einG;
        Eigen::Vector3d v_einG_fej = v_einG;
    }
    */
    //=========================================================================
    //=========================================================================

    // Allocate our residual and Jacobians
    int c = 0;
    int jacobsize = 4;
    res = Eigen::VectorXd::Zero(2 * total_meas);
    H_f = Eigen::MatrixXd::Zero(2 * total_meas, jacobsize);
    H_x = Eigen::MatrixXd::Zero(2 * total_meas, total_hx);

    // Derivative of p_FinG in respect to feature representation.
    // This only needs to be computed once and thus we pull it out of the loop
    Eigen::MatrixXd d_Lg_d_plinG;
    std::vector<Eigen::MatrixXd> d_Li_dx;
    std::vector<std::shared_ptr<Type>> d_Li_dx_order;
    LineUpdaterHelper::get_line_jacobian_representation(state, line, d_Lg_d_plinG, d_Li_dx, d_Li_dx_order);

    // Assert that all the ones in our order are already in our local jacobian mapping
    //for (auto &type : d_Li_dx_order) {
        //assert(map_hx.find(type) != map_hx.end());
    //}
    
    // Loop through each camera for this feature
    for (auto const &pair : line.timestamps) {

        // Our calibration between the IMU and CAMi frames
        std::shared_ptr<Vec> distortion = state->_cam_intrinsics.at(pair.first);
        std::shared_ptr<PoseJPL> calibration = state->_calib_IMUtoCAM.at(pair.first);
        Eigen::Matrix3d R_ItoC = calibration->Rot();
        Eigen::Vector3d p_IinC = calibration->pos();

      	double fu = distortion->value()(0);
      	double fv = distortion->value()(1);
       	double cu = distortion->value()(2);
      	double cv = distortion->value()(3);
        Eigen::Matrix3d K;
      	K << fv, 0, 0, 0, fu, 0, -fv*cu, -fu*cv, fu*fv;
line_debug << "[Starting] distortion: " << " -> (" << fu<<", "<<fv<<", "<<cu<< ", " << cv << ")\n";        
        Eigen::Matrix<double, 3, 6> d_I_d_Lc;
       	d_I_d_Lc.block(0, 0, 3, 3) = K;
       	d_I_d_Lc.block(0, 3, 3, 3) = Eigen::MatrixXd::Zero(3, 3);
        
        // Loop through all measurements for this specific camera
        for (size_t m = 0; m < line.timestamps[pair.first].size(); m++) {
        //=========================================================================
        //=========================================================================

            // Get current IMU clone state
            std::shared_ptr<PoseJPL> clone_Ii = state->_clones_IMU.at(line.timestamps[pair.first].at(m));
            Eigen::Matrix3d R_GtoIi = clone_Ii->Rot();
            Eigen::Vector3d p_IiinG = clone_Ii->pos();

            // Get current feature in the IMU
            Eigen::Vector3d n_einIi_tmp = R_GtoIi * d_linG * n_einG - R_GtoIi * skew_x(p_IiinG) * v_einG;
            Eigen::Vector3d v_einIi = R_GtoIi * v_einG;

            Eigen::Vector3d n_einIi = n_einIi_tmp / n_einIi_tmp.norm();
            double d_linIi = n_einIi_tmp.norm();
line_debug << "[Starting] n_einIi: " << " -> (" << n_einIi(0)<<" "<<n_einIi(1)<<" "<<n_einIi(2)<<")\n";
line_debug << "[Starting] v_einIi: " << " -> (" << v_einIi(0)<<" "<<v_einIi(1)<<" "<<v_einIi(2)<<")\n";
line_debug << "[Starting] d_einIi: " << " -> " << d_linIi << "\n";

            // Project the current feature into the current frame of reference
            Eigen::Vector3d n_einCi_tmp = R_ItoC * n_einIi_tmp + skew_x(p_IinC) * R_ItoC * v_einIi;
            Eigen::Vector3d n_einCi = n_einCi_tmp / n_einCi_tmp.norm();
            Eigen::Vector3d v_einCi = R_ItoC * v_einIi;
            double d_linCi = n_einCi_tmp.norm();

line_debug << "[Starting] n_einCi: " << " -> (" << n_einCi(0)<<" "<<n_einCi(1)<<" "<<n_einCi(2)<<")\n";
line_debug << "[Starting] v_einCi: " << " -> (" << v_einCi(0)<<" "<<v_einCi(1)<<" "<<v_einCi(2)<<")\n";
line_debug << "[Starting] d_linCi: " << " -> " << d_linCi << "\n";
            // If we are doing first estimate Jacobians, then overwrite with the first estimates
            if (state->_options.do_fej) {
                R_GtoIi = clone_Ii->Rot_fej();
                p_IiinG = clone_Ii->pos_fej();

                n_einIi_tmp = R_GtoIi * d_linG_fej * n_einG_fej - R_GtoIi * skew_x(p_IiinG) * v_einG_fej;
                n_einIi = n_einIi_tmp / n_einIi_tmp.norm();
                v_einIi = R_GtoIi * v_einG_fej;
                d_linIi = n_einIi_tmp.norm();
                
                n_einCi_tmp = R_ItoC * d_linIi * n_einIi + skew_x(p_IinC) * R_ItoC * v_einIi;
                n_einCi = n_einCi_tmp / n_einCi_tmp.norm();
                v_einCi = R_ItoC * v_einIi;
                d_linCi = n_einCi_tmp.norm();
            }

            Eigen::Matrix<double, 6, 6> d_Li_d_Lg;
            d_Li_d_Lg.block(0, 0, 3, 3) = R_GtoIi;
   	    d_Li_d_Lg.block(3, 0, 3, 3) = Eigen::MatrixXd::Zero(3, 3);
   	    d_Li_d_Lg.block(0, 3, 3, 3) = -R_GtoIi * skew_x(p_IiinG);
   	    d_Li_d_Lg.block(3, 3, 3, 3) = R_GtoIi;

            Eigen::Matrix<double, 6, 1> L_C;
            L_C.block(0, 0, 3, 1) = d_linCi * n_einCi;
            L_C.block(3, 0, 3, 1) = v_einCi;

            Eigen::Matrix<double, 3, 1> l_2d = d_I_d_Lc * L_C; 
line_debug << "[Starting] l_2d: " << " -> (" << l_2d(0)<<" "<<l_2d(1)<<" "<<l_2d(2)<<")\n";
            // Our residual
            Eigen::Matrix<double, 3, 1> startpoint_m, endpoint_m;

            startpoint_m << (double)line.startpoint[pair.first].at(m)(0), (double)line.startpoint[pair.first].at(m)(1), 1;
            endpoint_m << (double)line.endpoint[pair.first].at(m)(0), (double)line.endpoint[pair.first].at(m)(1), 1;
line_debug << "[Starting] startpoint_m: " << " -> (" << startpoint_m(0)<<" "<<startpoint_m(1)<<" "<<startpoint_m(2)<<")\n";
line_debug << "[Starting] endpoint_m: " << " -> (" << endpoint_m(0)<<" "<<endpoint_m(1)<<" "<<endpoint_m(2)<<")\n";
            Eigen::Vector2d res_c;
            double ln = sqrt(l_2d[0]*l_2d[0] + l_2d[1]*l_2d[1]);
  
            res_c << startpoint_m.transpose()*l_2d / ln, endpoint_m.transpose()*l_2d / ln;
      
            res.block(2 * c, 0, 2, 1) = res_c;

line_debug << "[Starting] res_c: " << " -> (" << res_c[0]<<" "<<res_c[1]<<")\n";
            //=========================================================================
            //=========================================================================

            
            Eigen::Matrix<double, 2, 3> d_zl_d_I;
            double e1 = l_2d.transpose() * startpoint_m;
            double e2 = l_2d.transpose() * endpoint_m;
            
            d_zl_d_I << (startpoint_m[0] - l_2d[0]*e1/(ln*ln))/ln, (startpoint_m[1] - l_2d[1]*e1/(ln*ln))/ln, 1/ln, (endpoint_m[0] - l_2d[0]*e2/(ln*ln))/ln, (endpoint_m[1] - l_2d[1]*e2/(ln*ln))/ln, 1/ln;
        
            Eigen::Matrix<double, 6, 6> d_Lc_d_Li;
       	    d_Lc_d_Li.block(0, 0, 3, 3) = R_ItoC;
            d_Lc_d_Li.block(3, 0, 3, 3) = Eigen::MatrixXd::Zero(3, 3);
            d_Lc_d_Li.block(0, 3, 3, 3) = skew_x(p_IinC) * R_ItoC;
            d_Lc_d_Li.block(3, 3, 3, 3) = R_ItoC;

            // 2x3, 3x6, 6x6, 6x4, 4x4 = 2x4
            H_f.block(2 * c, 0, 2, H_f.cols()).noalias() = d_zl_d_I * d_I_d_Lc * d_Lc_d_Li * d_Li_d_Lg * d_Lg_d_plinG;

            Eigen::Matrix<double, 6, 3> d_Li_d_theta;
            d_Li_d_theta.block(0, 0, 3, 3) = d_linG * skew_x(R_GtoIi * n_einG) - skew_x(R_GtoIi * skew_x(p_IiinG) * v_einG);
            d_Li_d_theta.block(3, 0, 3, 3) = skew_x(R_GtoIi * v_einG);

            Eigen::Matrix<double, 6, 3> d_Li_d_pIinG;
            d_Li_d_pIinG.block(0, 0, 3, 3) = R_GtoIi * skew_x(v_einG);
            d_Li_d_pIinG.block(3, 0, 3, 3) = Eigen::MatrixXd::Zero(3, 3);

            Eigen::Matrix<double, 6, 6> d_Lc_d_clone;
            d_Lc_d_clone.block(0, 0, 6, 3) = d_Lc_d_Li * d_Li_d_theta;
            d_Lc_d_clone.block(0, 3, 6, 3) = d_Lc_d_Li * d_Li_d_pIinG;

            // 2x6
            H_x.block(2 * c, map_hx[clone_Ii], 2, clone_Ii->size()).noalias() = d_zl_d_I * d_I_d_Lc * d_Lc_d_clone;

            // =========================================================================
            /*
            for (size_t i = 0; i < d_Li_dx_order.size(); i++) {
                H_x.block(2 * c, map_hx[d_Li_dx_order.at(i)], 2, d_Li_dx_order.at(i)->size()).noalias() += d_zl_d_I * d_I_d_Lc * d_Lc_d_Li * d_Li_dx.at(i);
            }
            */
            //=========================================================================
            //=========================================================================

            // Derivative of p_FinCi in respect to camera calibration (R_ItoC, p_IinC)
            if (state->_options.do_calib_camera_pose) {
                
                Eigen::Matrix<double, 6, 6> d_Lc_d_calib;
   	     	d_Lc_d_calib.block(0, 0, 3, 3) = d_linIi * skew_x(R_ItoC * n_einIi) + skew_x(p_IinC) * skew_x(R_ItoC * v_einIi);
     	   	d_Lc_d_calib.block(3, 0, 3, 3) = skew_x(R_ItoC * v_einIi);
     	   	d_Lc_d_calib.block(0, 3, 3, 3) = -skew_x(R_ItoC * v_einIi);
        	d_Lc_d_calib.block(3, 3, 3, 3) = Eigen::MatrixXd::Zero(3, 3);
                
                Eigen::Matrix<double, 2, 6> dz_dcalib = d_zl_d_I * d_I_d_Lc * d_Lc_d_calib;

                // Chainrule it and add it to the big jacobian
                H_x.block(2 * c, map_hx[calibration], 2, calibration->size()).noalias() += dz_dcalib;
            }      

            // Move the Jacobian and residual index forward
            c++;
        }
    }
line_debug.close();
}


void LineUpdaterHelper::nullspace_project_inplace(Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res) {

  // Apply the left nullspace of H_f to all variables
  // Based on "Matrix Computations 4th Edition by Golub and Van Loan"
  // See page 252, Algorithm 5.2.4 for how these two loops work
  // They use "matlab" index notation, thus we need to subtract 1 from all index
  Eigen::JacobiRotation<double> tempHo_GR;
  for (int n = 0; n < H_f.cols(); ++n) {
    for (int m = (int)H_f.rows() - 1; m > n; m--) {
      // Givens matrix G
      tempHo_GR.makeGivens(H_f(m - 1, n), H_f(m, n));
      // Multiply G to the corresponding lines (m-1,m) in each matrix
      // Note: we only apply G to the nonzero cols [n:Ho.cols()-n-1], while
      //       it is equivalent to applying G to the entire cols [0:Ho.cols()-1].
      (H_f.block(m - 1, n, 2, H_f.cols() - n)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      (H_x.block(m - 1, 0, 2, H_x.cols())).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      (res.block(m - 1, 0, 2, 1)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
    }
  }

  // The H_f jacobian max rank is 3 if it is a 3d position, thus size of the left nullspace is Hf.rows()-3
  // NOTE: need to eigen3 eval here since this experiences aliasing!
  // H_f = H_f.block(H_f.cols(),0,H_f.rows()-H_f.cols(),H_f.cols()).eval();
  H_x = H_x.block(H_f.cols(), 0, H_x.rows() - H_f.cols(), H_x.cols()).eval();
  res = res.block(H_f.cols(), 0, res.rows() - H_f.cols(), res.cols()).eval();

  // Sanity check
  assert(H_x.rows() == res.rows());
}


void LineUpdaterHelper::measurement_compress_inplace(Eigen::MatrixXd &H_x, Eigen::VectorXd &res) {

  // Return if H_x is a fat matrix (there is no need to compress in this case)
  if (H_x.rows() <= H_x.cols())
    return;

  // Do measurement compression through givens rotations
  // Based on "Matrix Computations 4th Edition by Golub and Van Loan"
  // See page 252, Algorithm 5.2.4 for how these two loops work
  // They use "matlab" index notation, thus we need to subtract 1 from all index
  Eigen::JacobiRotation<double> tempHo_GR;
  for (int n = 0; n < H_x.cols(); n++) {
    for (int m = (int)H_x.rows() - 1; m > n; m--) {
      // Givens matrix G
      tempHo_GR.makeGivens(H_x(m - 1, n), H_x(m, n));
      // Multiply G to the corresponding lines (m-1,m) in each matrix
      // Note: we only apply G to the nonzero cols [n:Ho.cols()-n-1], while
      //       it is equivalent to applying G to the entire cols [0:Ho.cols()-1].
      (H_x.block(m - 1, n, 2, H_x.cols() - n)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      (res.block(m - 1, 0, 2, 1)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
    }
  }

  // If H is a fat matrix, then use the rows
  // Else it should be same size as our state
  int r = std::min(H_x.rows(), H_x.cols());

  // Construct the smaller jacobian and residual after measurement compression
  assert(r <= H_x.rows());
  H_x.conservativeResize(r, H_x.cols());
  res.conservativeResize(r, res.cols());
}

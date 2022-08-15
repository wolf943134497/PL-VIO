/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2022 Patrick Geneva
 * Copyright (C) 2018-2022 Guoquan Huang
 * Copyright (C) 2018-2022 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "UpdaterMSCKF.h"
#include <fstream>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

void UpdaterMSCKF::update(std::shared_ptr<State> state, std::vector<std::shared_ptr<Feature>> &feature_vec, std::vector<std::shared_ptr<Line>> &line_vec) {

    // Return if no features
    if (feature_vec.empty())
        return;

    if (_use_line && line_vec.empty())
        return;

    // Start timing
    boost::posix_time::ptime rT0, rT1, rT2, rT3, rT4, rT5;
    rT0 = boost::posix_time::microsec_clock::local_time();

    // 0. Get all timestamps our clones are at (and thus valid measurement times)
    std::vector<double> clonetimes;
    for (const auto &clone_imu : state->_clones_IMU) {
        clonetimes.emplace_back(clone_imu.first);
    }

    // 1. Clean all feature measurements and make sure they all have valid clone times
    auto it0 = feature_vec.begin();
    while (it0 != feature_vec.end()) {

        // Clean the feature
        (*it0)->clean_old_measurements(clonetimes);

        // Count how many measurements
        int ct_meas = 0;
        for (const auto &pair : (*it0)->timestamps) {
            ct_meas += (*it0)->timestamps[pair.first].size();
        }

        // Remove if we don't have enough
        if (ct_meas < 2) {
            (*it0)->to_delete = true;
            it0 = feature_vec.erase(it0);
        } else {
            it0++;
        }
    }
    
    auto it00 = line_vec.begin();
    while (it00 != line_vec.end()) {

        // Clean the feature
        (*it00)->clean_old_measurements(clonetimes);

        // Count how many measurements
        int line_ct_meas = 0;
        for (const auto &line_pair : (*it00)->timestamps) {
            line_ct_meas += (*it00)->timestamps[line_pair.first].size();
        }

        // Remove if we don't have enough
        if (line_ct_meas < 2) {
            (*it00)->to_delete = true;
            it00 = line_vec.erase(it00);
        } else {
            it00++;
        }
    }

    rT1 = boost::posix_time::microsec_clock::local_time();

    // 2. Create vector of cloned *CAMERA* poses at each of our clone timesteps
    std::unordered_map<size_t, std::unordered_map<double, FeatureInitializer::ClonePose>> clones_cam;

    std::unordered_map<size_t, std::unordered_map<double, LineInitializer::ClonePose>> clones_cam_line;

    for (const auto &clone_calib : state->_calib_IMUtoCAM) {

        // For this camera, create the vector of camera poses
        std::unordered_map<double, FeatureInitializer::ClonePose> clones_cami;
        
        std::unordered_map<double, LineInitializer::ClonePose> clones_cami_line;

        for (const auto &clone_imu : state->_clones_IMU) {

            // Get current camera pose
            Eigen::Matrix<double, 3, 3> R_GtoCi = clone_calib.second->Rot() * clone_imu.second->Rot();
            Eigen::Matrix<double, 3, 1> p_CioinG = clone_imu.second->pos() - R_GtoCi.transpose() * clone_calib.second->pos();

            // Append to our map
            clones_cami.insert({clone_imu.first, FeatureInitializer::ClonePose(R_GtoCi, p_CioinG)});

            clones_cami_line.insert({clone_imu.first, LineInitializer::ClonePose(R_GtoCi, p_CioinG)});
        }

        // Append to our map
        clones_cam.insert({clone_calib.first, clones_cami});

        clones_cam_line.insert({clone_calib.first, clones_cami_line});
    }

    // 3. Try to triangulate all MSCKF or new SLAM features that have measurements
    std::ofstream myfile3;
    myfile3.open("/home/zhangyanyu/catkin_ws_ov/src/open_vins/Debug/sim_feat_est.txt", std::ofstream::app);

    auto it1 = feature_vec.begin();
    while (it1 != feature_vec.end()) {

        // Triangulate the feature and remove if it fails
        bool success_tri = true;
        if (initializer_feat->config().triangulate_1d) {
            success_tri = initializer_feat->single_triangulation_1d(it1->get(), clones_cam);
        } else {
            success_tri = initializer_feat->single_triangulation(it1->get(), clones_cam);
        }

        // Gauss-newton refine the feature
        bool success_refine = true;
        if (initializer_feat->config().refine_features) {
            success_refine = initializer_feat->single_gaussnewton(it1->get(), clones_cam);
        }

        // Remove the feature if not a success
        if (!success_tri || !success_refine) {
            (*it1)->to_delete = true;
            it1 = feature_vec.erase(it1);
            continue;
        }

        myfile3 << "[Estimation] feature ID: " << (int(it1->get()->featid)-4096) << " -> (" <<it1->get()->p_FinG(0)<<" "<<it1->get()->p_FinG(1)<<" "<<it1->get()->p_FinG(2)<<")\n";

        it1++;
    }
    myfile3.close();


    //std::ofstream myfile4;
    //myfile4.open("/home/zhangyanyu/catkin_ws_ov/src/open_vins/Debug/sim_line_est.txt", std::ofstream::app);

    auto it11 = line_vec.begin();
    while (it11 != line_vec.end()) {

        // Triangulate the feature and remove if it fails
        bool success_tri_line = true;

        success_tri_line = initializer_line->line_triangulation(it11->get(), clones_cam_line);

        // Remove the feature if not a success
        double cur_depth = cp_2_depth(it11->get()->p_LinG);
        if (!success_tri_line) {
            (*it11)->to_delete = true;
            it11 = line_vec.erase(it11);
            continue;
        }
        /*
        // For Debug
        Eigen::Matrix<double,6,1> cur_plk = rot_to_plk(cp_2_rot(it11->get()->p_LinG), cur_depth);
        Eigen::Matrix<double,3,1> n_e1_G = cur_plk.block(0, 0, 3, 1)/(cur_plk.block(0, 0, 3, 1).norm());
        Eigen::Matrix<double,3,1> v_e1_G = cur_plk.block(3, 0, 3, 1)/(cur_plk.block(3, 0, 3, 1).norm());

        myfile4 << "[Estimation] n_e1_in_G: " << int(it11->get()->lineid) << " -> "<<n_e1_G(0)<<" "<<n_e1_G(1)<<" "<<n_e1_G(2)<<"\n";
        myfile4 << "[Estimation] v_e1_in_G: " << int(it11->get()->lineid) << " -> "<<v_e1_G(0)<<" "<<v_e1_G(1)<<" "<<v_e1_G(2)<<"\n";
        myfile4 << "[Estimation] line ID: " << int(it11->get()->lineid) << " -> (" <<(it11->get()->p_LinG)(0)<<" "<<(it11->get()->p_LinG)(1)<<" "<<(it11->get()->p_LinG)(2)<<" "<<(it11->get()->p_LinG)(3)<<")" << " line depth -> " << cur_depth << "\n";
        */
        it11++;
    }
    //myfile4.close();
    

    rT2 = boost::posix_time::microsec_clock::local_time();

    // Calculate the max possible measurement size
    size_t max_meas_size = 0;
    for (size_t i = 0; i < feature_vec.size(); i++) {
        for (const auto &pair : feature_vec.at(i)->timestamps) {
            max_meas_size += 2 * feature_vec.at(i)->timestamps[pair.first].size();
        }
    }

    size_t max_line_meas_size = 0;
    for (size_t i = 0; i < line_vec.size(); i++) {
        for (const auto &line_pair : line_vec.at(i)->timestamps) {
            max_line_meas_size += 2 * line_vec.at(i)->timestamps[line_pair.first].size();
        }
    }

    // Calculate max possible state size (i.e. the size of our covariance)
    // NOTE: that when we have the single inverse depth representations, those are only 1dof in size
    size_t max_hx_size = state->max_covariance_size();
    for (auto &landmark : state->_features_SLAM) {
        max_hx_size -= landmark.second->size();
    }
    /*
    size_t max_hx_line_size = state->max_covariance_size();
    for (auto &line_landmark : state->_lines_SLAM) {
        max_hx_line_size -= line_landmark.second->size();
    }  
    */
    
    // Large Jacobian and residual of *all* features for this update
    Eigen::VectorXd res_big = Eigen::VectorXd::Zero(max_meas_size);
    Eigen::MatrixXd Hx_big = Eigen::MatrixXd::Zero(max_meas_size, max_hx_size);
    std::unordered_map<std::shared_ptr<Type>, size_t> Hx_mapping;
    std::vector<std::shared_ptr<Type>> Hx_order_big;
    size_t ct_jacob = 0;
    size_t ct_meas = 0;


    Eigen::VectorXd line_res_big = Eigen::VectorXd::Zero(max_line_meas_size);
    Eigen::MatrixXd line_Hx_big = Eigen::MatrixXd::Zero(max_line_meas_size, max_hx_size);
    std::unordered_map<std::shared_ptr<Type>, size_t> line_Hx_mapping;
    std::vector<std::shared_ptr<Type>> line_Hx_order_big;
    size_t line_ct_jacob = 0;
    size_t line_ct_meas = 0;


    // 4. Compute linear system for each feature, nullspace project, and reject
    auto it2 = feature_vec.begin();
    while (it2 != feature_vec.end()) {

        // Convert our feature into our current format
        UpdaterHelper::UpdaterHelperFeature feat;
        feat.featid = (*it2)->featid;
        feat.uvs = (*it2)->uvs;
        feat.uvs_norm = (*it2)->uvs_norm;
        feat.timestamps = (*it2)->timestamps;

        // If we are using single inverse depth, then it is equivalent to using the msckf inverse depth
        feat.feat_representation = state->_options.feat_rep_msckf;
        if (state->_options.feat_rep_msckf == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) {
            feat.feat_representation = LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH;
        }

        // Save the position and its fej value
        if (LandmarkRepresentation::is_relative_representation(feat.feat_representation)) {
            feat.anchor_cam_id = (*it2)->anchor_cam_id;
            feat.anchor_clone_timestamp = (*it2)->anchor_clone_timestamp;
            feat.p_FinA = (*it2)->p_FinA;
            feat.p_FinA_fej = (*it2)->p_FinA;
        } else {
            feat.p_FinG = (*it2)->p_FinG;
            feat.p_FinG_fej = (*it2)->p_FinG;
        }

        // Our return values (feature jacobian, state jacobian, residual, and order of state jacobian)
        Eigen::MatrixXd H_f;
        Eigen::MatrixXd H_x;
        Eigen::VectorXd res;
        std::vector<std::shared_ptr<Type>> Hx_order;

        // Get the Jacobian for this feature
        UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, H_x, res, Hx_order);

        // Nullspace project
        UpdaterHelper::nullspace_project_inplace(H_f, H_x, res);

        /// Chi2 distance check
        Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
        Eigen::MatrixXd S = H_x * P_marg * H_x.transpose();
        S.diagonal() += _options.sigma_pix_sq * Eigen::VectorXd::Ones(S.rows());
        double chi2 = res.dot(S.llt().solve(res));

        // Get our threshold (we precompute up to 500 but handle the case that it is more)
        double chi2_check;
        if (res.rows() < 500) {
            chi2_check = chi_squared_table[res.rows()];
        } else {
            boost::math::chi_squared chi_squared_dist(res.rows());
            chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
            PRINT_WARNING(YELLOW "chi2_check over the residual limit - %d\n" RESET, (int)res.rows());
        }

        // Check if we should delete or not
        if (chi2 > _options.chi2_multipler * chi2_check) {
            (*it2)->to_delete = true;
            it2 = feature_vec.erase(it2);
            // PRINT_DEBUG("featid = %d\n", feat.featid);
            // PRINT_DEBUG("chi2 = %f > %f\n", chi2, _options.chi2_multipler*chi2_check);
            // std::stringstream ss;
            // ss << "res = " << std::endl << res.transpose() << std::endl;
            // PRINT_DEBUG(ss.str().c_str());
            continue;
        }

        // We are good!!! Append to our large H vector
        size_t ct_hx = 0;
        for (const auto &var : Hx_order) {

            // Ensure that this variable is in our Jacobian
            if (Hx_mapping.find(var) == Hx_mapping.end()) {
                Hx_mapping.insert({var, ct_jacob});
                Hx_order_big.push_back(var);
                ct_jacob += var->size();
            }

            // Append to our large Jacobian
            Hx_big.block(ct_meas, Hx_mapping[var], H_x.rows(), var->size()) = H_x.block(0, ct_hx, H_x.rows(), var->size());
            ct_hx += var->size();
        }

        // Append our residual and move forward
        res_big.block(ct_meas, 0, res.rows(), 1) = res;
        ct_meas += res.rows();
        it2++;
    }


    auto it22 = line_vec.begin();
    while (it22 != line_vec.end()) {

        // Convert our feature into our current format
        LineUpdaterHelper::UpdaterHelperLine line;
        line.lineid = (*it22)->lineid;
        line.startpoint = (*it22)->startpoint;
        line.endpoint = (*it22)->endpoint;
        line.startpoint_norm = (*it22)->startpoint_norm;
        line.endpoint_norm = (*it22)->endpoint_norm;
        line.timestamps = (*it22)->timestamps;

        line.line_representation = state->_options.line_rep_msckf;
        //line.anchor_cam_id = (*it22)->anchor_cam_id;
        //line.anchor_clone_timestamp = (*it22)->anchor_clone_timestamp;
        //line.p_LinA = (*it22)->p_LinA;
        //line.p_LinA_fej = (*it22)->p_LinA;
        line.p_LinG = (*it22)->p_LinG;
        line.p_LinG_fej = (*it22)->p_LinG;

        // Our return values (feature jacobian, state jacobian, residual, and order of state jacobian)
        Eigen::MatrixXd line_H_f;
        Eigen::MatrixXd line_H_x;
        Eigen::VectorXd line_res;
        std::vector<std::shared_ptr<Type>> line_Hx_order;

        // Get the Jacobian for this feature
        LineUpdaterHelper::get_line_jacobian_full(state, line, line_H_f, line_H_x, line_res, line_Hx_order);

        // Nullspace project
        LineUpdaterHelper::nullspace_project_inplace(line_H_f, line_H_x, line_res);

        /// Chi2 distance check
        Eigen::MatrixXd line_P_marg = StateHelper::get_marginal_covariance(state, line_Hx_order);
        Eigen::MatrixXd line_S = line_H_x * line_P_marg * line_H_x.transpose();
        line_S.diagonal() += _options.sigma_pix_sq * Eigen::VectorXd::Ones(line_S.rows());
        double line_chi2 = line_res.dot(line_S.llt().solve(line_res));

        // Get our threshold (we precompute up to 500 but handle the case that it is more)
        double line_chi2_check;
        if (line_res.rows() < 250) {
            line_chi2_check = chi_squared_table[line_res.rows()];
        } else {
            boost::math::chi_squared chi_squared_dist(line_res.rows());
            line_chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
            PRINT_WARNING(YELLOW "chi2_check over the residual limit - %d\n" RESET, (int)line_res.rows());
        }

        // Check if we should delete or not
        if (line_chi2 > _options.chi2_multipler * line_chi2_check) {
            (*it22)->to_delete = true;
            it22 = line_vec.erase(it22);
            // PRINT_DEBUG("featid = %d\n", feat.featid);
            // PRINT_DEBUG("chi2 = %f > %f\n", chi2, _options.chi2_multipler*chi2_check);
            // std::stringstream ss;
            // ss << "res = " << std::endl << res.transpose() << std::endl;
            // PRINT_DEBUG(ss.str().c_str());
            continue;
        }

        // We are good!!! Append to our large H vector
        size_t line_ct_hx = 0;
        for (const auto &line_var : line_Hx_order) {

            // Ensure that this variable is in our Jacobian
            if (line_Hx_mapping.find(line_var) == line_Hx_mapping.end()) {
                line_Hx_mapping.insert({line_var, line_ct_jacob});
                line_Hx_order_big.push_back(line_var);
                line_ct_jacob += line_var->size();
            }

            // Append to our large Jacobian
            line_Hx_big.block(line_ct_meas, line_Hx_mapping[line_var], line_H_x.rows(), line_var->size()) = line_H_x.block(0, line_ct_hx, line_H_x.rows(), line_var->size());
            line_ct_hx += line_var->size();
        }

        // Append our residual and move forward
        line_res_big.block(line_ct_meas, 0, line_res.rows(), 1) = line_res;
        line_ct_meas += line_res.rows();
        it22++;
    }

    
    rT3 = boost::posix_time::microsec_clock::local_time();

    // We have appended all features to our Hx_big, res_big
    // Delete it so we do not reuse information
    for (size_t f = 0; f < feature_vec.size(); f++) {
        feature_vec[f]->to_delete = true;
    }

    for (size_t ff = 0; ff < line_vec.size(); ff++) {
        line_vec[ff]->to_delete = true;
    }
    

    // Return if we don't have anything and resize our matrices
    if (ct_meas < 1) {
        return;
    }

    if (_use_line && line_ct_meas < 1) {
        return;
    }

    assert(ct_meas <= max_meas_size);
    assert(ct_jacob <= max_hx_size);
    res_big.conservativeResize(ct_meas, 1);
    Hx_big.conservativeResize(ct_meas, ct_jacob);

    assert(line_ct_meas <= max_line_meas_size);
    assert(line_ct_jacob <= max_hx_size);
    line_res_big.conservativeResize(line_ct_meas, 1);
    line_Hx_big.conservativeResize(line_ct_meas, line_ct_jacob);

    // 5. Perform measurement compression
    UpdaterHelper::measurement_compress_inplace(Hx_big, res_big);
    
    LineUpdaterHelper::measurement_compress_inplace(line_Hx_big, line_res_big);

    if (Hx_big.rows() < 1) {
        return;
    }


    if (_use_line && line_Hx_big.rows() < 1) {
        return;
    }

    rT4 = boost::posix_time::microsec_clock::local_time();

    // Our noise is isotropic, so make it here after our compression
    Eigen::MatrixXd R_big = _options.sigma_pix_sq * Eigen::MatrixXd::Identity(res_big.rows(), res_big.rows());
    Eigen::MatrixXd R_big_line = _options.sigma_pix_sq * Eigen::MatrixXd::Identity(line_res_big.rows(), line_res_big.rows());

    // 6. With all good features update the state
    StateHelper::EKFUpdate(state, Hx_order_big, Hx_big, res_big, R_big);
    if (_use_line) {
        StateHelper::EKFUpdate(state, line_Hx_order_big, line_Hx_big, line_res_big, R_big_line);
    }

    rT5 = boost::posix_time::microsec_clock::local_time();

    // Debug print timing information
    PRINT_DEBUG("[MSCKF-UP]: %.4f seconds to clean\n", (rT1 - rT0).total_microseconds() * 1e-6);
    PRINT_DEBUG("[MSCKF-UP]: %.4f seconds to triangulate\n", (rT2 - rT1).total_microseconds() * 1e-6);
    PRINT_DEBUG("[MSCKF-UP]: %.4f seconds create system (%d features) (%d lines)\n", (rT3 - rT2).total_microseconds() * 1e-6, (int)feature_vec.size());
    PRINT_DEBUG("[MSCKF-UP]: %.4f seconds compress system\n", (rT4 - rT3).total_microseconds() * 1e-6);
    PRINT_DEBUG("[MSCKF-UP]: %.4f seconds update state (%d size) (%d size)\n", (rT5 - rT4).total_microseconds() * 1e-6, (int)res_big.rows());
    PRINT_DEBUG("[MSCKF-UP]: %.4f seconds total\n", (rT5 - rT1).total_microseconds() * 1e-6);
}

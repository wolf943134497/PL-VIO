/*
 * SPL_VIO
 * Copyright (C) 2022-2023 Yanyu Zhang
 * Copyright (C) 2022-2023 Wei Ren
 */

#include "InertialInitializer.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_init;

InertialInitializer::InertialInitializer(InertialInitializerOptions &params_, std::shared_ptr<ov_core::FeatureDatabase> db, std::shared_ptr<ov_core::LineDatabase> line_db)
    : params(params_), _db(db), _line_db(line_db) {

  // Vector of our IMU data
  imu_data = std::make_shared<std::vector<ov_core::ImuData>>();

  // Create initializers
  init_static = std::make_shared<StaticInitializer>(params, _db, _line_db, imu_data);
  // init_dynamic = std::make_shared<DynamicInitializer>(params, _db, _line_db, imu_data);
}

bool InertialInitializer::initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<ov_type::Type>> &order, std::shared_ptr<ov_type::IMU> t_imu, bool wait_for_jerk) {

  // Get the newest and oldest timestamps we will try to initialize between!
  double newest_cam_time = -1;
  for (auto const &feat : _db->get_internal_data()) {
    for (auto const &camtimepair : feat.second->timestamps) {
      for (auto const &time : camtimepair.second) {
        newest_cam_time = std::max(newest_cam_time, time);
      }
    }
  }

  double oldest_time = newest_cam_time - params.init_window_time - 0.01;
  if (newest_cam_time < 0 || oldest_time < 0) {
    return false;
  }

  // Remove all measurements that are older then our initialization window
  // Then we will try to use all features that are in the feature database!

  _db->cleanup_measurements(oldest_time);
  _line_db->cleanup_measurements(oldest_time);
  
  auto it_imu = imu_data->begin();
  while (it_imu != imu_data->end() && it_imu->timestamp < oldest_time + params.calib_camimu_dt) {
    it_imu = imu_data->erase(it_imu);
  }

  // Compute the disparity of the system at the current timestep
  // If disparity is zero or negative we will always use the static initializer
  bool disparity_detected_moving = false;
  if (params.init_max_disparity > 0) {

    // Get the disparity statistics from this image to the previous
    int num_features = 0;
    double average_disparity = 0.0;
    double variance_disparity = 0.0;
    int num_lines = 0;
    double line_average_disparity = 0.0;
    double line_variance_disparity = 0.0;
    FeatureHelper::compute_disparity(_db, average_disparity, variance_disparity, num_features);

PRINT_DEBUG(YELLOW "[init]: features db size %d \n" RESET, num_features);

    LineHelper::compute_disparity(_line_db, line_average_disparity, line_variance_disparity, num_lines);

PRINT_DEBUG(YELLOW "[init]: lines db size %d \n" RESET, num_lines);

    // Return if we can't compute the disparity
    if (num_features < 10) {
      PRINT_DEBUG(YELLOW "[init]: not enough features to compute disparity %d < 10\n" RESET, num_features);
      return false;
    }
    
    if (num_lines < 5) {
      PRINT_DEBUG(YELLOW "[init]: not enough lines to compute disparity %d < 5\n" RESET, num_lines);
     // return false;
    }

    // Check if it passed our check!
    PRINT_DEBUG(YELLOW "[init]: disparity of the platform is %.4f (%.4f threshold)\n" RESET, average_disparity, params.init_max_disparity);
    PRINT_DEBUG(YELLOW "[init]: line disparity of the platform is %.4f (%.4f threshold)\n" RESET, line_average_disparity, params.init_max_disparity);
    disparity_detected_moving = ((average_disparity > params.init_max_disparity) && (line_average_disparity > params.init_max_disparity));
  }
    
  // ======================================================================
  PRINT_DEBUG(GREEN "[init]: USING STATIC INITIALIZER METHOD!\n" RESET);
  return init_static->initialize(timestamp, covariance, order, t_imu, wait_for_jerk);
  // ======================================================================
  /*
  // Use our static initializer!
  if (!disparity_detected_moving && params.init_imu_thresh > 0.0) {
    PRINT_DEBUG(GREEN "[init]: USING STATIC INITIALIZER METHOD!\n" RESET);
    return init_static->initialize(timestamp, covariance, order, t_imu, wait_for_jerk);
  } else {
    PRINT_DEBUG(GREEN "[init]: USING DYNAMIC INITIALIZER METHOD!\n" RESET);
    std::map<double, std::shared_ptr<ov_type::PoseJPL>> _clones_IMU;
    std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>> _features_SLAM;
    std::unordered_map<size_t, std::shared_ptr<ov_type::Line_Landmark>> _lines_SLAM;
    std::unordered_map<size_t, std::shared_ptr<ov_type::PoseJPL>> _calib_IMUtoCAM;
    std::unordered_map<size_t, std::shared_ptr<ov_type::Vec>> _cam_intrinsics;
    return init_dynamic->initialize(timestamp, covariance, order, t_imu, _clones_IMU, _features_SLAM, _lines_SLAM, _calib_IMUtoCAM, _cam_intrinsics);
  }
  */
}

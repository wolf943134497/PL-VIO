/*
 * SPL_VIO
 * Copyright (C) 2022-2023 Yanyu Zhang
 * Copyright (C) 2022-2023 Wei Ren
 */

#ifndef OV_CORE_LINE_H
#define OV_CORE_LINE_H

#include <Eigen/Eigen>
#include <iostream>
#include <unordered_map>
#include <vector>

namespace ov_core {
    
class Line {

public:
  /// Unique ID of this line
  size_t lineid;
  
  /// If this line should be deleted
  bool to_delete;
  
  /// UV coordinates that this line startpoint has been seen from (mapped by camera ID)
  std::unordered_map<size_t, std::vector<Eigen::Vector2f>> startpoint;
  
  /// UV coordinates that this line endpoint has been seen from (mapped by camera ID)
  std::unordered_map<size_t, std::vector<Eigen::Vector2f>> endpoint;
  
  /// UV normalized coordinates that this line startpoint has been seen from (mapped by camera ID)
  std::unordered_map<size_t, std::vector<Eigen::Vector2f>> startpoint_norm;
  
  /// UV normalized coordinates that this line endpoint has been seen from (mapped by camera ID)
  std::unordered_map<size_t, std::vector<Eigen::Vector2f>> endpoint_norm;
  
  /// Timestamps of each UV measurement (mapped by camera ID)
  std::unordered_map<size_t, std::vector<double>> timestamps;
  
  /// What camera ID our pose is anchored in!! By default the first measurement is the anchor.
  int anchor_cam_id = -1;

  /// Timestamp of anchor clone
  double anchor_clone_timestamp;
  
  
  Eigen::Matrix<double, 4, 1> p_LinA;
  
  Eigen::Matrix<double, 4, 1> p_LinG;
  
  /**
   * @brief Remove measurements that do not occur at passed timestamps.
   */
  void clean_old_measurements(const std::vector<double> &valid_times);
  
  /**
   * @brief Remove measurements that occur at the invalid timestamps
   */
  void clean_invalid_measurements(const std::vector<double> &invalid_times);
  
  /**
   * @brief Remove measurements that are older then the specified timestamp.
   */
  void clean_older_measurements(double timestamp);
};

} // namespace ov_core

#endif /* OV_CORE_LINE_H */

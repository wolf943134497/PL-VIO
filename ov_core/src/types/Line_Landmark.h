/*
 * SPL_VIO
 * Copyright (C) 2022-2023 Yanyu Zhang
 * Copyright (C) 2022-2023 Wei Ren
 */

#ifndef OV_TYPE_LINE_LANDMARK_H
#define OV_TYPE_LINE_LANDMARK_H

#include "Line_Landmark_Rep.h"
#include "Vec.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

namespace ov_type {

/**
 * @brief Type that implements a persistent SLAM feature.
 *
 * We store the feature ID that should match the IDs in the trackers.
 * Additionally if this is an anchored representation we store what clone timestamp this is anchored from and what camera.
 * If this features should be marginalized its flag can be set and during cleanup it will be removed.
 */
class Line_Landmark : public Vec {

public:
  /// Default constructor (line feature is a Vec of size 4)
  Line_Landmark(int dim) : Vec(dim) {}

  /// Feature ID of this landmark (corresponds to frontend id)
  size_t _lineid;

  /// What unique camera stream this slam feature was observed from
  int _unique_camera_id = -1;

  /// What camera ID our pose is anchored in!! By default the first measurement is the anchor.
  int _anchor_cam_id = -1;

  /// Timestamp of anchor clone
  double _anchor_clone_timestamp = -1;

  /// Boolean if this landmark has had at least one anchor change
  bool has_had_anchor_change = false;

  /// Boolean if this landmark should be marginalized out
  bool should_marg = false;

  /// What feature representation this feature currently has
  Line_Landmark_Rep::Line_Representation _line_representation;


Eigen::Matrix<double, 4, 1> quat_multiply(const Eigen::Matrix<double, 4, 1> &q, const Eigen::Matrix<double, 4, 1> &p) {
  Eigen::Matrix<double, 4, 1> q_t;
  Eigen::Matrix<double, 4, 4> Qm;
  // create big L matrix
  Qm.block(0, 0, 3, 3) = q(3, 0) * Eigen::MatrixXd::Identity(3, 3) - skew_x(q.block(0, 0, 3, 1));
  Qm.block(0, 3, 3, 1) = q.block(0, 0, 3, 1);
  Qm.block(3, 0, 1, 3) = -q.block(0, 0, 3, 1).transpose();
  Qm(3, 3) = q(3, 0);
  q_t = Qm * p;
  // ensure unique by forcing q_4 to be >0
  if (q_t(3, 0) < 0) {
    q_t *= -1;
  }
  // normalize and return
  return q_t / q_t.norm();
}

Eigen::Matrix<double, 3, 3> skew_x(const Eigen::Matrix<double, 3, 1> &w) {
  Eigen::Matrix<double, 3, 3> w_x;
  w_x << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  return w_x;
}

  /**
   * @brief Overrides the default vector update rule
   * We want to selectively update the FEJ value if we are using an anchored representation.
   * @param dx Additive error state correction
   */
  void update(const Eigen::VectorXd &dx) override {
    // Update estimate
    assert(dx.rows() == _size);
    //set_value(_value + dx);
    Eigen::VectorXd result = (_value.norm() + dx.norm()) * quat_multiply(dx, _value);
    set_value(result);


    // Ensure we are not near zero in the z-direction
    // if (LandmarkRepresentation::is_relative_representation(_feat_representation) && _value(_value.rows() - 1) < 1e-8) {
    //  PRINT_DEBUG(YELLOW "WARNING DEPTH %.8f BECAME CLOSE TO ZERO IN UPDATE!!!\n" RESET, _value(_value.rows() - 1));
    //  should_marg = true;
    // }
  }

  /**
   * @brief Will return the position of the feature in the global frame of reference.
   * @param getfej Set to true to get the landmark FEJ value
   * @return Position of feature either in global or anchor frame
   */
  Eigen::Matrix<double, 4, 1> get_line(bool getfej) const;

  /**
   * @brief Will set the current value based on the representation.
   * @param p_FinG Position of the feature either in global or anchor frame
   * @param isfej Set to true to set the landmark FEJ value
   */
  void set_from_line(Eigen::Matrix<double, 4, 1> p_LinG, bool isfej);

};
} // namespace ov_type

#endif // OV_TYPE_LINE_LANDMARK_H

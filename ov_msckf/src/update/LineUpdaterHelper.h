#ifndef OV_MSCKF_LINE_UPDATER_HELPER_H
#define OV_MSCKF_LINE_UPDATER_HELPER_H

#include <Eigen/Eigen>

#include "feat/Line.h"
#include "state/State.h"
#include "state/StateOptions.h"
#include "types/Line_Landmark_Rep.h"
#include "utils/colors.h"
#include "utils/quat_ops.h"
#include "utils/line_geometry.h"

namespace ov_msckf {

/**
 * @brief Class that has helper functions for our updaters.
 *
 * Can compute the Jacobian for a single feature representation.
 * This will create the Jacobian based on what representation our state is in.
 * If we are using the anchor representation then we also have additional Jacobians in respect to the anchor state.
 * Also has functions such as nullspace projection and full jacobian construction.
 * For derivations look at @ref update-feat page which has detailed equations.
 *
 */
class LineUpdaterHelper {
public:
  /**
   * @brief Feature object that our UpdaterHelper leverages, has all measurements and means
   */
  struct UpdaterHelperLine {

    /// Unique ID of this feature
    size_t lineid;

    /// UV coordinates that this feature has been seen from (mapped by camera ID)
    std::unordered_map<size_t, std::vector<Eigen::Vector2f>> startpoint;
    std::unordered_map<size_t, std::vector<Eigen::Vector2f>> endpoint;

    // UV normalized coordinates that this feature has been seen from (mapped by camera ID)
    std::unordered_map<size_t, std::vector<Eigen::Vector2f>> startpoint_norm;
    std::unordered_map<size_t, std::vector<Eigen::Vector2f>> endpoint_norm;

    /// Timestamps of each UV measurement (mapped by camera ID)
    std::unordered_map<size_t, std::vector<double>> timestamps;

    /// What representation our feature is in
    ov_type::Line_Landmark_Rep::Line_Representation line_representation;

    /// What camera ID our pose is anchored in!! By default the first measurement is the anchor.
    int anchor_cam_id = -1;

    /// Timestamp of anchor clone
    double anchor_clone_timestamp = -1;
    
    Eigen::Matrix<double, 4, 1> p_LinA;

    Eigen::Matrix<double, 4, 1> p_LinA_fej;

    Eigen::Matrix<double, 4, 1> p_LinG;

    Eigen::Matrix<double, 4, 1> p_LinG_fej;

  };
  
    static void get_line_jacobian_representation(std::shared_ptr<State> state, UpdaterHelperLine &line, Eigen::MatrixXd &H_f, std::vector<Eigen::MatrixXd> &H_x, std::vector<std::shared_ptr<ov_type::Type>> &x_order);

    
    static void get_line_jacobian_full(std::shared_ptr<State> state, UpdaterHelperLine &line, Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res, std::vector<std::shared_ptr<ov_type::Type>> &x_order);
    
    /**
     * @brief This will project the left nullspace of H_f onto the linear system.
     *
     * Please see the @ref update-null for details on how this works.
     * This is the MSCKF nullspace projection which removes the dependency on the feature state.
     * Note that this is done **in place** so all matrices will be different after a function call.
     *
     * @param H_f Jacobian with nullspace we want to project onto the system [res = Hx*(x-xhat)+Hf(f-fhat)+n]
     * @param H_x State jacobian
     * @param res Measurement residual
     */
    static void nullspace_project_inplace(Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res);

    /**
     * @brief This will perform measurement compression
     *
     * Please see the @ref update-compress for details on how this works.
     * Note that this is done **in place** so all matrices will be different after a function call.
     *
     * @param H_x State jacobian
     * @param res Measurement residual
     */
    static void measurement_compress_inplace(Eigen::MatrixXd &H_x, Eigen::VectorXd &res);
    
};

} // namespace ov_msckf

#endif // OV_MSCKF_LINE_UPDATER_HELPER_H

/*
 * SPL_VIO
 * Copyright (C) 2022-2023 Yanyu Zhang
 * Copyright (C) 2022-2023 Wei Ren
 */

#ifndef OV_INIT_STATICINITIALIZER_H
#define OV_INIT_STATICINITIALIZER_H

#include "init/InertialInitializerOptions.h"
#include "utils/helper.h"

#include "feat/FeatureHelper.h"
#include "feat/LineHelper.h"
#include "types/IMU.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

namespace ov_init {

/**
 * @brief Initializer for a static visual-inertial system.
 *
 * This implementation that assumes that the imu starts from standing still.
 * To initialize from standstill:
 * 1. Collect all inertial measurements
 * 2. See if within the last window there was a jump in acceleration
 * 3. If the jump is past our threshold we should init (i.e. we have started moving)
 * 4. Use the *previous* window, which should have been stationary to initialize orientation
 * 5. Return a roll and pitch aligned with gravity and biases.
 *
 */
class StaticInitializer {

public:
    /**
     * @brief Default constructor
     * @param params_ Parameters loaded from either ROS or CMDLINE
     * @param db Feature tracker database with all features in it
     * @param imu_data_ Shared pointer to our IMU vector of historical information
     */
    explicit StaticInitializer(InertialInitializerOptions &params_, std::shared_ptr<ov_core::FeatureDatabase> db, std::shared_ptr<ov_core::LineDatabase> line_db, std::shared_ptr<std::vector<ov_core::ImuData>> imu_data_) : params(params_), _db(db), _line_db(line_db), imu_data(imu_data_) {}

    /**
     * @brief Try to get the initialized system using just the imu
     *
     * This will check if we have had a large enough jump in our acceleration.
     * If we have then we will use the period of time before this jump to initialize the state.
     * This assumes that our imu is sitting still and is not moving (so this would fail if we are experiencing constant acceleration).
     *
     * In the case that we do not wait for a jump (i.e. `wait_for_jerk` is false), then the system will try  to initialize as soon as possible.
     * This is only recommended if you have zero velocity update enabled to handle the stationary cases.
     * To initialize in this case, we need to have the average angular variance be below the set threshold (i.e. we need to be stationary).
     *
     * @param[out] timestamp Timestamp we have initialized the state at
     * @param[out] covariance Calculated covariance of the returned state
     * @param[out] order Order of the covariance matrix
     * @param[out] t_imu Our imu type element
     * @param wait_for_jerk If true we will wait for a "jerk"
     * @return True if we have successfully initialized our system
     */
    bool initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<ov_type::Type>> &order, std::shared_ptr<ov_type::IMU> t_imu, bool wait_for_jerk = true);

private:
    /// Initialization parameters
    InertialInitializerOptions params;

    /// Feature tracker database with all features in it
    std::shared_ptr<ov_core::FeatureDatabase> _db;
  
    std::shared_ptr<ov_core::LineDatabase> _line_db;

    /// Our history of IMU messages (time, angular, linear)
    std::shared_ptr<std::vector<ov_core::ImuData>> imu_data;
};

} // namespace ov_init

#endif // OV_INIT_STATICINITIALIZER_H

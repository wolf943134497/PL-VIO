/*
 * SPL_VIO
 * Copyright (C) 2022-2023 Yanyu Zhang
 * Copyright (C) 2022-2023 Wei Ren
 */

#ifndef OPEN_VINS_LINEINITIALIZER_H
#define OPEN_VINS_LINEINITIALIZER_H

#include <unordered_map>

#include "Line.h"
#include "utils/print.h"
#include "utils/line_geometry.h"
#include "utils/quat_ops.h"
#include "LineInitializerOptions.h"

namespace ov_core {
    
class LineInitializer {

public:
    /**
     * @brief Structure which stores pose estimates for use in triangulation
     *
     * - R_GtoC - rotation from global to camera
     * - p_CinG - position of camera in global frame
     */
    struct ClonePose {

        /// Rotation
        Eigen::Matrix<double, 3, 3> _Rot;

        /// Position
        Eigen::Matrix<double, 3, 1> _pos;

        /// Constructs pose from rotation and position
        ClonePose(const Eigen::Matrix<double, 3, 3> &R, const Eigen::Matrix<double, 3, 1> &p) {
            _Rot = R;
            _pos = p;
        }

        /// Constructs pose from quaternion and position
        ClonePose(const Eigen::Matrix<double, 4, 1> &q, const Eigen::Matrix<double, 3, 1> &p) {
            _Rot = quat_2_Rot(q);
            _pos = p;
        }

        /// Default constructor
        ClonePose() {
            _Rot = Eigen::Matrix<double, 3, 3>::Identity();
            _pos = Eigen::Matrix<double, 3, 1>::Zero();
        }

        /// Accessor for rotation
        const Eigen::Matrix<double, 3, 3> &Rot() { return _Rot; }

        /// Accessor for position
        const Eigen::Matrix<double, 3, 1> &pos() { return _pos; }
    };
    
    LineInitializer(LineInitializerOptions &options) : _options(options) {}
    
    bool line_triangulation(Line *line, std::unordered_map<size_t, std::unordered_map<double, ClonePose>> &clonesCAM);
    
    const LineInitializerOptions config() { return _options; }
     
     
protected:
    /// Contains options for the initializer process
    LineInitializerOptions _options;
  
};

} // namespace ov_core

#endif // OPEN_VINS_LINEINITIALIZER_H

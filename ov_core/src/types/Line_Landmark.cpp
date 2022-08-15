/*
 * SPL_VIO
 * Copyright (C) 2022-2023 Yanyu Zhang
 * Copyright (C) 2022-2023 Wei Ren
 */

#include "Line_Landmark.h"

using namespace ov_type;


Eigen::Matrix<double, 4, 1> Line_Landmark::get_line(bool getfej) const {

    if (_line_representation == Line_Landmark_Rep::Line_Representation::CP_LINE) {
        return (getfej) ? fej() : value();
    }

    // Failure
    assert(false);
    return Eigen::Matrix<double, 4, 1>::Zero();
}

void Line_Landmark::set_from_line(Eigen::Matrix<double, 4, 1> p_LinG, bool isfej) {

    if (_line_representation == Line_Landmark_Rep::Line_Representation::CP_LINE) {
        if (isfej)
            set_fej(p_LinG);
        else
            set_value(p_LinG);
        return;
    }

    // Failure
    assert(false);
}

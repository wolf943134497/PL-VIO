/*
 * SPL_VIO
 * Copyright (C) 2022-2023 Yanyu Zhang
 * Copyright (C) 2022-2023 Wei Ren
 */

#ifndef OV_CORE_LINE_HELPER_H
#define OV_CORE_LINE_HELPER_H

#include <Eigen/Eigen>
#include <memory>
#include <mutex>
#include <vector>

#include "Line.h"
#include "LineDatabase.h"
#include "utils/print.h"

namespace ov_core {

/**
 * @brief Contains some nice helper functions for features.
 *
 * These functions should only depend on feature and the feature database.
 */
class LineHelper {

public:
    /**
     * @brief This functions will compute the disparity between common features in the two frames.
     *
     * First we find all features in the first frame.
     * Then we loop through each and find the uv of it in the next requested frame.
     * Features are skipped if no tracked feature is found (it was lost).
     * NOTE: this is on the RAW coordinates of the feature not the normalized ones.
     * NOTE: This computes the disparity over all cameras!
     *
     * @param db Feature database pointer
     * @param time0 First camera frame timestamp
     * @param time1 Second camera frame timestamp
     * @param disp_mean Average raw disparity
     * @param disp_var Variance of the disparities
     * @param total_feats Total number of common features
     */
    static void compute_line_disparity(std::shared_ptr<ov_core::LineDatabase> line_db, double time0, double time1, double &disp_mean, double &disp_var, int &total_lines) {

        // Get features seen from the first image
        std::vector<std::shared_ptr<Line>> lines0 = line_db->lines_containing(time0, false, true);

        // Compute the disparity
        std::vector<double> disparities;
        for (auto &line : lines0) {

            // Get the two uvs for both times
            for (auto &campairs : line->timestamps) {

                // First find the two timestamps
                size_t camid = campairs.first;
                auto it0 = std::find(line->timestamps.at(camid).begin(), line->timestamps.at(camid).end(), time0);
                auto it1 = std::find(line->timestamps.at(camid).begin(), line->timestamps.at(camid).end(), time1);
                if (it0 == line->timestamps.at(camid).end() || it1 == line->timestamps.at(camid).end())
                    continue;
                auto idx0 = std::distance(line->timestamps.at(camid).begin(), it0);
                auto idx1 = std::distance(line->timestamps.at(camid).begin(), it1);

                // Now lets calculate the disparity
                // startpoint(X1, Y1), endpoint(X2, Y2)
                // A * X + B * Y + C = 0
                // A = Y2 - Y1
                // B = X1 - X2
                // C = X2 * Y1 - X1 * Y2 
                double A = line->endpoint.at(camid).at(idx0)[1]- line->startpoint.at(camid).at(idx0)[1];
                double B = line->startpoint.at(camid).at(idx0)[0] - line->endpoint.at(camid).at(idx0)[0];
                double C = (line->endpoint.at(camid).at(idx0)[0] * line->startpoint.at(camid).at(idx0)[1]) - (line->startpoint.at(camid).at(idx0)[0] * line->endpoint.at(camid).at(idx0)[1]);
                
                double dist0 = (line->startpoint.at(camid).at(idx1)[0] * A + line->startpoint.at(camid).at(idx1)[1] * B + C) / std::sqrt(A * A + B * B);
                double dist1 = (line->endpoint.at(camid).at(idx1)[0] * A + line->endpoint.at(camid).at(idx1)[1] * B + C) / std::sqrt(A * A + B * B);
                
                Eigen::Matrix<double,2,1> dist;
                dist << dist0, dist1;
                disparities.push_back(dist.norm());
            }
        }

        // If no disparities, just return
        if (disparities.size() < 2) {
            disp_mean = -1;
            disp_var = -1;
            total_lines = 0;
        }

        // Compute mean and standard deviation in respect to it
        disp_mean = 0;
        for (double disp_i : disparities) {
            disp_mean += disp_i;
        }
        disp_mean /= (double)disparities.size();
        disp_var = 0;
        for (double &disp_i : disparities) {
            disp_var += std::pow(disp_i - disp_mean, 2);
        }
        disp_var = std::sqrt(disp_var / (double)(disparities.size() - 1));
        total_lines = (int)disparities.size();
    }

    /**
     * @brief This functions will compute the disparity over all features we have
     *
     * NOTE: this is on the RAW coordinates of the feature not the normalized ones.
     * NOTE: This computes the disparity over all cameras!
     *
     * @param db Feature database pointer
     * @param disp_mean Average raw disparity
     * @param disp_var Variance of the disparities
     * @param total_feats Total number of common features
     */
    static void compute_disparity(std::shared_ptr<ov_core::LineDatabase> line_db, double &disp_mean, double &disp_var, int &total_lines) {

    // Compute the disparity
    std::vector<double> disparities;

    for (auto &line : line_db->get_internal_data()) {
        for (auto &campairs : line.second->timestamps) {

            // Skip if only one observation
            if (campairs.second.size() < 2)
                continue;

            // Now lets calculate the disparity
            size_t camid = campairs.first;
            
            // Now lets calculate the disparity
            // startpoint(X1, Y1), endpoint(X2, Y2)
            // A * X + B * Y + C = 0
            // A = Y2 - Y1
            // B = X1 - X2
            // C = X2 * Y1 - X1 * Y2 
            double A = line.second->endpoint.at(camid).at(0)[1]- line.second->startpoint.at(camid).at(0)[1];
            double B = line.second->startpoint.at(camid).at(0)[0] - line.second->endpoint.at(camid).at(0)[0];
            double C = (line.second->endpoint.at(camid).at(0)[0] * line.second->startpoint.at(camid).at(0)[1]) - (line.second->startpoint.at(camid).at(0)[0] * line.second->endpoint.at(camid).at(0)[1]);
                
            double dist0 = (line.second->startpoint.at(camid).at(campairs.second.size() - 1)[0] * A + line.second->startpoint.at(camid).at(campairs.second.size() - 1)[1] * B + C) / std::sqrt(A * A + B * B);
            double dist1 = (line.second->endpoint.at(camid).at(campairs.second.size() - 1)[0] * A + line.second->endpoint.at(camid).at(campairs.second.size() - 1)[1] * B + C) / std::sqrt(A * A + B * B);
                
            Eigen::Matrix<double,2,1> dist;
            dist << dist0, dist1;
            disparities.push_back(dist.norm());
        }
    }

    // If no disparities, just return
    if (disparities.size() < 2) {
        disp_mean = -1;
        disp_var = -1;
        total_lines = 0;
    }

    // Compute mean and standard deviation in respect to it
    disp_mean = 0;
    for (double disp_i : disparities) {
        disp_mean += disp_i;
    }
    disp_mean /= (double)disparities.size();
    disp_var = 0;
    for (double &disp_i : disparities) {
        disp_var += std::pow(disp_i - disp_mean, 2);
    }
    disp_var = std::sqrt(disp_var / (double)(disparities.size() - 1));
    total_lines = (int)disparities.size();
  }

private:
  // Cannot construct this class
  LineHelper() {}
};

} // namespace ov_core

#endif /* OV_CORE_LINE_HELPER_H */

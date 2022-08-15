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

#include "LineTrackSIM.h"

using namespace ov_core;

void LineTrackSIM::feed_measurement_simulation(double timestamp, const std::vector<int> &camids,
                                           const std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> &lines) {

  // Assert our two vectors are equal
  assert(camids.size() == lines.size());

  // Loop through each camera
  for (size_t i = 0; i < camids.size(); i++) {

    // Current camera id
    int cam_id = camids.at(i);

    // Our good ids and points
    std::vector<cv::line_descriptor::KeyLine> line_good_left;
    std::vector<size_t> line_good_ids_left;

    // Update our feature database, with theses new observations
    // NOTE: we add the "currid" since we need to offset the simulator
    // NOTE: ids by the number of aruoc tags we have specified as tracking
    for (const auto &line : lines.at(i)) {

      // Get our id value
      size_t line_id = line.first + currid;

      // Create the keypoint
      cv::line_descriptor::KeyLine kline_left;

      kline_left.startPointX = line.second(0);
      kline_left.startPointY = line.second(1);
      kline_left.endPointX = line.second(2);
      kline_left.endPointY = line.second(3);

      line_good_left.push_back(kline_left);
      line_good_ids_left.push_back(line_id);

      // Append to the database
      cv::Point2f nline_l = camera_calib.at(cam_id)->undistort_cv(kline_left.getStartPoint());
      cv::Point2f nline_r = camera_calib.at(cam_id)->undistort_cv(kline_left.getEndPoint());
      //cv::Point2f nline_l = kline_left.getStartPoint();
      //cv::Point2f nline_r = kline_left.getEndPoint();


      Eigen::Vector2f a, b, c, d;
      a(0) = kline_left.getStartPoint().x;
      a(1) = kline_left.getStartPoint().y;
      b(0) = kline_left.getEndPoint().x;
      b(1) = kline_left.getEndPoint().y;
      c(0) = nline_l.x;
      c(1) = nline_l.y;
      d(0) = nline_r.x;
      d(1) = nline_r.y;

      line_database->update_line(line_id, timestamp, cam_id, a, b, c, d);
    }

    // Get our width and height
    int width = camera_calib.at(cam_id)->w();
    int height = camera_calib.at(cam_id)->h();

    // Move forward in time
    //img_last[cam_id] = cv::Mat::zeros(cv::Size(width, height), CV_8UC1);
    //img_mask_last[cam_id] = cv::Mat::zeros(cv::Size(width, height), CV_8UC1);
    lines_last[cam_id] = line_good_left;
    line_ids_last[cam_id] = line_good_ids_left;
  }
}


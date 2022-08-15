/*
 * SPL_VIO
 * Copyright (C) 2022-2023 Yanyu Zhang
 * Copyright (C) 2022-2023 Wei Ren
 */

#include "LineTrackBase.h"

using namespace ov_core;

void LineTrackBase::display_active(cv::Mat &img_out, int r1, int g1, int b1, std::string overlay) {

    // Cache the images to prevent other threads from editing while we viz (which can be slow)
    std::map<size_t, cv::Mat> img_last_cache, img_mask_last_cache;
    for (auto const &pair : img_last) {
        img_last_cache.insert({pair.first, pair.second.clone()});
    }

    // Get the largest width and height
    int max_width = -1;
    int max_height = -1;
    for (auto const &pair : img_last_cache) {
        if (max_width < pair.second.cols)
        max_width = pair.second.cols;
        if (max_height < pair.second.rows)
        max_height = pair.second.rows;
    }

    // Return if we didn't have a last image
    if (max_width == -1 || max_height == -1)
        return;

    // If the image is "small" thus we should use smaller display codes
    bool is_small = (std::min(max_width, max_height) < 400);

    // If the image is "new" then draw the images from scratch
    // Otherwise, we grab the subset of the main image and draw on top of it
    bool image_new = ((int)img_last_cache.size() * max_width != img_out.cols || max_height != img_out.rows);

    // If new, then resize the current image
    if (image_new)
        img_out = cv::Mat(max_height, (int)img_last_cache.size() * max_width, CV_8UC3, cv::Scalar(0, 0, 0));

    // Loop through each image, and draw
    int index_cam = 0;
    
    for (auto const &pair : img_last_cache) {
        // Lock this image
        std::lock_guard<std::mutex> lck(mtx_feeds.at(pair.first));
        // select the subset of the image
        cv::Mat img_temp;
        if (image_new)
            cv::cvtColor(img_last_cache[pair.first], img_temp, cv::COLOR_GRAY2RGB);
        else
            img_temp = img_out(cv::Rect(max_width * index_cam, 0, max_width, max_height));
        // draw, loop through all keylines

        for (size_t i = 0; i < lines_last[pair.first].size(); i++) {
            cv::Point2f startPoint = cv::Point(int(lines_last[pair.first].at(i).startPointX), int(lines_last[pair.first].at(i).startPointY));
            cv::Point2f endPoint = cv::Point(int(lines_last[pair.first].at(i).endPointX), int(lines_last[pair.first].at(i).endPointY));
            cv::line(img_temp, startPoint, endPoint, cv::Scalar(r1, g1, b1), 2 ,8);
        }

        // Draw what camera this is
        auto txtpt = (is_small) ? cv::Point(10, 30) : cv::Point(30, 60);
        if (overlay == "") {
            cv::putText(img_temp, "CAM:" + std::to_string((int)pair.first), txtpt, cv::FONT_HERSHEY_COMPLEX_SMALL, (is_small) ? 1.5 : 3.0, cv::Scalar(0, 255, 0), 3);
        } else {
            cv::putText(img_temp, overlay, txtpt, cv::FONT_HERSHEY_COMPLEX_SMALL, (is_small) ? 1.5 : 3.0, cv::Scalar(0, 0, 255), 3);
        }

        // Replace the output image
        img_temp.copyTo(img_out(cv::Rect(max_width * index_cam, 0, img_last_cache[pair.first].cols, img_last_cache[pair.first].rows)));
        index_cam++;

    }
    
}


void LineTrackBase::display_history(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2, std::string overlay) {

    // Cache the images to prevent other threads from editing while we viz (which can be slow)
    std::map<size_t, cv::Mat> img_last_cache, img_mask_last_cache;
    for (auto const &pair : img_last) {
        img_last_cache.insert({pair.first, pair.second.clone()});
    }

    // Get the largest width and height
    int max_width = -1;
    int max_height = -1;
    for (auto const &pair : img_last_cache) {
        if (max_width < pair.second.cols)
            max_width = pair.second.cols;
        if (max_height < pair.second.rows)
            max_height = pair.second.rows;
    }

    // Return if we didn't have a last image
    if (max_width == -1 || max_height == -1)
        return;

    // If the image is "small" thus we shoudl use smaller display codes
    bool is_small = (std::min(max_width, max_height) < 400);

    // If the image is "new" then draw the images from scratch
    // Otherwise, we grab the subset of the main image and draw on top of it
    bool image_new = ((int)img_last_cache.size() * max_width != img_out.cols || max_height != img_out.rows);

    // If new, then resize the current image
    if (image_new)
        img_out = cv::Mat(max_height, (int)img_last_cache.size() * max_width, CV_8UC3, cv::Scalar(0, 0, 0));

    // Max tracks to show (otherwise it clutters up the screen)
    size_t maxtracks = 3;

    // Loop through each image, and draw
    int index_cam = 0;
    for (auto const &pair : img_last_cache) {
        // Lock this image
        std::lock_guard<std::mutex> lck(mtx_feeds.at(pair.first));
        // select the subset of the image
        cv::Mat img_temp;
        if (image_new)
            cv::cvtColor(img_last_cache[pair.first], img_temp, cv::COLOR_GRAY2RGB);
        else
            img_temp = img_out(cv::Rect(max_width * index_cam, 0, max_width, max_height));
        // draw, loop through all keypoints
        for (size_t i = 0; i < line_ids_last[pair.first].size(); i++) {
            // Get the line from the database
            std::shared_ptr<Line> line = line_database->get_line(line_ids_last[pair.first].at(i));
            // Skip if the line is null
            if (line == nullptr || line->startpoint[pair.first].empty() || line->to_delete)
                continue;
            // Draw the history of this line (start at the last inserted one)
            
            for (size_t z = line->startpoint[pair.first].size() - 1; z > 0; z--) {
                // Check if we have reached the max
                if (line->startpoint[pair.first].size() - z > maxtracks)
                    break;
                // Calculate what color we are drawing in
                bool is_stereo = (line->startpoint.size() > 1);
                int color_r = (is_stereo ? b2 : r2) - (int)((is_stereo ? b1 : r1) / line->startpoint[pair.first].size() * z);
                int color_g = (is_stereo ? r2 : g2) - (int)((is_stereo ? r1 : g1) / line->startpoint[pair.first].size() * z);
                int color_b = (is_stereo ? g2 : b2) - (int)((is_stereo ? g1 : b1) / line->startpoint[pair.first].size() * z);
                
                // Draw current point
                cv::Point2f startPoint_c(line->startpoint[pair.first].at(z)(0), line->startpoint[pair.first].at(z)(1));
                cv::Point2f endPoint_c(line->endpoint[pair.first].at(z)(0), line->endpoint[pair.first].at(z)(1));
                cv::line(img_temp, startPoint_c, endPoint_c, cv::Scalar(color_r, color_g, color_b), 2 ,8);
                
                // If there is a next point, then display the line from this point to the next
                if (z + 1 < line->startpoint[pair.first].size()) {
                    cv::Point2f startPoint_n(line->startpoint[pair.first].at(z+1)(0), line->startpoint[pair.first].at(z+1)(1));
                    cv::Point2f endPoint_n(line->endpoint[pair.first].at(z+1)(0), line->endpoint[pair.first].at(z+1)(1));
                    cv::line(img_temp, startPoint_n, endPoint_n, cv::Scalar(color_r, color_g, color_b), 2 ,8);
                    
                    cv::line(img_temp, (startPoint_c+endPoint_c)/2, (startPoint_n+endPoint_n)/2, cv::Scalar(0, 255, 0), 2);
                }
            }
        }       
    // Draw what camera this is
    auto txtpt = (is_small) ? cv::Point(10, 30) : cv::Point(30, 60);
    if (overlay == "") {
      cv::putText(img_temp, "CAM:" + std::to_string((int)pair.first), txtpt, cv::FONT_HERSHEY_COMPLEX_SMALL, (is_small) ? 1.5 : 3.0,
                  cv::Scalar(0, 255, 0), 3);
    } else {
      cv::putText(img_temp, overlay, txtpt, cv::FONT_HERSHEY_COMPLEX_SMALL, (is_small) ? 1.5 : 3.0, cv::Scalar(0, 0, 255), 3);
    }

    // Replace the output image
    img_temp.copyTo(img_out(cv::Rect(max_width * index_cam, 0, img_last_cache[pair.first].cols, img_last_cache[pair.first].rows)));
    index_cam++;
  }
}



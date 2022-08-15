/*
 * SPL_VIO
 * Copyright (C) 2022-2023 Yanyu Zhang
 * Copyright (C) 2022-2023 Wei Ren
 */

#ifndef OV_CORE_LINE_TRACK_BASE_H
#define OV_CORE_LINE_TRACK_BASE_H

#include <atomic>
#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "LBD_LSD.h"
#include "cam/CamBase.h"
#include "feat/LineDatabase.h"
#include "utils/colors.h"
#include "utils/opencv_lambda_body.h"
#include "utils/sensor_data.h"
#include "utils/print.h"


namespace ov_core {
    
    
class LineTrackBase {

public:
    /**
     * @brief Desired pre-processing image method.
     */
    enum Line_HistogramMethod { NONE, HISTOGRAM };
    
    
    LineTrackBase(std::unordered_map<size_t, std::shared_ptr<CamBase>> cameras, int numlines, int numaruco, bool stereo, Line_HistogramMethod line_histmethod) : camera_calib(cameras), line_database(new LineDatabase()), num_lines(numlines), use_stereo(stereo), line_histogram_method(line_histmethod) {
        // Our current feature ID should be larger then the number of aruco tags we have (each has 4 corners)
        //currid = 4 * (size_t)numaruco + 1;
        currid = 0;

        // Create our mutex array based on the number of cameras we have
        // See https://stackoverflow.com/a/24170141/7718197
        if (mtx_feeds.empty() || mtx_feeds.size() != camera_calib.size()) {
            std::vector<std::mutex> list(camera_calib.size());
            mtx_feeds.swap(list);
        }
    }
  
    virtual ~LineTrackBase() {}
    
    /**
     * @brief Process a new image
     * @param message Contains our timestamp, images, and camera ids
     */
    virtual void feed_new_camera(const CameraData &message) = 0;
    
    /**
     * @brief Shows lines extracted in the last image
     * @param img_out image to which we will overlayed lines on
     * @param r1,g1,b1 first color to draw in
     * @param r2,g2,b2 second color to draw in
     * @param overlay Text overlay to replace to normal "cam0" in the top left of screen
     */
    virtual void display_active(cv::Mat &img_out, int r1, int g1, int b1, std::string overlay = "");
    
    /**
     * @brief Shows a "trail" for each line (i.e. its history)
     * @param img_out image to which we will overlayed lines on
     * @param r1,g1,b1 first color to draw in
     * @param r2,g2,b2 second color to draw in
     * @param highlighted unique ids which we wish to highlight (e.g. slam feats)
     * @param overlay Text overlay to replace to normal "cam0" in the top left of screen
     */
    
    
    virtual void display_history(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2, std::string overlay = "");
    
    
    /**
     * @brief Get the line database with all the track information
     * @return FeatureDatabase pointer that one can query for features
     */
    std::shared_ptr<LineDatabase> get_line_database() { return line_database; }
    
    /**
     * @brief Changes the ID of an actively tracked line to another one.
     *
     * This function can be helpfull if you detect a loop-closure with an old frame.
       * One could then change the id of an active feature to match the old feature id!
     *
     * @param id_old Old id we want to change
     * @param id_new Id we want to change the old id to
     */
    void change_line_id(size_t id_old, size_t id_new) {

        // If found in db then replace
        if (line_database->get_internal_data().find(id_old) != line_database->get_internal_data().end()) {
            std::shared_ptr<Line> line = line_database->get_internal_data().at(id_old);
            line_database->get_internal_data().erase(id_old);
            line->lineid = id_new;
            line_database->get_internal_data().insert({id_new, line});
        }

        // Update current track IDs
        for (auto &cam_ids_pair : line_ids_last) {
            for (size_t i = 0; i < cam_ids_pair.second.size(); i++) {
                if (cam_ids_pair.second.at(i) == id_old) {
                    line_ids_last.at(cam_ids_pair.first).at(i) = id_new;
                }
            }
        }
    }
   
    int get_num_lines() { return num_lines; }

    /// Setter method for number of active features
    void set_num_lines(int _num_lines) { num_lines = _num_lines; }

    
    
protected:
    /// Camera object which has all calibration in it
    std::unordered_map<size_t, std::shared_ptr<CamBase>> camera_calib;

    /// Database with all our current line
    std::shared_ptr<LineDatabase> line_database;

    /// If we are a fisheye model or not
    std::map<size_t, bool> camera_fisheye;

    /// If we should use binocular tracking or stereo tracking for multi-camera
    bool use_stereo;

    /// What histogram equalization method we should pre-process images with?
    Line_HistogramMethod line_histogram_method;
    
    int num_lines;

    /// Mutexs for our last set of image storage (img_last, lines_last, and ids_last)
    std::vector<std::mutex> mtx_feeds;

    /// Last set of images (use map so all trackers render in the same order)
    std::map<size_t, cv::Mat> img_last;

    /// Last set of tracked points
    std::unordered_map<size_t, std::vector<cv::line_descriptor::KeyLine>> lines_last;

    /// Set of IDs of each current feature in the database
    std::unordered_map<size_t, std::vector<size_t>> line_ids_last;

    /// Master ID for this tracker (atomic to allow for multi-threading)
    std::atomic<size_t> currid;

};

} // namespace ov_core

#endif /* OV_CORE_LINE_TRACK_BASE_H */

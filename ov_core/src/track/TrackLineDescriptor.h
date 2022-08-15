/*
 * SPL_VIO
 * Copyright (C) 2022-2023 Yanyu Zhang
 * Copyright (C) 2022-2023 Wei Ren
 */

#ifndef OV_CORE_TRACK_LINE_DESC_H
#define OV_CORE_TRACK_LINE_DESC_H

#include <opencv2/features2d.hpp>

#include "LBD_LSD.h"
#include "LineTrackBase.h"

namespace ov_core {

class TrackLineDescriptor : public LineTrackBase {

public:

    explicit TrackLineDescriptor(std::unordered_map<size_t, std::shared_ptr<CamBase>> cameras, int numlines, int numaruco, bool binocular, Line_HistogramMethod line_histmethod): LineTrackBase(cameras, numlines, numaruco, binocular, line_histmethod) {}
    
    /**
     * @brief Process a new image
     * @param message Contains our timestamp, images, and camera ids
     */
    
    void feed_new_camera(const CameraData &message);
    
protected:
   /**
    * @brief Process a new monocular image
    * @param message Contains our timestamp, images, and camera ids
    * @param msg_id the camera index in message data vector
    */
    void feed_line_monocular(const CameraData &message, size_t msg_id);
    
    
    void feed_line_stereo(const CameraData &message, size_t msg_id_left, size_t msg_id_right);
   
  
    /**
     * @brief Detects new lines in the current image
     * @param img0 image we will detect features on
     * @param lns0 vector of extracted keylines
     * @param ln_desc0 vector of the extracted lines descriptors
     * @param ln_ids0 vector of all new IDs
     *
     * Given a set of images, and their currently extracted line features, this will try to add new features.
     * We return all extracted descriptors here since we DO NOT need to do stereo tracking left to right.
     * Our vector of IDs will be later overwritten when we match features temporally to the previous frame's 
     * features
     */
    void perform_line_detection_monocular(const cv::Mat &img0, std::vector<cv::line_descriptor::KeyLine> &lns0, cv::Mat &ln_desc0, std::vector<size_t> &ln_ids0);
    
    
    void perform_line_detection_stereo(const cv::Mat &img0, const cv::Mat &img1, std::vector<cv::line_descriptor::KeyLine> &lns0, std::vector<cv::line_descriptor::KeyLine> &lns1, cv::Mat &ln_desc0, cv::Mat &ln_desc1, std::vector<size_t> &ln_ids0, std::vector<size_t> &ln_ids1);
    
    
    /**
     * @brief Find line matches between two keypoint+descriptor sets.
     * @param lns0 first vector of keylines
     * @param lns1 second vector of keylines
     * @param ln_desc0 first vector of descriptors
     * @param ln_desc1 second vector of decriptors
     * @param id0 id of the first camera
     * @param id1 id of the second camera
     * @param line_matches vector of matches that we have found
     */
    void line_robust_match(std::vector<cv::line_descriptor::KeyLine> &lns0, std::vector<cv::line_descriptor::KeyLine> &lns1, cv::Mat &ln_desc0, cv::Mat &ln_desc1, std::vector<cv::DMatch> &line_matches);
    
    /**
     * @brief Visualize line matches between two images.
     * @param img0 first image
     * @param img1 second image
     * @param lns0 first vector of keylines
     * @param lns1 second vector of keylines
     * @param line_matches vector of matches that we have found
     */
    void line_visualize(const cv::Mat &img0, const cv::Mat &img1, std::vector<cv::line_descriptor::KeyLine> &lns0, std::vector<cv::line_descriptor::KeyLine> &lns1, std::vector<cv::DMatch> &line_matches);
    
    
    // Timing variables
    boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7;

    // Descriptor matrices
    std::unordered_map<size_t, cv::Mat> line_desc_last;
  
};

} // namespace ov_core

#endif /* OV_CORE_TRACK_LINE_DESC_H */




  
  
  
  
  
  
  
  
  
  
  
  

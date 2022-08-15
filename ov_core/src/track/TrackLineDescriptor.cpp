/*
 * SPL_VIO
 * Copyright (C) 2022-2023 Yanyu Zhang
 * Copyright (C) 2022-2023 Wei Ren
 */

#include <cmath>
#include "TrackLineDescriptor.h"

using namespace ov_core;


void TrackLineDescriptor::feed_new_camera(const CameraData &message) {

    // Error check that we have all the data
    if (message.sensor_ids.empty() || message.sensor_ids.size() != message.images.size() || message.images.size() != message.masks.size()) {
        PRINT_ERROR(RED "[ERROR]: MESSAGE DATA SIZES DO NOT MATCH OR EMPTY!!!\n" RESET);
        PRINT_ERROR(RED "[ERROR]:   - message.sensor_ids.size() = %zu\n" RESET, message.sensor_ids.size());
        PRINT_ERROR(RED "[ERROR]:   - message.images.size() = %zu\n" RESET, message.images.size());
        PRINT_ERROR(RED "[ERROR]:   - message.masks.size() = %zu\n" RESET, message.masks.size());
        std::exit(EXIT_FAILURE);
    }

    // Either call our stereo or monocular version
    // If we are doing binocular tracking, then we should parallize our tracking
    size_t num_images = message.images.size();
    if (num_images == 1) {
        feed_line_monocular(message, 0);
    } else if (num_images == 2 && use_stereo) {
        //feed_line_stereo(message, 0, 1);
    } else if (!use_stereo) {
        parallel_for_(cv::Range(0, (int)num_images), LambdaBody([&](const cv::Range &range) {
            for (int i = range.start; i < range.end; i++) {
                feed_line_monocular(message, i);
            }
        }));
    } else {
        PRINT_ERROR(RED "[ERROR]: invalid number of images passed %zu, we only support mono or stereo tracking", num_images);
        std::exit(EXIT_FAILURE);
    }
}


void TrackLineDescriptor::feed_line_monocular(const CameraData &message, size_t msg_id) {
    
    // Start timing
    rT1 = boost::posix_time::microsec_clock::local_time();

    // Lock this data feed for this camera
    size_t cam_id = message.sensor_ids.at(msg_id);
    std::lock_guard<std::mutex> lck(mtx_feeds.at(cam_id));

    // Histogram equalize
    cv::Mat img;
    if (line_histogram_method == Line_HistogramMethod::HISTOGRAM) {
        cv::equalizeHist(message.images.at(msg_id), img);

        //cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(10.0, cv::Size(8, 8));
        //clahe->apply(message.images.at(msg_id), img);
    } else {
        img = message.images.at(msg_id);
    }

    // If we are the first frame (or have lost tracking), initialize our descriptors
    if (lines_last.find(cam_id) == lines_last.end() || lines_last[cam_id].empty()) {
        perform_line_detection_monocular(img, lines_last[cam_id], line_desc_last[cam_id], line_ids_last[cam_id]);
        //img_last[cam_id] = img;
        return;
    }

    // Our new keypoints and descriptor for the new image
    std::vector<cv::line_descriptor::KeyLine> lines_new;
    cv::Mat line_desc_new;
    std::vector<size_t> line_ids_new;

    // First, extract new descriptors for this new image
    perform_line_detection_monocular(img, lines_new, line_desc_new, line_ids_new);
    rT2 = boost::posix_time::microsec_clock::local_time();

    // Our matches temporally
    std::vector<cv::DMatch> line_matches_ll;

    // Lets match temporally
    line_robust_match(lines_last[cam_id], lines_new, line_desc_last[cam_id], line_desc_new, line_matches_ll);
    rT3 = boost::posix_time::microsec_clock::local_time();
    
    // Get our "good tracks"
    std::vector<cv::line_descriptor::KeyLine> line_good_left;
    std::vector<size_t> line_good_ids_left;
    cv::Mat line_good_desc_left;

    // Count how many we have tracked from the last time
    int num_tracklast = 0;

    // Loop through all current left to right lines
    // We want to see if any of theses have matches to the previous frame
    // If we have a match new->old then we want to use that ID instead of the new one
    for (size_t i = 0; i < lines_new.size(); i++) {

        // Loop through all left matches, and find the old "train" id
        int idll = -1;
        for (size_t j = 0; j < line_matches_ll.size(); j++) {
            if (line_matches_ll[j].trainIdx == (int)i) {
                idll = line_matches_ll[j].queryIdx;
            }
        }

        // Then lets replace the current ID with the old ID if found
        // Else just append the current feature and its unique ID
        line_good_left.push_back(lines_new[i]);
        line_good_desc_left.push_back(line_desc_new.row((int)i));
        if (idll != -1) {
            line_good_ids_left.push_back(line_ids_last[cam_id][idll]);
            num_tracklast++;
        } else {
            line_good_ids_left.push_back(line_ids_new[i]);
        }
    }
    rT4 = boost::posix_time::microsec_clock::local_time();

    // Update our line database, with theses new observations
    for (size_t i = 0; i < line_good_left.size(); i++) {
        Eigen::Vector2f start_ln_left = Eigen::Vector2f(line_good_left.at(i).startPointX, line_good_left.at(i).startPointY);
        Eigen::Vector2f end_ln_left = Eigen::Vector2f(line_good_left.at(i).endPointX, line_good_left.at(i).endPointY);
        
        Eigen::Vector2f start_ln_left_un = camera_calib.at(cam_id)->undistort_f(Eigen::Vector2f(line_good_left.at(i).startPointX, line_good_left.at(i).startPointY));
        Eigen::Vector2f end_ln_left_un = camera_calib.at(cam_id)->undistort_f(Eigen::Vector2f(line_good_left.at(i).endPointX, line_good_left.at(i).endPointY));
        
        line_database->update_line(line_good_ids_left.at(i), message.timestamp, cam_id, start_ln_left, end_ln_left, start_ln_left_un, end_ln_left_un);
    }

    // Move forward in time
    //img_last[cam_id] = img;
    lines_last[cam_id] = line_good_left;
    line_ids_last[cam_id] = line_good_ids_left;
    line_desc_last[cam_id] = line_good_desc_left;
    rT5 = boost::posix_time::microsec_clock::local_time();

}

/*
void TrackLineDescriptor::feed_line_stereo(const CameraData &message, size_t msg_id_left, size_t msg_id_right) {
    
    // Start timing
    rT1 = boost::posix_time::microsec_clock::local_time();

    // Lock this data feed for this camera
    size_t cam_id_left = message.sensor_ids.at(msg_id_left);
    size_t cam_id_right = message.sensor_ids.at(msg_id_right);
    std::lock_guard<std::mutex> lck1(mtx_feeds.at(cam_id_left));
    std::lock_guard<std::mutex> lck2(mtx_feeds.at(cam_id_right));

    // Histogram equalize images
    cv::Mat img_left, img_right, mask_left, mask_right;
    if (line_histogram_method == Line_HistogramMethod::HISTOGRAM) {
        cv::equalizeHist(message.images.at(msg_id_left), img_left);
        cv::equalizeHist(message.images.at(msg_id_right), img_right);
    } else {
        img_left = message.images.at(msg_id_left);
        img_right = message.images.at(msg_id_right);
    }
    
    // If we are the first frame (or have lost tracking), initialize our descriptors
    if (lines_last[cam_id_left].empty() || lines_last[cam_id_right].empty()) {
        perform_line_detection_stereo(img_left, img_right, lines_last[cam_id_left], lines_last[cam_id_right], line_desc_last[cam_id_left], line_desc_last[cam_id_right], line_ids_last[cam_id_left], line_ids_last[cam_id_right]);
        //img_last[cam_id_left] = img_left;
        //img_last[cam_id_right] = img_right;
        return;
    }

    // Our new keypoints and descriptor for the new image
    std::vector<cv::line_descriptor::KeyLine> lines_left_new, lines_right_new;
    cv::Mat desc_left_new, desc_right_new;
    std::vector<size_t> ids_left_new, ids_right_new;

    // First, extract new descriptors for this new image
    perform_line_detection_stereo(img_left, img_right, lines_left_new, lines_right_new, desc_left_new, desc_right_new, ids_left_new, ids_right_new);
    rT2 = boost::posix_time::microsec_clock::local_time();
    
    // Our matches temporally
    std::vector<cv::DMatch> matches_ll, matches_rr;
    parallel_for_(cv::Range(0, 2), LambdaBody([&](const cv::Range &range) {
        for (int i = range.start; i < range.end; i++) {
            bool is_left = (i == 0);
            line_robust_match(lines_last[is_left ? cam_id_left : cam_id_right], is_left ? lines_left_new : lines_right_new, line_desc_last[is_left ? cam_id_left : cam_id_right], is_left ? desc_left_new : desc_right_new, is_left ? matches_ll : matches_rr);
        }
    }));
    rT3 = boost::posix_time::microsec_clock::local_time();

    // Get our "good tracks"
    std::vector<cv::line_descriptor::KeyLine> good_left, good_right;
    std::vector<size_t> good_ids_left, good_ids_right;
    cv::Mat good_desc_left, good_desc_right;

    // Points must be of equal size
    assert(lines_last[cam_id_left].size() == lines_last[cam_id_right].size());
    assert(lines_left_new.size() == lines_right_new.size());

    // Count how many we have tracked from the last time
    int num_tracklast = 0;
    
    // Loop through all current left to right points
    // We want to see if any of theses have matches to the previous frame
    // If we have a match new->old then we want to use that ID instead of the new one
    
    for (size_t i = 0; i < lines_left_new.size(); i++) {

        // Loop through all left matches, and find the old "train" id
        int idll = -1;
        for (size_t j = 0; j < matches_ll.size(); j++) {
            if (matches_ll[j].trainIdx == (int)i) {
                idll = matches_ll[j].queryIdx;
            }
        }

        // Loop through all left matches, and find the old "train" id
        int idrr = -1;
        for (size_t j = 0; j < matches_rr.size(); j++) {
            if (matches_rr[j].trainIdx == (int)i) {
                idrr = matches_rr[j].queryIdx;
            }
        }

        // If we found a good stereo track from left to left, and right to right
        // Then lets replace the current ID with the old ID
        // We also check that we are linked to the same past ID value
        if (idll != -1 && idrr != -1 && line_ids_last[cam_id_left][idll] == line_ids_last[cam_id_right][idrr]) {
            good_left.push_back(lines_left_new[i]);
            good_right.push_back(lines_right_new[i]);
            good_desc_left.push_back(desc_left_new.row((int)i));
            good_desc_right.push_back(desc_right_new.row((int)i));
            good_ids_left.push_back(line_ids_last[cam_id_left][idll]);
            good_ids_right.push_back(line_ids_last[cam_id_right][idrr]);
            num_tracklast++;
        } else {
            // Else just append the current feature and its unique ID
            good_left.push_back(lines_left_new[i]);
            good_right.push_back(lines_right_new[i]);
            good_desc_left.push_back(desc_left_new.row((int)i));
            good_desc_right.push_back(desc_right_new.row((int)i));
            good_ids_left.push_back(ids_left_new[i]);
            good_ids_right.push_back(ids_left_new[i]);
        }
    } 
    rT4 = boost::posix_time::microsec_clock::local_time();

    //===================================================================================
    //===================================================================================

    // Update our feature database, with theses new observations
    for (size_t i = 0; i < good_left.size(); i++) {
        // Assert that our IDs are the same
        assert(good_ids_left.at(i) == good_ids_right.at(i));
        // Try to undistort the point
        Eigen::Vector2f start_ln_left = Eigen::Vector2f(good_left.at(i).startPointX, good_left.at(i).startPointY);
        Eigen::Vector2f end_ln_left = Eigen::Vector2f(good_left.at(i).endPointX, good_left.at(i).endPointY);
        Eigen::Vector2f start_ln_right = Eigen::Vector2f(good_right.at(i).startPointX, good_right.at(i).startPointY);
        Eigen::Vector2f end_ln_right = Eigen::Vector2f(good_right.at(i).endPointX, good_right.at(i).endPointY);
        
        Eigen::Vector2f start_ln_left_un = camera_calib.at(cam_id_left)->undistort_f(Eigen::Vector2f(good_left.at(i).startPointX, good_left.at(i).startPointY));
        Eigen::Vector2f end_ln_left_un = camera_calib.at(cam_id_left)->undistort_f(Eigen::Vector2f(good_left.at(i).endPointX, good_left.at(i).endPointY));
        Eigen::Vector2f start_ln_right_un = camera_calib.at(cam_id_right)->undistort_f(Eigen::Vector2f(good_right.at(i).startPointX, good_right.at(i).startPointY));
        Eigen::Vector2f end_ln_right_un = camera_calib.at(cam_id_right)->undistort_f(Eigen::Vector2f(good_right.at(i).endPointX, good_right.at(i).endPointY));
        
        // Append to the database
        line_database->update_line(good_ids_left.at(i), message.timestamp, cam_id_left, start_ln_left, end_ln_left, start_ln_left_un, end_ln_left_un);
        line_database->update_line(good_ids_right.at(i), message.timestamp, cam_id_right, start_ln_right, end_ln_right, start_ln_right_un, end_ln_right_un);
    }


    // Debug info
    // PRINT_DEBUG("LtoL = %d | RtoR = %d | LtoR = %d | good = %d | fromlast = %d\n", (int)matches_ll.size(),
    //       (int)matches_rr.size(),(int)ids_left_new.size(),(int)good_left.size(),num_tracklast);

    // Move forward in time
    //img_last[cam_id_left] = img_left;
    //img_last[cam_id_right] = img_right;
    lines_last[cam_id_left] = good_left;
    lines_last[cam_id_right] = good_right;
    line_ids_last[cam_id_left] = good_ids_left;
    line_ids_last[cam_id_right] = good_ids_right;
    line_desc_last[cam_id_left] = good_desc_left;
    line_desc_last[cam_id_right] = good_desc_right;
    
    rT5 = boost::posix_time::microsec_clock::local_time();

}
*/

void TrackLineDescriptor::perform_line_detection_monocular(const cv::Mat &img0, std::vector<cv::line_descriptor::KeyLine> &lns0, cv::Mat &ln_desc0, std::vector<size_t> &ln_ids0) {

    // Assert that we need line features
    assert(lns0.empty());

    // Extract our line features
    std::vector<cv::line_descriptor::KeyLine> lns0_ext;
    LBD_LSD::LBD_Detect(img0, lns0_ext);

    // For all new lines, extract their descriptors
    cv::Ptr<cv::line_descriptor::BinaryDescriptor> bd = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();

    // Compute BinaryDescriptor
    cv::Mat desc0_ext;;
    bd->compute(img0, lns0_ext, desc0_ext);
    
    for ( int i = 0; i < (int) lns0_ext.size(); i++ )
    {
        if( lns0_ext[i].octave == 0 && lns0_ext[i].lineLength >= 60)
        {
            lns0.push_back( lns0_ext[i] );
            ln_desc0.push_back( desc0_ext.row( i ) );
            size_t temp = ++currid;
            ln_ids0.push_back(temp);
        }
    }
}

/*
void TrackLineDescriptor::perform_line_detection_stereo(const cv::Mat &img0, const cv::Mat &img1, std::vector<cv::line_descriptor::KeyLine> &lns0, std::vector<cv::line_descriptor::KeyLine> &lns1, cv::Mat &ln_desc0, cv::Mat &ln_desc1, std::vector<size_t> &ln_ids0, std::vector<size_t> &ln_ids1) {

    // Assert that we need line features
    assert(lns0.empty());
    assert(lns1.empty());
    
    // Extract our line features
    std::vector<cv::line_descriptor::KeyLine> lns0_ext, lns1_ext;
    cv::Mat desc0_ext, desc1_ext;
    
    // For all new lines, extract their descriptors
    cv::Ptr<cv::line_descriptor::BinaryDescriptor> bd0 = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
    cv::Ptr<cv::line_descriptor::BinaryDescriptor> bd1 = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
    
    parallel_for_(cv::Range(0, 2), LambdaBody([&](const cv::Range &range) {
        for (int i = range.start; i < range.end; i++) {
            bool is_left = (i == 0);
            LBD_LSD::LSD_Detect(is_left ? img0 : img1, is_left ? lns0_ext : lns1_ext);
            (is_left ? bd0 : bd1)->compute(is_left ? img0 : img1, is_left ? lns0_ext : lns1_ext, is_left ? desc0_ext : desc1_ext);
        }
    }));
    
    // Do matching from the left to the right image
    std::vector<cv::DMatch> matches;
    line_robust_match(lns0_ext, lns1_ext, desc0_ext, desc1_ext, matches);
    
    
    for ( int i = 0; i < (int)matches.size(); i++ )
    {
        if( lns0_ext[i].octave == 0 && lns0_ext[i].lineLength >= 30 && lns1_ext[i].octave == 0 && lns1_ext[i].lineLength >= 30) {
            lns0.push_back(lns0_ext[i]);
            lns1.push_back(lns1_ext[i]);
            ln_desc0.push_back(desc0_ext.row(i));
            ln_desc1.push_back(desc1_ext.row(i));
            
            size_t temp = ++currid;
            ln_ids0.push_back(temp);
            ln_ids1.push_back(temp);
        }
    }
}
*/

void TrackLineDescriptor::line_robust_match(std::vector<cv::line_descriptor::KeyLine> &lns0, std::vector<cv::line_descriptor::KeyLine> &lns1, cv::Mat &ln_desc0, cv::Mat &ln_desc1, std::vector<cv::DMatch> &line_matches) {

    int MATCHES_DIST_THRESHOLD = 30;
    
    cv::Ptr<cv::line_descriptor::BinaryDescriptorMatcher> bdm = cv::line_descriptor::BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
    
    std::vector<cv::DMatch> lsd_matches;
    bdm->match(ln_desc0, ln_desc1, lsd_matches);

    std::vector<cv::line_descriptor::KeyLine> good_keylines;
    line_matches.clear();
    for ( int i = 0; i < (int) lsd_matches.size(); i++ ) {
        if( lsd_matches[i].distance < MATCHES_DIST_THRESHOLD ){

        cv::DMatch mt = lsd_matches.at(i);
        cv::line_descriptor::KeyLine line1 =  lns0[mt.queryIdx] ;
        cv::line_descriptor::KeyLine line2 =  lns1[mt.trainIdx] ;
        cv::Point2f serr = line1.getStartPoint() - line2.getStartPoint();
        cv::Point2f eerr = line1.getEndPoint() - line2.getEndPoint();
        if((serr.dot(serr) < 60 * 60) && (eerr.dot(eerr) < 60 * 60)) {
            line_matches.push_back( lsd_matches[i] );
        }
        }
    }
}


void TrackLineDescriptor::line_visualize(const cv::Mat &img0, const cv::Mat &img1, std::vector<cv::line_descriptor::KeyLine> &lns0, std::vector<cv::line_descriptor::KeyLine> &lns1, std::vector<cv::DMatch> &line_matches) {
    
    cv::Mat imageMat0, imageMat1, imageMat2, imageMat3;
    if (img0.channels() != 3){
        cv::cvtColor(img0, imageMat0, cv::COLOR_GRAY2BGR);
        cv::cvtColor(img0, imageMat2, cv::COLOR_GRAY2BGR);
    }
    else{
        imageMat0 = img0;
        imageMat2 = img0;
    }
    
    if (img1.channels() != 3){
        cv::cvtColor(img1, imageMat1, cv::COLOR_GRAY2BGR);
        cv::cvtColor(img1, imageMat3, cv::COLOR_GRAY2BGR);
    }
    else{
        imageMat1 = img1;
        imageMat3 = img1;
    }

    int lowest = 0, highest = 255;
    int range = (highest - lowest) + 1;
    for (int k = 0; k < int(line_matches.size()); ++k) {

        cv::DMatch mt = line_matches[k];

        cv::line_descriptor::KeyLine line1 = lns0[mt.queryIdx];
        cv::line_descriptor::KeyLine line2 = lns1[mt.trainIdx];

        unsigned int r = lowest + int(rand() % range);
        unsigned int g = lowest + int(rand() % range);
        unsigned int b = lowest + int(rand() % range);
        cv::Point startPoint = cv::Point(int(line1.startPointX), int(line1.startPointY));
        cv::Point endPoint = cv::Point(int(line1.endPointX), int(line1.endPointY));
        cv::line(imageMat0, startPoint, endPoint, cv::Scalar(r, g, b),2 ,8);

        cv::Point startPoint2 = cv::Point(int(line2.startPointX), int(line2.startPointY));
        cv::Point endPoint2 = cv::Point(int(line2.endPointX), int(line2.endPointY));
        cv::line(imageMat1, startPoint2, endPoint2, cv::Scalar(r, g, b),2, 8);
        cv::line(imageMat1, startPoint, startPoint2, cv::Scalar(0, 0, 255),1, 8);
        cv::line(imageMat1, endPoint, endPoint2, cv::Scalar(0, 0, 255),1, 8);

    }
    imwrite("images0_line.jpg", imageMat0);
    imwrite("images1_line.jpg", imageMat1);
    
    /* plot matches */
    
    cv::Mat matched_img;
    std::vector<char> lsd_mask( line_matches.size(), 1 );
    cv::line_descriptor::drawLineMatches(imageMat2, lns0,
                                         imageMat3, lns1,
                                         line_matches,
                                         matched_img,
                                         cv::Scalar::all(-1),
                                         cv::Scalar::all(-1), lsd_mask,
                                         cv::line_descriptor::DrawLinesMatchesFlags::DEFAULT);
    imwrite("match_lines.jpg", matched_img);
}






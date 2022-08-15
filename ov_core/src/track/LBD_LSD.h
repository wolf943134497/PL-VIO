/*
 * SPL_VIO
 * Copyright (C) 2022-2023 Yanyu Zhang
 * Copyright (C) 2022-2023 Wei Ren
 */

#ifndef OV_CORE_LBD_LSD_H
#define OV_CORE_LBD_LSD_H

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/features2d/features2d.hpp>
#include <Eigen/Eigen>
#include <memory>
#include <mutex>
#include <vector>

#include <opencv2/line_descriptor.hpp>
#include <opencv2/features2d.hpp>

namespace ov_core {
    
class LBD_LSD {
    
public:
    
static void LBD_Detect(const cv::Mat &img, std::vector<cv::line_descriptor::KeyLine> &keylines) {

    cv::Ptr<cv::line_descriptor::BinaryDescriptor> bd = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();

    // Compute line segment
    bd->detect(img, keylines);
    
}


static void LSD_Detect(const cv::Mat &img, std::vector<cv::line_descriptor::KeyLine> &keylines) {
    
    cv::Ptr<cv::line_descriptor::LSDDetector> lsd = cv::line_descriptor::LSDDetector::createLSDDetector();
    
    // scale: scale factor used in pyramids generation
    // numOctaves: number of octaves inside pyramid
    // detect(img, keylines, scale, numOctaves)
    lsd->detect(img, keylines, 2, 2);

}

};

} // namespace ov_core

#endif  /* OV_CORE_LBD_LSD_H */

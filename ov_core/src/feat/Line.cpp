/*
 * SPL_VIO
 * Copyright (C) 2022-2023 Yanyu Zhang
 * Copyright (C) 2022-2023 Wei Ren
 */

#include "Line.h"

using namespace ov_core;

/**
 * @brief Remove measurements that do not occur at passed timestamps.
 */
void Line::clean_old_measurements(const std::vector<double> &valid_times) {

    // Loop through each of the cameras we have
    for (auto const &pair : timestamps) {

        // Assert that we have all the parts of a measurement
        assert(timestamps[pair.first].size() == startpoint[pair.first].size());
        assert(timestamps[pair.first].size() == endpoint[pair.first].size());
        assert(timestamps[pair.first].size() == startpoint_norm[pair.first].size());
        assert(timestamps[pair.first].size() == endpoint_norm[pair.first].size());

        // Our iterators
        auto it11 = timestamps[pair.first].begin();
        auto it22 = startpoint[pair.first].begin();
        auto it33 = endpoint[pair.first].begin();
        auto it44 = startpoint_norm[pair.first].begin();
        auto it55 = endpoint_norm[pair.first].begin();

        // Loop through measurement times, remove ones that are not in our timestamps
        while (it11 != timestamps[pair.first].end()) {
            if (std::find(valid_times.begin(), valid_times.end(), *it11) == valid_times.end()) {
                it11 = timestamps[pair.first].erase(it11);
                it22 = startpoint[pair.first].erase(it22);
                it33 = endpoint[pair.first].erase(it33);
                it44 = startpoint_norm[pair.first].erase(it44);
                it55 = endpoint_norm[pair.first].erase(it55);
            } else {
                ++it11;
                ++it22;
                ++it33;
                ++it44;
                ++it55;
            }
        }
    }
}
  
/**
 * @brief Remove measurements that occur at the invalid timestamps
 */
void Line::clean_invalid_measurements(const std::vector<double> &invalid_times) {

    // Loop through each of the cameras we have
    for (auto const &pair : timestamps) {

        // Assert that we have all the parts of a measurement
        assert(timestamps[pair.first].size() == startpoint[pair.first].size());
        assert(timestamps[pair.first].size() == endpoint[pair.first].size());
        assert(timestamps[pair.first].size() == startpoint_norm[pair.first].size());
        assert(timestamps[pair.first].size() == endpoint_norm[pair.first].size());

        // Our iterators
        auto it11 = timestamps[pair.first].begin();
        auto it22 = startpoint[pair.first].begin();
        auto it33 = endpoint[pair.first].begin();
        auto it44 = startpoint_norm[pair.first].begin();
        auto it55 = endpoint_norm[pair.first].begin();

        // Loop through measurement times, remove ones that are in our timestamps
        while (it11 != timestamps[pair.first].end()) {
            if (std::find(invalid_times.begin(), invalid_times.end(), *it11) != invalid_times.end()) {
                it11 = timestamps[pair.first].erase(it11);
                it22 = startpoint[pair.first].erase(it22);
                it33 = endpoint[pair.first].erase(it33);
                it44 = startpoint_norm[pair.first].erase(it44);
                it55 = endpoint_norm[pair.first].erase(it55);
            } else {
                ++it11;
                ++it22;
                ++it33;
                ++it44;
                ++it55;
            }
        }
    }
}
  
/**
 * @brief Remove measurements that are older then the specified timestamp.
 */
void Line::clean_older_measurements(double timestamp) {

    // Loop through each of the cameras we have
    for (auto const &pair : timestamps) {

        // Assert that we have all the parts of a measurement
        assert(timestamps[pair.first].size() == startpoint[pair.first].size());
        assert(timestamps[pair.first].size() == endpoint[pair.first].size());
        assert(timestamps[pair.first].size() == startpoint_norm[pair.first].size());
        assert(timestamps[pair.first].size() == endpoint_norm[pair.first].size());

        // Our iterators
        auto it11 = timestamps[pair.first].begin();
        auto it22 = startpoint[pair.first].begin();
        auto it33 = endpoint[pair.first].begin();
        auto it44 = startpoint_norm[pair.first].begin();
        auto it55 = endpoint_norm[pair.first].begin();

        // Loop through measurement times, remove ones that are older then the specified one
        while (it11 != timestamps[pair.first].end()) {
            if (*it11 <= timestamp) {
                it11 = timestamps[pair.first].erase(it11);
                it22 = startpoint[pair.first].erase(it22);
                it33 = endpoint[pair.first].erase(it33);
                it44 = startpoint_norm[pair.first].erase(it44);
                it55 = endpoint_norm[pair.first].erase(it55);
            } else {
                ++it11;
                ++it22;
                ++it33;
                ++it44;
                ++it55;
            }
        }
    }
}

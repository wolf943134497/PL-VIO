/*
 * SPL_VIO
 * Copyright (C) 2022-2023 Yanyu Zhang
 * Copyright (C) 2022-2023 Wei Ren
 */

#ifndef OV_CORE_LINE_DATABASE_H
#define OV_CORE_LINE_DATABASE_H

#include <Eigen/Eigen>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <memory>
#include <mutex>

#include "Line.h"
#include "utils/print.h"

namespace ov_core {
    
class LineDatabase {

public:
    /**
     * @brief Default constructor
     */
    LineDatabase() {}
    
    /**
     * @brief Get a specified line
     */
    std::shared_ptr<Line> get_line(size_t id, bool remove = false) {
    std::lock_guard<std::mutex> lck(mtx);
        if (lines_idlookup.find(id) != lines_idlookup.end()) {
            std::shared_ptr<Line> temp = lines_idlookup.at(id);
            if (remove)
                lines_idlookup.erase(id);
            return temp;
        } else {
            return nullptr;
        }

    }
    
    /**
     * @brief Update a line object
     */
    void update_line(size_t id, double timestamp, size_t cam_id, Eigen::Vector2f p1, Eigen::Vector2f p2, Eigen::Vector2f p1_n, Eigen::Vector2f p2_n) {
        // Find this line using the ID lookup
        std::lock_guard<std::mutex> lck(mtx);
        if (lines_idlookup.find(id) != lines_idlookup.end()) {
            // Get our line
            std::shared_ptr<Line> line = lines_idlookup.at(id);
            // Append this new information to it!
            line->startpoint[cam_id].push_back(p1);
            line->endpoint[cam_id].push_back(p2);
            line->startpoint_norm[cam_id].push_back(p1_n);
            line->endpoint_norm[cam_id].push_back(p2_n);
            line->timestamps[cam_id].push_back(timestamp);
        return;
        }

        // Else we have not found the line, so lets make it be a new one!
        std::shared_ptr<Line> line = std::make_shared<Line>();
        line->lineid = id;
        line->startpoint[cam_id].push_back(p1);
        line->endpoint[cam_id].push_back(p2);
        line->startpoint_norm[cam_id].push_back(p1_n);
        line->endpoint_norm[cam_id].push_back(p2_n);
        line->timestamps[cam_id].push_back(timestamp);

        // Append this new line into our database
        lines_idlookup[id] = line;
    }
    
    /**
     * @brief Get lines that do not have newer measurement then the specified time.
     */
    std::vector<std::shared_ptr<Line>> lines_not_containing_newer(double timestamp, bool remove = false, bool skip_deleted = false) {
        
        // Our vector of lines that do not have measurements after the specified time
        std::vector<std::shared_ptr<Line>> lines_old;

        // Now lets loop through all lines, and just make sure they are not old
        std::lock_guard<std::mutex> lck(mtx);
        for (auto it = lines_idlookup.begin(); it != lines_idlookup.end();) {
            // Skip if already deleted
            if (skip_deleted && (*it).second->to_delete) {
                it++;
                continue;
            }
            // Loop through each camera
            // If we have a measurement greater-than or equal to the specified, this measurement is find
            bool has_newer_measurement = false;
            for (auto const &pair : (*it).second->timestamps) {
                has_newer_measurement = (!pair.second.empty() && pair.second.at(pair.second.size() - 1) >= timestamp);
                if (has_newer_measurement) {
                    break;
                }
            }
            // If it is not being actively tracked, then it is old
            if (!has_newer_measurement) {
                lines_old.push_back((*it).second);
                if (remove)
                    lines_idlookup.erase(it++);
                else
                    it++;
            } else {
                it++;
            }
        }

        // Debugging
        // PRINT_DEBUG("line db size = %u\n", lines_idlookup.size())

        // Return the old lines
        return lines_old;
    }
    
    /**
     * @brief Get lines that has measurements older then the specified time.
     */
    std::vector<std::shared_ptr<Line>> lines_containing_older(double timestamp, bool remove = false, bool skip_deleted = false) {

        // Our vector of old lines
        std::vector<std::shared_ptr<Line>> lines_old;

        // Now lets loop through all lines, and just make sure they are not old
        std::lock_guard<std::mutex> lck(mtx);
        for (auto it = lines_idlookup.begin(); it != lines_idlookup.end();) {
            // Skip if already deleted
            if (skip_deleted && (*it).second->to_delete) {
                it++;
                continue;
            }
            // Loop through each camera
            // Check if we have at least one time older then the requested
            bool found_containing_older = false;
            for (auto const &pair : (*it).second->timestamps) {
                found_containing_older = (!pair.second.empty() && pair.second.at(0) < timestamp);
                if (found_containing_older) {
                    break;
                }
            }
            // If it has an older timestamp, then add it
            if (found_containing_older) {
                lines_old.push_back((*it).second);
                if (remove)
                    lines_idlookup.erase(it++);
                else
                    it++;
            } else {
                it++;
            }
        }

        // Debugging
        // PRINT_DEBUG("line db size = %u\n", lines_idlookup.size())

        // Return the old lines
        return lines_old;
    }
    
    /**
     * @brief Get lines that has measurements at the specified time.
     */    
    std::vector<std::shared_ptr<Line>> lines_containing(double timestamp, bool remove = false, bool skip_deleted = false) {

        // Our vector of old lines
        std::vector<std::shared_ptr<Line>> lines_has_timestamp;

        // Now lets loop through all lines, and just make sure they are not
        std::lock_guard<std::mutex> lck(mtx);
        for (auto it = lines_idlookup.begin(); it != lines_idlookup.end();) {
            // Skip if already deleted
            if (skip_deleted && (*it).second->to_delete) {
                it++;
                continue;
            }
            // Boolean if it has the timestamp
            // Break out if we found a single timestamp that is equal to the specified time
            bool has_timestamp = false;
            for (auto const &pair : (*it).second->timestamps) {
                has_timestamp = (std::find(pair.second.begin(), pair.second.end(), timestamp) != pair.second.end());
                if (has_timestamp) {
                    break;
                }
            }
            // Remove this line if it contains the specified timestamp
            if (has_timestamp) {
                lines_has_timestamp.push_back((*it).second);
                if (remove)
                    lines_idlookup.erase(it++);
                else
                    it++;
            } else {
                it++;
            }
        }

        // Debugging
        // PRINT_DEBUG("line db size = %u\n", lines_idlookup.size())
        // PRINT_DEBUG("return vector = %u\n", lines_has_timestamp.size())

        // Return the lines
        return lines_has_timestamp;
    }
    
    /**
     * @brief This function will delete all lines that have been used up.
     */
    void cleanup() {
        // Loop through all lines
        // int sizebefore = (int)lines_idlookup.size();
        std::lock_guard<std::mutex> lck(mtx);
        for (auto it = lines_idlookup.begin(); it != lines_idlookup.end();) {
            // If delete flag is set, then delete it
            if ((*it).second->to_delete) {
                lines_idlookup.erase(it++);
            } else {
                it++;
            }
        }
        // PRINT_DEBUG("line db = %d -> %d\n", sizebefore, (int)lines_idlookup.size() << std::endl;
    }
    
    /**
     * @brief This function will delete all line measurements that are older then the specified timestamp
     */
    void cleanup_measurements(double timestamp) {
        std::lock_guard<std::mutex> lck(mtx);
        for (auto it = lines_idlookup.begin(); it != lines_idlookup.end();) {
            // Remove the older measurements
            (*it).second->clean_older_measurements(timestamp);
            // Count how many measurements
            int ct_meas = 0;
            for (const auto &pair : (*it).second->timestamps) {
                ct_meas += (int)(pair.second.size());
            }
            // If delete flag is set, then delete it
            if (ct_meas < 1) {
                lines_idlookup.erase(it++);
            } else {
                it++;
            }
        }
    }
  
    /**
     * @brief This function will delete all line measurements that are at the specified timestamp
     */
    void cleanup_measurements_exact(double timestamp) {
        std::lock_guard<std::mutex> lck(mtx);
        std::vector<double> timestamps = {timestamp};
        for (auto it = lines_idlookup.begin(); it != lines_idlookup.end();) {
            // Remove the older measurements
            (*it).second->clean_invalid_measurements(timestamps);
            // Count how many measurements
            int ct_meas = 0;
            for (const auto &pair : (*it).second->timestamps) {
                ct_meas += (int)(pair.second.size());
            }
            // If delete flag is set, then delete it
            if (ct_meas < 1) {
                lines_idlookup.erase(it++);
            } else {
                it++;
            }
        }
    }
     
    /**
     * @brief Returns the size of the line database
     */
    size_t size() {
        std::lock_guard<std::mutex> lck(mtx);
        return lines_idlookup.size();
    }
    
    /**
     * @brief Returns the internal data (should not normally be used)
     */
    std::unordered_map<size_t, std::shared_ptr<Line>> get_internal_data() {
        std::lock_guard<std::mutex> lck(mtx);
        return lines_idlookup;
    }
    
    /**
     * @brief Gets the oldest time in the database
     */
    double get_oldest_timestamp() {
        std::lock_guard<std::mutex> lck(mtx);
        double oldest_time = -1;
        for (auto const &line : lines_idlookup) {
            for (auto const &camtimepair : line.second->timestamps) {
                if (!camtimepair.second.empty() && (oldest_time == -1 || oldest_time < camtimepair.second.at(0))) {
                oldest_time = camtimepair.second.at(0);
                }
            }
        }
        return oldest_time;
    }
    
    /**
     * @brief Will update the passed database with this database's latest line information.
     */
    void append_new_measurements(const std::shared_ptr<LineDatabase> &database) {
        std::lock_guard<std::mutex> lck(mtx);

        // Loop through the other database's internal database
        // int sizebefore = (int)lines_idlookup.size();
        for (const auto &line : database->get_internal_data()) {
            if (lines_idlookup.find(line.first) != lines_idlookup.end()) {

                // For this line, now try to append the new measurement data
                std::shared_ptr<Line> temp = lines_idlookup.at(line.first);
                for (const auto &times : line.second->timestamps) {
                    // Append the whole camera vector is not seen
                    // Otherwise need to loop through each and append
                    size_t cam_id = times.first;
                    if (temp->timestamps.find(cam_id) == temp->timestamps.end()) {
                        
                        temp->timestamps[cam_id] = line.second->timestamps.at(cam_id);
                        
                        temp->startpoint[cam_id] = line.second->startpoint.at(cam_id);
                        temp->endpoint[cam_id] = line.second->endpoint.at(cam_id);
                        temp->startpoint_norm[cam_id] = line.second->startpoint_norm.at(cam_id);
                        temp->endpoint_norm[cam_id] = line.second->endpoint_norm.at(cam_id);
                        
                    } else {
                        auto temp_times = temp->timestamps.at(cam_id);
                        for (size_t i = 0; i < line.second->timestamps.at(cam_id).size(); i++) {
                            double time_to_find = line.second->timestamps.at(cam_id).at(i);
                            if (std::find(temp_times.begin(), temp_times.end(), time_to_find) == temp_times.end()) {
                                temp->timestamps.at(cam_id).push_back(line.second->timestamps.at(cam_id).at(i));
                                temp->startpoint.at(cam_id).push_back(line.second->startpoint.at(cam_id).at(i));
                                temp->endpoint.at(cam_id).push_back(line.second->endpoint.at(cam_id).at(i));
                                temp->startpoint_norm.at(cam_id).push_back(line.second->startpoint_norm.at(cam_id).at(i));
                                temp->endpoint_norm.at(cam_id).push_back(line.second->endpoint_norm.at(cam_id).at(i));
                            }
                        }
                    }
                }

            } else {

                // Else we have not found the line, so lets make it be a new one!
                std::shared_ptr<Line> temp = std::make_shared<Line>();
                temp->lineid = line.second->lineid;
                temp->timestamps = line.second->timestamps;
                temp->startpoint = line.second->startpoint;
                temp->endpoint = line.second->endpoint;
                temp->startpoint_norm = line.second->startpoint_norm;
                temp->endpoint_norm = line.second->endpoint_norm;
                lines_idlookup[line.first] = temp;
            }
        }
         // PRINT_DEBUG("line db = %d -> %d\n", sizebefore, (int)lines_idlookup.size());
    }
    
    
protected:
    /// Mutex lock for our map
    std::mutex mtx;

    /// Our lookup array that allow use to query based on ID
    std::unordered_map<size_t, std::shared_ptr<Line>> lines_idlookup;

};

} // namespace ov_core

#endif /* OV_CORE_LINE_DATABASE_H */

/*
 * SPL_VIO
 * Copyright (C) 2022-2023 Yanyu Zhang
 * Copyright (C) 2022-2023 Wei Ren
 */

#ifndef OV_TYPE_LANDMARK_REP_H
#define OV_TYPE_LANDMARK_REP_H

#include <string>
#include <iostream>
namespace ov_type {

/**
 * @brief Class has useful line representation types
 */
class Line_Landmark_Rep {

public:
  /**
   * @brief What line representation our state can use
   */
  enum Line_Representation {
    CP_LINE,
    UNKNOWN
  };

  /**
   * @brief Returns a string line representation of this enum value.
   * Used to debug print out what the user has selected as the representation.
   * @param feat_representation  Representation we want to check
   * @return String version of the passed enum
   */
  static inline std::string as_string(Line_Representation line_representation) {
    if (line_representation == CP_LINE) {
      return "CP_LINE";
    }
    return "UNKNOWN";
  }

  /**
   * @brief Returns a string representation of this enum value.
   * Used to debug print out what the user has selected as the representation.
   * @param feat_representation String we want to find the enum of
   * @return Representation, will be "unknown" if we coun't parse it
   */
  static inline Line_Representation from_string(const std::string &line_representation) {
    if (line_representation == "CP_LINE") {
      return CP_LINE;
    }
    return UNKNOWN;
  }

  /**
   * @brief Helper function that checks if the passed feature representation is a relative or global
   * @param feat_representation Representation we want to check
   * @return True if it is a relative representation
   */
  static inline bool is_relative_representation(Line_Representation line_representation) {

    return false;
  }

private:
  /**
   * All function in this class should be static.
   * Thus an instance of this class cannot be created.
   */
  Line_Landmark_Rep(){};
};

} // namespace ov_type

#endif // OV_TYPE_LANDMARK_REP_H

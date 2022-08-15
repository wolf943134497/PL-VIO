/*
 * SPL_VIO
 * Copyright (C) 2022-2023 Yanyu Zhang
 * Copyright (C) 2022-2023 Wei Ren
 */

#ifndef OV_CORE_LINEINITIALIZEROPTIONS_H
#define OV_CORE_LINEINITIALIZEROPTIONS_H

#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"

namespace ov_core {

/**
 * @brief Struct which stores all our feature initializer options
 */
struct LineInitializerOptions {

  /// If we should perform 1d triangulation instead of 3d
  bool line_triangulate = true;



  /// Nice print function of what parameters we have loaded
  void print(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr) {
    if (parser != nullptr) {
      parser->parse_config("line_triangulate", line_triangulate, false);
    }
    PRINT_DEBUG("\t- line_triangulate: %d\n", line_triangulate);

  }
};

} // namespace ov_core

#endif // OV_LINE_LINEINITIALIZEROPTIONS_H

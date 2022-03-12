// Copyright 2022 Chen Jun

#ifndef ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include "armor_detector/armor_detector.hpp"

namespace rm_auto_aim
{
class NumberClassifier
{
public:
  NumberClassifier(
    const double & small_height_ratio, const double & large_height_ratio,
    const double & width_ratio, const double & ct, const std::string & template_path);

  void extractNumbers(const cv::Mat & src, std::vector<Armor> & armors);

  void xorClassify(std::vector<Armor> & armors, cv::Mat & xor_show);

  // Height scaling factor
  double height_factor;
  // Small armor width scaling factor
  double small_width_factor;
  // Large armor width scaling factor
  double large_width_factor;
  double similarity_threshold;

private:
  std::map<char, cv::Mat> small_armor_templates_;
  std::map<char, cv::Mat> large_armor_templates_;
};
}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_

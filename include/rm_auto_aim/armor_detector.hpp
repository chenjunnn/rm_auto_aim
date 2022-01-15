// Copyright 2022 Chen Jun

#ifndef RM_AUTO_AIM__ARMOR_DETECTOR_HPP_
#define RM_AUTO_AIM__ARMOR_DETECTOR_HPP_

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/node.hpp>
#include <vector>

namespace rm_auto_aim
{
using Light = cv::RotatedRect;
using Armor = cv::Point[4];

class ArmorDetector
{
public:
  explicit ArmorDetector(rclcpp::Node & node);

  std::vector<Armor> detectArmors(const cv::Mat & img);
  cv::Mat preprocessImage(const cv::Mat & img);
  std::vector<Light> findLights(const cv::Mat & binary_img);

  enum DetectColor { RED, BULE } detect_color;
  struct PreprocessParams
  {
    double hmin, hmax, vmin, vmax;
  } r_params, b_params;
  struct LightParams
  {
    // TODO(chenjun): need to add
  } light_params;

private:
  std::vector<Armor> matchArmors(const std::vector<Light> & lights);
  bool isLight(const Light & light);

  rclcpp::Node & node_;
  std::vector<Light> lights_;
  std::vector<Armor> armors_;
};

}  // namespace rm_auto_aim

#endif  // RM_AUTO_AIM__ARMOR_DETECTOR_HPP_
// Copyright 2022 Chen Jun

#ifndef ARMOR_DETECTOR__ARMOR_DETECTOR_HPP_
#define ARMOR_DETECTOR__ARMOR_DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

#include "auto_aim_interfaces/msg/debug_armors.hpp"
#include "auto_aim_interfaces/msg/debug_lights.hpp"

namespace rm_auto_aim
{
enum Color { RED = 0, BULE = 1 };

enum ArmorType { SMALL = 0, LARGE = 1 };

struct Light : public cv::RotatedRect
{
  Light() = default;
  explicit Light(cv::RotatedRect box);

  Color color;
  cv::Point2f top, bottom;
  double length;
  double width;
  float tilt_angle;
};

struct Armor
{
  Armor() = default;
  Armor(const Light & l1, const Light & l2);

  Light left_light, right_light;
  cv::Point2f center;

  cv::Mat number_img;

  char number;
  float similarity;
  std::string classfication_result;
  ArmorType armor_type;
};

class ArmorDetector
{
public:
  struct LightParams
  {
    // width / height
    double min_ratio;
    double max_ratio;
    // vertical angle
    double max_angle;
  };
  struct ArmorParams
  {
    double min_light_ratio;
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    // horizontal angle
    double max_angle;
  };

  ArmorDetector(
    const int & init_min_l, const Color & init_color, const LightParams & init_l,
    const ArmorParams & init_a);

  int min_lightness;
  Color detect_color;
  LightParams l;
  ArmorParams a;

  // Debug msgs
  auto_aim_interfaces::msg::DebugLights debug_lights;
  auto_aim_interfaces::msg::DebugArmors debug_armors;

  cv::Mat preprocessImage(const cv::Mat & rbg_img);

  std::vector<Light> findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img);

  std::vector<Armor> matchLights(const std::vector<Light> & lights);

private:
  bool isLight(const Light & light);

  bool containLight(
    const Light & light_1, const Light & light_2, const std::vector<Light> & lights);

  bool isArmor(Armor & armor);
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_DETECTOR_HPP_

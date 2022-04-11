// Copyright 2022 Chen Jun

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <vector>

#include "armor_detector/armor_detector.hpp"
#include "auto_aim_interfaces/msg/debug_armor.hpp"
#include "auto_aim_interfaces/msg/debug_light.hpp"

namespace rm_auto_aim
{
Light::Light(cv::RotatedRect box) : cv::RotatedRect(box)
{
  cv::Point2f p[4];
  box.points(p);
  std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });
  top = (p[0] + p[1]) / 2;
  bottom = (p[2] + p[3]) / 2;

  length = cv::norm(top - bottom);
  width = cv::norm(p[0] - p[1]);

  tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
  tilt_angle = tilt_angle / CV_PI * 180;
}

Armor::Armor(const Light & l1, const Light & l2)
{
  if (l1.center.x < l2.center.x) {
    left_light = l1, right_light = l2;
  } else {
    left_light = l2, right_light = l1;
  }
  center = (left_light.center + right_light.center) / 2;
}

ArmorDetector::ArmorDetector(
  const int & init_min_l, const Color & init_color, const LightParams & init_l,
  const ArmorParams & init_a)
: min_lightness(init_min_l), detect_color(init_color), l(init_l), a(init_a)
{
}

cv::Mat ArmorDetector::preprocessImage(const cv::Mat & rgb_img)
{
  cv::Mat hsv_img;
  cv::cvtColor(rgb_img, hsv_img, cv::COLOR_RGB2HLS);

  cv::Mat binary_img;
  cv::inRange(hsv_img, cv::Scalar(0, min_lightness, 0), cv::Scalar(180, 255, 255), binary_img);

  return binary_img;
}

std::vector<Light> ArmorDetector::findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img)
{
  using std::vector;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  vector<Light> lights;
  this->debug_lights.data.clear();

  for (const auto & contour : contours) {
    if (contour.size() < 5) continue;

    auto r_rect = cv::minAreaRect(contour);
    auto light = Light(r_rect);

    if (isLight(light)) {
      int sum_r = 0;
      int sum_b = 0;
      for (auto & point : contour) {
        sum_r += rbg_img.at<cv::Vec3b>(point.y, point.x)[0];
        sum_b += rbg_img.at<cv::Vec3b>(point.y, point.x)[2];
      }
      // Sum of red pixels > sum of blue pixels ?
      light.color = sum_r > sum_b ? Color::RED : Color::BULE;
      lights.emplace_back(light);
    }
  }

  return lights;
}

bool ArmorDetector::isLight(const Light & light)
{
  // The ratio of light (short side / long side)
  float ratio = light.width / light.length;
  bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;

  bool angle_ok = light.tilt_angle < l.max_angle;

  bool is_light = ratio_ok && angle_ok;

  // Fill in debug information
  auto_aim_interfaces::msg::DebugLight light_data;
  light_data.center_x = light.center.x;
  light_data.ratio = ratio;
  light_data.angle = light.tilt_angle;
  light_data.is_light = is_light;
  this->debug_lights.data.emplace_back(light_data);

  return is_light;
}

std::vector<Armor> ArmorDetector::matchLights(const std::vector<Light> & lights)
{
  std::vector<Armor> armors;
  this->debug_armors.data.clear();

  // Loop all the pairing of lights
  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
      if (light_1->color != detect_color || light_2->color != detect_color) continue;

      if (containLight(*light_1, *light_2, lights)) {
        continue;
      }
      auto armor = Armor(*light_1, *light_2);
      if (isArmor(armor)) {
        armors.emplace_back(armor);
      }
    }
  }

  return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights
bool ArmorDetector::containLight(
  const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
{
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);

  for (const auto & test_light : lights) {
    if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

    if (
      bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
      bounding_rect.contains(test_light.center)) {
      return true;
    }
  }

  return false;
}

bool ArmorDetector::isArmor(Armor & armor)
{
  Light light_1 = armor.left_light;
  Light light_2 = armor.right_light;
  // Ratio of the length of 2 lights (short side / long side)
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                             : light_2.length / light_1.length;
  bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

  // Distance between the center of 2 lights (unit : light length)
  float avg_light_length = (light_1.length + light_2.length) / 2;
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok = (a.min_small_center_distance < center_distance &&
                             center_distance < a.max_small_center_distance) ||
                            (a.min_large_center_distance < center_distance &&
                             center_distance < a.max_large_center_distance);

  // Angle of light center connection
  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  bool angle_ok = angle < a.max_angle;

  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;
  armor.armor_type = center_distance > a.min_large_center_distance ? LARGE : SMALL;
  // Fill in debug information
  auto_aim_interfaces::msg::DebugArmor armor_data;
  armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
  armor_data.light_ratio = light_length_ratio;
  armor_data.center_distance = center_distance;
  armor_data.angle = angle;
  armor_data.is_armor = is_armor;
  armor_data.armor_type = (armor.armor_type == SMALL);
  this->debug_armors.data.emplace_back(armor_data);

  return is_armor;
}

}  // namespace rm_auto_aim

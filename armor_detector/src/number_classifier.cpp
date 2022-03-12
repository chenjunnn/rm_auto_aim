// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#include "armor_detector/number_classifier.hpp"

#include <pstl/glue_algorithm_defs.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include "armor_detector/armor_detector.hpp"

namespace rm_auto_aim
{
NumberClassifier::NumberClassifier(
  const double & hf, const double & swf, const double & lwf, const double & ct,
  const std::string & template_path)
: height_factor(hf), small_width_factor(swf), large_width_factor(lwf), similarity_threshold(ct)
{
  small_armor_templates_ = {
    {'2', cv::imread(template_path + "2.png", cv::IMREAD_GRAYSCALE)},
    {'3', cv::imread(template_path + "3.png", cv::IMREAD_GRAYSCALE)},
    {'4', cv::imread(template_path + "4.png", cv::IMREAD_GRAYSCALE)},
    {'5', cv::imread(template_path + "5.png", cv::IMREAD_GRAYSCALE)},
    {'6', cv::imread(template_path + "outpost.png", cv::IMREAD_GRAYSCALE)},
  };
  large_armor_templates_ = {
    {'1', cv::imread(template_path + "1.png", cv::IMREAD_GRAYSCALE)},
    {'7', cv::imread(template_path + "guard.png", cv::IMREAD_GRAYSCALE)},
    {'8', cv::imread(template_path + "base.png", cv::IMREAD_GRAYSCALE)},
  };
}

void NumberClassifier::extractNumbers(const cv::Mat & src, std::vector<Armor> & armors)
{
  for (auto & armor : armors) {
    auto width_factor = armor.armor_type == LARGE ? large_width_factor : small_width_factor;
    // Scaling height
    auto left_height_diff = armor.left_light.bottom - armor.left_light.top;
    auto right_height_diff = armor.right_light.bottom - armor.right_light.top;

    auto left_center = (armor.left_light.top + armor.left_light.bottom) / 2;
    auto right_center = (armor.right_light.top + armor.right_light.bottom) / 2;

    auto top_left = left_center - left_height_diff / 2 * height_factor;
    auto top_right = right_center - right_height_diff / 2 * height_factor;
    auto bottom_left = left_center + left_height_diff / 2 * height_factor;
    auto bottom_right = right_center + right_height_diff / 2 * height_factor;

    // Scaling width
    auto top_width_diff = armor.right_light.top - armor.left_light.top;
    auto bottom_width_diff = armor.right_light.bottom - armor.left_light.bottom;

    auto top_center = (top_left + top_right) / 2;
    auto bottom_center = (bottom_left + bottom_right) / 2;

    top_left = top_center - top_width_diff / 2 * width_factor;
    top_right = top_center + top_width_diff / 2 * width_factor;
    bottom_left = bottom_center - bottom_width_diff / 2 * width_factor;
    bottom_right = bottom_center + bottom_width_diff / 2 * width_factor;

    cv::Point2f number_vertices[4] = {bottom_left, top_left, top_right, bottom_right};

    const auto output_size = armor.armor_type == LARGE ? cv::Size(28, 28) : cv::Size(20, 28);
    cv::Point2f target_vertices[4] = {
      cv::Point(0, output_size.height - 1),
      cv::Point(0, 0),
      cv::Point(output_size.width - 1, 0),
      cv::Point(output_size.width - 1, output_size.height - 1),
    };

    auto rotation_matrix = cv::getPerspectiveTransform(number_vertices, target_vertices);

    cv::Mat number_image;
    cv::warpPerspective(src, number_image, rotation_matrix, output_size);

    cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);

    cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    armor.number_img = number_image;
  }
}

void NumberClassifier::xorClassify(std::vector<Armor> & armors, cv::Mat & xor_show)
{
  double full_mat_sum;
  std::map<char, cv::Mat> templates;
  cv::Mat xor_result;
  std::vector<cv::Mat> all_xor_results;

  for (auto & armor : armors) {
    if (armor.armor_type == SMALL) {
      full_mat_sum = 20 * 28 * 255;
      templates = small_armor_templates_;
    } else {
      full_mat_sum = 28 * 28 * 255;
      templates = large_armor_templates_;
    }
    armor.similarity = 0;
    std::vector<cv::Mat> xor_results;
    for (const auto & number_template : templates) {
      cv::bitwise_xor(armor.number_img, number_template.second, xor_result);
      xor_results.emplace_back(xor_result.clone());

      double diff_sum = cv::sum(xor_result)[0];
      double similarity = 1 - diff_sum / full_mat_sum;

      if (similarity > armor.similarity) {
        armor.similarity = similarity;
        armor.number = number_template.first;
      }
    }
    std::stringstream result_ss;
    result_ss << armor.number << ":_" << std::fixed << std::setprecision(1)
              << armor.similarity * 100.0 << "%";
    armor.classfication_result = result_ss.str();

    cv::Mat vertical_concat;
    cv::vconcat(xor_results, vertical_concat);
    all_xor_results.emplace_back(vertical_concat.clone());
  }

  cv::hconcat(all_xor_results, xor_show);

  armors.erase(
    std::remove_if(
      armors.begin(), armors.end(),
      [this](const Armor & armor) { return armor.similarity < similarity_threshold; }),
    armors.end());
}

}  // namespace rm_auto_aim

// Copyright 2022 Chen Jun

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

namespace rm_auto_aim
{
NumberClassifier::NumberClassifier(
  const double & height_ratio, const double & width_ratio, const double & c_t,
  const std::string & template_path)
: hr(height_ratio), wr(width_ratio), confidence_threshold(c_t)
{
  templates_ = {
    {'2', cv::imread(template_path + "2.png", cv::IMREAD_GRAYSCALE)},
    {'3', cv::imread(template_path + "3.png", cv::IMREAD_GRAYSCALE)},
    {'4', cv::imread(template_path + "4.png", cv::IMREAD_GRAYSCALE)},
    {'5', cv::imread(template_path + "5.png", cv::IMREAD_GRAYSCALE)},
  };
}

void NumberClassifier::extractNumbers(const cv::Mat & src, std::vector<Armor> & armors)
{
  for (auto & armor : armors) {
    auto top_width_diff = armor.right_light.top - armor.left_light.top;
    auto bottom_width_diff = armor.right_light.bottom - armor.left_light.bottom;
    auto left_height_diff = armor.left_light.bottom - armor.left_light.top;
    auto right_height_diff = armor.right_light.bottom - armor.right_light.top;

    // The order is bottomLeft, topLeft, topRight, bottomRight
    cv::Point2f number_vertices[4] = {
      armor.center + left_height_diff * hr - bottom_width_diff * wr,
      armor.center - left_height_diff * hr - top_width_diff * wr,
      armor.center - right_height_diff * hr + top_width_diff * wr,
      armor.center + right_height_diff * hr + bottom_width_diff * wr};

    const auto output_size = cv::Size(20, 28);
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

void NumberClassifier::xorClassify(std::vector<Armor> & armors, cv::Mat & xor_all)
{
  double full_mat_sum = 20 * 28 * 255;
  cv::Mat xor_result;
  std::vector<cv::Mat> xor_results;

  for (auto & armor : armors) {
    armor.similarity = 0;
    for (const auto & id_template : templates_) {
      cv::bitwise_xor(armor.number_img, id_template.second, xor_result);
      xor_results.emplace_back(xor_result.clone());

      double diff_sum = cv::sum(xor_result)[0];
      double similarity = 1 - diff_sum / full_mat_sum;

      if (similarity > armor.similarity) {
        armor.similarity = similarity;
        armor.number = id_template.first;
      }
    }

    std::stringstream result_ss;
    result_ss << armor.number << ":_" << std::fixed << std::setprecision(1)
              << armor.similarity * 100.0 << "%";
    armor.classfication_result = result_ss.str();
  }

  cv::vconcat(xor_results, xor_all);

  armors.erase(
    std::remove_if(
      armors.begin(), armors.end(),
      [this](const Armor & armor) { return armor.similarity < confidence_threshold; }),
    armors.end());
}

}  // namespace rm_auto_aim

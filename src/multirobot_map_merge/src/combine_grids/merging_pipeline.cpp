#include <combine_grids/grid_compositor.h>
#include <combine_grids/grid_warper.h>
#include <combine_grids/merging_pipeline.h>
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>

#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>

#include "estimation_internal.h"

namespace combine_grids
{
bool MergingPipeline::estimateTransforms(FeatureType feature_type,
                                         double confidence)
{
  std::vector<cv::detail::ImageFeatures> image_features;
  std::vector<cv::detail::MatchesInfo> pairwise_matches;
  std::vector<cv::detail::CameraParams> transforms;
  std::vector<int> good_indices;
  auto finder = internal::chooseFeatureFinder(feature_type);
  cv::Ptr<cv::detail::FeaturesMatcher> matcher =
      cv::makePtr<cv::detail::AffineBestOf2NearestMatcher>();
  cv::Ptr<cv::detail::Estimator> estimator =
      cv::makePtr<cv::detail::AffineBasedEstimator>();
  cv::Ptr<cv::detail::BundleAdjusterBase> adjuster =
      cv::makePtr<cv::detail::BundleAdjusterAffinePartial>();

  if (images_.empty()) {
    return true;
  }

  static rclcpp::Logger logger = rclcpp::get_logger("estimateTransforms");
  RCLCPP_DEBUG(logger, "computing features");
  image_features.reserve(images_.size());
  for (const cv::Mat& image : images_) {
    image_features.emplace_back();
    if (!image.empty()) {
#if CV_VERSION_MAJOR >= 4
      cv::detail::computeImageFeatures(finder, image, image_features.back());
#else
      (*finder)(image, image_features.back());
#endif
    }
  }
  finder = {};

  RCLCPP_DEBUG(logger, "pairwise matching features");
  (*matcher)(image_features, pairwise_matches);
  matcher = {};

#ifndef NDEBUG
  internal::writeDebugMatchingInfo(images_, image_features, pairwise_matches);
#endif

  good_indices = cv::detail::leaveBiggestComponent(
      image_features, pairwise_matches, static_cast<float>(confidence));

  if (good_indices.size() == 1) {
    transforms_.clear();
    transforms_.resize(images_.size());

    for (size_t i = 0; i < images_.size(); ++i) {
      if (!images_[i].empty()) {
        transforms_[i] = cv::Mat::eye(3, 3, CV_64F);
        break;
      }
    }
    return true;
  }

  RCLCPP_DEBUG(logger, "calculating transforms in global reference frame");
  if (!(*estimator)(image_features, pairwise_matches, transforms)) {
    return false;
  }

  for (auto& transform : transforms) {
    transform.R.convertTo(transform.R, CV_32F);
  }
  RCLCPP_DEBUG(logger, "optimizing global transforms");
  adjuster->setConfThresh(confidence);
  if (!(*adjuster)(image_features, pairwise_matches, transforms)) {
    RCLCPP_WARN(logger, "Bundle adjusting failed. Could not estimate transforms.");
    return false;
  }

  transforms_.clear();
  transforms_.resize(images_.size());
  size_t i = 0;
  for (auto& j : good_indices) {
    transforms[i].R.convertTo(transforms_[static_cast<size_t>(j)], CV_64F);
    ++i;
  }

  return true;
}

static inline bool isIdentity(const cv::Mat& matrix)
{
  if (matrix.empty()) {
    return false;
  }
  cv::MatExpr diff = matrix != cv::Mat::eye(matrix.size(), matrix.type());
  return cv::countNonZero(diff) == 0;
}

nav_msgs::msg::OccupancyGrid::SharedPtr MergingPipeline::composeGrids()
{
  rcpputils::check_true(images_.size() == transforms_.size());
  rcpputils::check_true(images_.size() == grids_.size());
  static rclcpp::Logger logger = rclcpp::get_logger("composeGrids");

  if (images_.empty()) {
    return nullptr;
  }

  RCLCPP_DEBUG(logger, "warping grids");
  internal::GridWarper warper;
  std::vector<cv::Mat> imgs_warped;
  imgs_warped.reserve(images_.size());
  std::vector<cv::Rect> rois;
  rois.reserve(images_.size());

  for (size_t i = 0; i < images_.size(); ++i) {
    if (!transforms_[i].empty() && !images_[i].empty()) {
      imgs_warped.emplace_back();
      rois.emplace_back(
          warper.warp(images_[i], transforms_[i], imgs_warped.back()));
    }
  }

  if (imgs_warped.empty()) {
    return nullptr;
  }

  RCLCPP_DEBUG(logger, "compositing result grid");
  nav_msgs::msg::OccupancyGrid::SharedPtr result;
  internal::GridCompositor compositor;
  result = compositor.compose(imgs_warped, rois);

  float any_resolution = 0.0;
  for (size_t i = 0; i < transforms_.size(); ++i) {
    if (isIdentity(transforms_[i])) {
      result->info.resolution = grids_[i]->info.resolution;
      break;
    }
    if (grids_[i]) {
      any_resolution = grids_[i]->info.resolution;
    }
  }
  if (result->info.resolution <= 0.f) {
    result->info.resolution = any_resolution;
  }

  result->info.origin.position.x =
      -(result->info.width / 2.0) * double(result->info.resolution);
  result->info.origin.position.y =
      -(result->info.height / 2.0) * double(result->info.resolution);
  result->info.origin.orientation.w = 1.0;

  return result;
}

std::vector<geometry_msgs::msg::Transform> MergingPipeline::getTransforms() const
{
  std::vector<geometry_msgs::msg::Transform> result;
  result.reserve(transforms_.size());

  for (auto& transform : transforms_) {
    if (transform.empty()) {
      result.emplace_back();
      continue;
    }

    rcpputils::require_true(transform.type() == CV_64F);
    geometry_msgs::msg::Transform ros_transform;
    ros_transform.translation.x = transform.at<double>(0, 2);
    ros_transform.translation.y = transform.at<double>(1, 2);
    ros_transform.translation.z = 0.;

    double a = transform.at<double>(0, 0);
    double b = transform.at<double>(1, 0);
    ros_transform.rotation.w = std::sqrt(2. + 2. * a) * 0.5;
    ros_transform.rotation.x = 0.;
    ros_transform.rotation.y = 0.;
    ros_transform.rotation.z = std::copysign(std::sqrt(2. - 2. * a) * 0.5, b);

    result.push_back(ros_transform);
  }

  return result;
}

}

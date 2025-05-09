#ifndef ESTIMATION_INTERNAL_H_
#define ESTIMATION_INTERNAL_H_

#include <combine_grids/merging_pipeline.h>

#include <iostream>
#include <cassert>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/stitching/detail/matchers.hpp>

#ifdef HAVE_OPENCV_XFEATURES2D
#include <opencv2/xfeatures2d/nonfree.hpp>
#endif

namespace combine_grids
{
namespace internal
{
#if CV_VERSION_MAJOR >= 4

static inline cv::Ptr<cv::Feature2D> chooseFeatureFinder(FeatureType type)
{
  switch (type) {
    case FeatureType::AKAZE:
      return cv::AKAZE::create();
    case FeatureType::ORB:
      return cv::ORB::create();
    case FeatureType::SURF:
#ifdef HAVE_OPENCV_XFEATURES2D
      return cv::xfeatures2d::SURF::create();
#else
      return cv::AKAZE::create();
#endif
  }

  assert(false);
  return {};
}

#else

static inline cv::Ptr<cv::detail::FeaturesFinder>
chooseFeatureFinder(FeatureType type)
{
  switch (type) {
    case FeatureType::AKAZE:
      return cv::makePtr<cv::detail::AKAZEFeaturesFinder>();
    case FeatureType::ORB:
      return cv::makePtr<cv::detail::OrbFeaturesFinder>();
    case FeatureType::SURF:
      return cv::makePtr<cv::detail::SurfFeaturesFinder>();
  }

  assert(false);
  return {};
}

#endif

static inline void writeDebugMatchingInfo(
    const std::vector<cv::Mat>& images,
    const std::vector<cv::detail::ImageFeatures>& image_features,
    const std::vector<cv::detail::MatchesInfo>& pairwise_matches)
{
  for (auto& match_info : pairwise_matches) {
    if (match_info.H.empty() ||
        match_info.src_img_idx >= match_info.dst_img_idx) {
      continue;
    }
    std::cout << match_info.src_img_idx << " " << match_info.dst_img_idx
              << std::endl
              << "features: "
              << image_features[size_t(match_info.src_img_idx)].keypoints.size()
              << " "
              << image_features[size_t(match_info.dst_img_idx)].keypoints.size()
              << std::endl
              << "matches: " << match_info.matches.size() << std::endl
              << "inliers: " << match_info.num_inliers << std::endl
              << "inliers/matches ratio: "
              << match_info.num_inliers / double(match_info.matches.size())
              << std::endl
              << "confidence: " << match_info.confidence << std::endl
              << match_info.H << std::endl;
    cv::Mat img;

    cv::drawMatches(images[size_t(match_info.src_img_idx)],
                    image_features[size_t(match_info.src_img_idx)].keypoints,
                    images[size_t(match_info.dst_img_idx)],
                    image_features[size_t(match_info.dst_img_idx)].keypoints,
                    match_info.matches, img);
    cv::imwrite(std::to_string(match_info.src_img_idx) + "_" +
                    std::to_string(match_info.dst_img_idx) + "_matches.png",
                img);
 
    cv::drawMatches(
        images[size_t(match_info.src_img_idx)],
        image_features[size_t(match_info.src_img_idx)].keypoints,
        images[size_t(match_info.dst_img_idx)],
        image_features[size_t(match_info.dst_img_idx)].keypoints,
        match_info.matches, img, cv::Scalar::all(-1), cv::Scalar::all(-1),
        *reinterpret_cast<const std::vector<char>*>(&match_info.inliers_mask));
    cv::imwrite(std::to_string(match_info.src_img_idx) + "_" +
                    std::to_string(match_info.dst_img_idx) +
                    "_matches_inliers.png",
                img);
  }
}

}  
} 

#endif 

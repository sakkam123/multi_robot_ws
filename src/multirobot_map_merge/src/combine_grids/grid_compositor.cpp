#include <combine_grids/grid_compositor.h>
#include <opencv2/stitching/detail/util.hpp>
#include <rcpputils/asserts.hpp>

namespace combine_grids
{
namespace internal
{
nav_msgs::msg::OccupancyGrid::SharedPtr GridCompositor::compose(
    const std::vector<cv::Mat>& grids, const std::vector<cv::Rect>& rois)
{
  rcpputils::require_true(grids.size() == rois.size());

  nav_msgs::msg::OccupancyGrid::SharedPtr result_grid(new nav_msgs::msg::OccupancyGrid());

  std::vector<cv::Point> corners;
  corners.reserve(grids.size());
  std::vector<cv::Size> sizes;
  sizes.reserve(grids.size());
  for (auto& roi : rois) {
    corners.push_back(roi.tl());
    sizes.push_back(roi.size());
  }
  cv::Rect dst_roi = cv::detail::resultRoi(corners, sizes);

  result_grid->info.width = static_cast<uint>(dst_roi.width);
  result_grid->info.height = static_cast<uint>(dst_roi.height);
  result_grid->data.resize(static_cast<size_t>(dst_roi.area()), -1);
  cv::Mat result(dst_roi.size(), CV_8S, result_grid->data.data());

  for (size_t i = 0; i < grids.size(); ++i) {
    cv::Rect roi = cv::Rect(corners[i] - dst_roi.tl(), sizes[i]);
    cv::Mat result_roi(result, roi);
    cv::Mat warped_signed(grids[i].size(), CV_8S, const_cast<uchar*>(grids[i].ptr()));
    cv::max(result_roi, warped_signed, result_roi);
  }

  return result_grid;
}

}  
} 


#include <combine_grids/grid_warper.h>
#include <opencv2/stitching/detail/warpers.hpp>
#include <rcpputils/asserts.hpp>

namespace combine_grids
{
namespace internal
{
cv::Rect GridWarper::warp(const cv::Mat& grid, const cv::Mat& transform,
                          cv::Mat& warped_grid)
{
  rcpputils::require_true(transform.type() == CV_64F);
  cv::Mat H;
  invertAffineTransform(transform.rowRange(0, 2), H);
  cv::Rect roi = warpRoi(grid, H);
  H.at<double>(0, 2) -= roi.tl().x;
  H.at<double>(1, 2) -= roi.tl().y;
  warpAffine(grid, warped_grid, H, roi.size(), cv::INTER_NEAREST,
             cv::BORDER_CONSTANT,
             cv::Scalar::all(255));
  rcpputils::assert_true(roi.size() == warped_grid.size());
  return roi;
}

cv::Rect GridWarper::warpRoi(const cv::Mat& grid, const cv::Mat& transform)
{
  cv::Ptr<cv::detail::PlaneWarper> warper =
      cv::makePtr<cv::detail::PlaneWarper>();
  cv::Mat H;
  transform.convertTo(H, CV_32F);
  cv::Mat T = cv::Mat::zeros(3, 1, CV_32F);
  H.colRange(2, 3).rowRange(0, 2).copyTo(T.rowRange(0, 2));
  cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
  H.colRange(0, 2).copyTo(R.rowRange(0, 2).colRange(0, 2));
  return warper->warpRoi(grid.size(), cv::Mat::eye(3, 3, CV_32F), R, T);
}

}
} 


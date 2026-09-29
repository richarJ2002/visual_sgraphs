/*!
 * @file            private_functions.h
 *
 * @brief           Declares module-internal helpers shared between the
 *                  ORBextractor translation units.
 *
 * @note            These helpers were file-static in ORBextractor.cc;
 *                  external linkage here is module-internal only. Names
 *                  are kept verbatim (identifier renaming is a separate step).
 */

#ifndef VS_GRAPHS_CORE_ORBEXTRACTOR_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_ORBEXTRACTOR_PRIVATE_FUNCTIONS_H

#include "ORBextractor.h"
#include "ORBextractorStatus.h"

#include <opencv2/core/core.hpp>

#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{

/*! @brief Side of the square patch the ORB descriptor samples, in pixels. */
inline constexpr int PATCH_SIZE = 31;

/*! @brief Radius of the circular patch that sets a keypoint orientation, in
 *         pixels (PATCH_SIZE / 2). */
inline constexpr int HALF_PATCH_SIZE = 15;

/*! @brief Image border, in pixels, kept free of keypoints so that every
 *         patch fits inside the image. */
inline constexpr int EDGE_THRESHOLD = 19;

/*!
 * @brief        Computes the dominant orientation of a patch.
 */
[[nodiscard]] ORBextractorStatus
    computeIntensityCentroidAngle(const cv::Mat          &image_in,
                                  cv::Point2f             point_in,
                                  const std::vector<int> &maximumU_in,
                                  float &intensityCentroidAngle_out);

/*!
 * @brief        Computes the ORB descriptor of one keypoint.
 */
[[nodiscard]] ORBextractorStatus
    computeOrbDescriptor(const cv::KeyPoint &kpt_in,
                         const cv::Mat      &image_in,
                         const cv::Point    *p_briefPattern_in,
                         unsigned char      *p_descriptor_inout);

/*!
 * @brief        Assigns orientations to all keypoints.
 */
[[nodiscard]] ORBextractorStatus
    computeOrientation(const cv::Mat             &image_in,
                       std::vector<cv::KeyPoint> &keypoints_in,
                       const std::vector<int>    &orientationMaximumOffset_in);

/*!
 * @brief        Orders octree nodes by keypoint count, descending.
 */
bool compareNodes(std::pair<int, ExtractorNode *> &e1_in,
                  std::pair<int, ExtractorNode *> &e2_in);

/*!
 * @brief        Computes descriptors for all keypoints.
 */
[[nodiscard]] ORBextractorStatus
    computeDescriptors(const cv::Mat                &image_in,
                       std::vector<cv::KeyPoint>    &keypoints_in,
                       cv::Mat                      &descriptors_out,
                       const std::vector<cv::Point> &briefPattern_in);

} // namespace core
} // namespace vs_graphs

#endif /* VS_GRAPHS_CORE_ORBEXTRACTOR_PRIVATE_FUNCTIONS_H */

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

#include <opencv2/core/core.hpp>

#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief        Computes the dominant orientation of a patch.
 */
float computeIntensityCentroidAngle(const cv::Mat          &image_in,
                                    cv::Point2f             point_in,
                                    const std::vector<int> &maximumU_in);

/*!
 * @brief        Computes the ORB descriptor of one keypoint.
 */
void computeOrbDescriptor(const cv::KeyPoint &kpt_in,
                          const cv::Mat      &image_in,
                          const cv::Point    *p_briefPattern_in,
                          unsigned char      *p_descriptor_inout);

/*!
 * @brief        Assigns orientations to all keypoints.
 */
void computeOrientation(const cv::Mat             &image_in,
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
void computeDescriptors(const cv::Mat                &image_in,
                        std::vector<cv::KeyPoint>    &keypoints_in,
                        cv::Mat                      &descriptors_out,
                        const std::vector<cv::Point> &briefPattern_in);

} // namespace core
} // namespace vs_graphs

#endif /* VS_GRAPHS_CORE_ORBEXTRACTOR_PRIVATE_FUNCTIONS_H */

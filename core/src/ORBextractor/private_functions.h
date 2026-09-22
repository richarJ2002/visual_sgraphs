/*!
 * @file            private_functions.h
 *
 * @brief           Declares module-internal helpers shared between the
 *                  ORBextractor translation units.
 *
 * @note            These helpers were file-static in ORBextractor.cc;
 *                  external linkage here is module-internal only. Names
 *                  are kept verbatim (WP-01C renaming is a separate step).
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
float IC_Angle(const cv::Mat &image,
               cv::Point2f pt,
               const std::vector<int> &u_max);

/*!
 * @brief        Computes the ORB descriptor of one keypoint.
 */
void computeOrbDescriptor(const cv::KeyPoint &kpt,
                          const cv::Mat &img,
                          const cv::Point *briefPattern,
                          unsigned char *desc);

/*!
 * @brief        Assigns orientations to all keypoints.
 */
void computeOrientation(const cv::Mat &image,
                        std::vector<cv::KeyPoint> &keypoints,
                        const std::vector<int> &orientationMaxOffset);

/*!
 * @brief        Orders octree nodes by keypoint count, descending.
 */
bool compareNodes(std::pair<int, ExtractorNode *> &e1,
                  std::pair<int, ExtractorNode *> &e2);

/*!
 * @brief        Computes descriptors for all keypoints.
 */
void computeDescriptors(const cv::Mat &image,
                        std::vector<cv::KeyPoint> &keypoints,
                        cv::Mat &descriptors,
                        const std::vector<cv::Point> &briefPattern);

} // namespace core
} // namespace vs_graphs

#endif /* VS_GRAPHS_CORE_ORBEXTRACTOR_PRIVATE_FUNCTIONS_H */

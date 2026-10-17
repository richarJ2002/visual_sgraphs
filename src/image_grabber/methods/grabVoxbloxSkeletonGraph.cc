/*!
 * @file            grabVoxbloxSkeletonGraph.cc
 *
 * @brief           Hands the inertial frontend's sparse graph to SLAM.
 *
 * @date            09/10/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
/* None */

#include "image_grabber/objects/ImageGrabberClass.h"

namespace vs_graphs::image_grabber
{

void ImageGrabber::grabVoxbloxSkeletonGraph(
    const visualization_msgs::msg::MarkerArray &msgSkeletonGraph_in)
{
    /*!
     * Pass the skeleton graph to a buffer to be processed by the
     * SemanticSegmentation thread
     */
    vs_graphs::common::setVoxbloxSkeletonCluster(msgSkeletonGraph_in);
}
} /* namespace vs_graphs::image_grabber */

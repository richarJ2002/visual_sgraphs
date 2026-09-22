/*!
 * @file         construct.cc
 *
 * @brief        Implements FrameDrawer::FrameDrawer declared in FrameDrawer.h.
 *
 * @note         Deviates from CPP_CODING_STANDARD §5.6 (constructor bodies
 *               in the header): the body names Tracking::SYSTEM_NOT_READY,
 *               but FrameDrawer.h and Tracking.h include each other, so the
 *               body cannot see a complete Tracking from every include
 *               order. Kept out-of-line to preserve all include orders.
 */

#include "FrameDrawer.h"

namespace vs_graphs
{
namespace core
{

FrameDrawer::FrameDrawer(Atlas *pAtlas) :
    both(false),
    p_atlas(pAtlas)
{
    state      = Tracking::SYSTEM_NOT_READY;
    image      = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    imageRight = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
}

} // namespace core
} // namespace vs_graphs

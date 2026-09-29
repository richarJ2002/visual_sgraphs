/*!
 * @file         drawTextInfo.cc
 *
 * @brief        Implements FrameDrawer::drawTextInfo declared in FrameDrawer.h.
 */

#include "FrameDrawer.h"
#include "Tracking.h"

#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

FrameDrawerStatus FrameDrawer::drawTextInfo(cv::Mat &sourceImage_in,
                                            int      trackingState_in,
                                            cv::Mat &annotatedImage_out)
{
    stringstream textStream;
    if (trackingState_in == Tracking::NO_IMAGES_YET)
        textStream << " WAITING FOR IMAGES";
    else if (trackingState_in == Tracking::NOT_INITIALIZED)
        textStream << " TRYING TO INITIALIZE ";
    else if (trackingState_in == Tracking::OK)
    {
        if (!isTrackingOnlyMode)
            textStream << "SLAM MODE |  ";
        else
            textStream << "LOCALIZATION | ";
        int mapCount{};
        if (p_atlas->countMaps(mapCount) != AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: countMaps returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        unsigned long keyFrameCountValue{};
        if (p_atlas->getKeyFrameCount(keyFrameCountValue) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getKeyFrameCount returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        int           keyFrameCount = static_cast<int>(keyFrameCountValue);
        unsigned long mapPointCountValue{};
        if (p_atlas->getMapPointCount(mapPointCountValue) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPointCount returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        int mapPointCount = static_cast<int>(mapPointCountValue);
        textStream << "Maps: " << mapCount << ", KFs: " << keyFrameCount
                   << ", MPs: " << mapPointCount
                   << ", Matches: " << trackedCount;
        if (trackedVOCount > 0)
            textStream << ", + VO matches: " << trackedVOCount;
    }
    else if (trackingState_in == Tracking::LOST)
    {
        textStream << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if (trackingState_in == Tracking::SYSTEM_NOT_READY)
    {
        textStream << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int      baseline = 0;
    cv::Size textSize = cv::getTextSize(textStream.str(),
                                        cv::FONT_HERSHEY_PLAIN,
                                        1,
                                        1,
                                        &baseline);

    annotatedImage_out = cv::Mat(sourceImage_in.rows + textSize.height + 10,
                                 sourceImage_in.cols,
                                 sourceImage_in.type());
    sourceImage_in.copyTo(annotatedImage_out.rowRange(0, sourceImage_in.rows)
                              .colRange(0, sourceImage_in.cols));
    annotatedImage_out.rowRange(sourceImage_in.rows, annotatedImage_out.rows) =
        cv::Mat::zeros(textSize.height + 10,
                       sourceImage_in.cols,
                       sourceImage_in.type());
    cv::putText(annotatedImage_out,
                textStream.str(),
                cv::Point(5, annotatedImage_out.rows - 5),
                cv::FONT_HERSHEY_PLAIN,
                1,
                cv::Scalar(255, 255, 255),
                1,
                8);

    return FrameDrawerStatus::FRAME_DRAWER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

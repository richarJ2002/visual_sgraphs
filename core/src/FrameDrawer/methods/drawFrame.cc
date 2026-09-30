/*!
 * @file         drawFrame.cc
 *
 * @brief        Implements FrameDrawer::drawFrame declared in FrameDrawer.h.
 */

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

FrameDrawerStatus FrameDrawer::drawFrame(cv::Mat &frameImage_out,
                                         float    imageScale_in)
{
    cv::Mat displayImage;
    std::vector<cv::KeyPoint>
        initialKeyPoints; // Initialization: KeyPoints in reference frame
    std::vector<int> initialMatchIndices; // Initialization: correspondeces with
                                          // reference keypoints
    std::vector<cv::KeyPoint> currentKeyPoints; // KeyPoints in current frame
    std::vector<bool>         visualOdometryFlags,
        mapPointFlags; // Tracked MapPoints in current frame
    std::vector<std::pair<cv::Point2f, cv::Point2f>> initialTracks;
    int                drawState; // Tracking state
    std::vector<float> currentDepthValues;
    float              depthCutoff;

    Frame                                    drawnFrame;
    std::vector<MapPoint *>                  localMapPoints;
    std::vector<cv::KeyPoint>                matchedKeyPoints;
    std::vector<MapPoint *>                  matchedMapPoints;
    std::vector<cv::KeyPoint>                outlierKeyPoints;
    std::vector<MapPoint *>                  outlierMapPoints;
    std::map<long unsigned int, cv::Point2f> projectedPointMap;
    std::map<long unsigned int, cv::Point2f> matchedInImageMap;

    cv::Scalar standardColor(0, 255, 0);
    cv::Scalar odometryColor(255, 0, 0);

    // Copy variables within scoped mutex
    {
        std::unique_lock<std::mutex> stateLock(frameStateMutex);
        drawState = state;
        if (state == Tracking::SYSTEM_NOT_READY)
            state = Tracking::NO_IMAGES_YET;

        image.copyTo(displayImage);

        if (state == Tracking::NOT_INITIALIZED)
        {
            currentKeyPoints    = currentKeys;
            initialKeyPoints    = iniKeys;
            initialMatchIndices = iniMatches;
            initialTracks       = tracks;
        }
        else if (state == Tracking::OK)
        {
            currentKeyPoints    = currentKeys;
            visualOdometryFlags = isVisualOdometryPoint;
            mapPointFlags       = isTrackedMapPoint;

            drawnFrame        = currentFrame;
            localMapPoints    = localMap;
            matchedKeyPoints  = matchedKeys;
            matchedMapPoints  = matchedMPs;
            outlierKeyPoints  = outlierKeys;
            outlierMapPoints  = outlierMPs;
            projectedPointMap = projectPoints;
            matchedInImageMap = matchedInImage;

            currentDepthValues = currentDepths;
            depthCutoff        = depthThreshold;
        }
        else if (state == Tracking::LOST)
        {
            currentKeyPoints = currentKeys;
        }
    }

    if (imageScale_in != 1.f)
    {
        int imageWidth  = displayImage.cols / imageScale_in;
        int imageHeight = displayImage.rows / imageScale_in;
        cv::resize(displayImage,
                   displayImage,
                   cv::Size(imageWidth, imageHeight));
    }

    if (displayImage.channels() < 3) // this should be always true
        cv::cvtColor(displayImage, displayImage, cv::COLOR_GRAY2BGR);

    // Draw
    if (drawState == Tracking::NOT_INITIALIZED)
    {
        for (unsigned int keyPointIndex = 0;
             keyPointIndex < initialMatchIndices.size();
             keyPointIndex++)
        {
            if (initialMatchIndices[keyPointIndex] >= 0)
            {
                cv::Point2f drawPoint1, drawPoint2;
                if (imageScale_in != 1.f)
                {
                    drawPoint1 =
                        initialKeyPoints[keyPointIndex].pt / imageScale_in;
                    drawPoint2 =
                        currentKeyPoints[initialMatchIndices[keyPointIndex]]
                            .pt /
                        imageScale_in;
                }
                else
                {
                    drawPoint1 = initialKeyPoints[keyPointIndex].pt;
                    drawPoint2 =
                        currentKeyPoints[initialMatchIndices[keyPointIndex]].pt;
                }
                cv::line(displayImage, drawPoint1, drawPoint2, standardColor);
            }
        }
        for (std::vector<std::pair<cv::Point2f, cv::Point2f>>::iterator
                 trackIt = initialTracks.begin();
             trackIt != initialTracks.end();
             trackIt++)
        {
            cv::Point2f drawPoint1, drawPoint2;
            if (imageScale_in != 1.f)
            {
                drawPoint1 = (*trackIt).first / imageScale_in;
                drawPoint2 = (*trackIt).second / imageScale_in;
            }
            else
            {
                drawPoint1 = (*trackIt).first;
                drawPoint2 = (*trackIt).second;
            }
            cv::line(displayImage, drawPoint1, drawPoint2, standardColor, 5);
        }
    }
    else if (drawState == Tracking::OK) // TRACKING
    {
        trackedCount                     = 0;
        trackedVOCount                   = 0;
        const float markerHalfSize       = 5;
        int         currentKeyPointCount = currentKeyPoints.size();
        for (int keyPointIndex = 0; keyPointIndex < currentKeyPointCount;
             keyPointIndex++)
        {
            if (visualOdometryFlags[keyPointIndex] ||
                mapPointFlags[keyPointIndex])
            {
                cv::Point2f drawPoint1, drawPoint2;
                cv::Point2f keyPointPixel;
                if (imageScale_in != 1.f)
                {
                    keyPointPixel =
                        currentKeyPoints[keyPointIndex].pt / imageScale_in;
                    float pixelX =
                        currentKeyPoints[keyPointIndex].pt.x / imageScale_in;
                    float pixelY =
                        currentKeyPoints[keyPointIndex].pt.y / imageScale_in;
                    drawPoint1.x = pixelX - markerHalfSize;
                    drawPoint1.y = pixelY - markerHalfSize;
                    drawPoint2.x = pixelX + markerHalfSize;
                    drawPoint2.y = pixelY + markerHalfSize;
                }
                else
                {
                    keyPointPixel = currentKeyPoints[keyPointIndex].pt;
                    drawPoint1.x =
                        currentKeyPoints[keyPointIndex].pt.x - markerHalfSize;
                    drawPoint1.y =
                        currentKeyPoints[keyPointIndex].pt.y - markerHalfSize;
                    drawPoint2.x =
                        currentKeyPoints[keyPointIndex].pt.x + markerHalfSize;
                    drawPoint2.y =
                        currentKeyPoints[keyPointIndex].pt.y + markerHalfSize;
                }

                // This is a match to a MapPoint in the map
                if (mapPointFlags[keyPointIndex])
                {
                    cv::rectangle(displayImage,
                                  drawPoint1,
                                  drawPoint2,
                                  standardColor);
                    cv::circle(displayImage,
                               keyPointPixel,
                               2,
                               standardColor,
                               -1);
                    trackedCount++;
                }
                else // This is match to a "visual odometry" MapPoint created in
                     // the last frame
                {
                    cv::rectangle(displayImage,
                                  drawPoint1,
                                  drawPoint2,
                                  odometryColor);
                    cv::circle(displayImage,
                               keyPointPixel,
                               2,
                               odometryColor,
                               -1);
                    trackedVOCount++;
                }
            }
        }
    }

    cv::Mat imageWithInformation;
    if (drawTextInfo(displayImage, drawState, imageWithInformation) !=
        FrameDrawerStatus::FRAME_DRAWER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: drawTextInfo returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    frameImage_out = imageWithInformation;
    return FrameDrawerStatus::FRAME_DRAWER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

/*!
 * @file         drawRightFrame.cc
 *
 * @brief        Implements FrameDrawer::drawRightFrame declared in
 *               FrameDrawer.h.
 */

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace vs_graphs
{
namespace core
{

cv::Mat FrameDrawer::drawRightFrame(float imageScale_in)
{
    cv::Mat displayImage;
    vector<cv::KeyPoint>
        initialKeyPoints; // Initialization: KeyPoints in reference frame
    vector<int> initialMatchIndices; // Initialization: correspondeces with
                                     // reference keypoints
    vector<cv::KeyPoint> currentKeyPoints; // KeyPoints in current frame
    vector<bool>         visualOdometryFlags,
        mapPointFlags; // Tracked MapPoints in current frame
    int drawState;     // Tracking state

    // Copy variables within scoped mutex
    {
        unique_lock<mutex> stateLock(frameStateMutex);
        drawState = state;
        if (state == Tracking::SYSTEM_NOT_READY)
            state = Tracking::NO_IMAGES_YET;

        imageRight.copyTo(displayImage);

        if (state == Tracking::NOT_INITIALIZED)
        {
            currentKeyPoints    = currentKeysRight;
            initialKeyPoints    = iniKeys;
            initialMatchIndices = iniMatches;
        }
        else if (state == Tracking::OK)
        {
            currentKeyPoints    = currentKeysRight;
            visualOdometryFlags = isVisualOdometryPoint;
            mapPointFlags       = isTrackedMapPoint;
        }
        else if (state == Tracking::LOST)
        {
            currentKeyPoints = currentKeysRight;
        }
    } // destroy scoped mutex -> release mutex

    if (imageScale_in != 1.f)
    {
        int imageWidth  = displayImage.cols / imageScale_in;
        int imageHeight = displayImage.rows / imageScale_in;
        cv::resize(displayImage,
                   displayImage,
                   cv::Size(imageWidth, imageHeight));
    }

    if (displayImage.channels() < 3) // this should be always true
        cvtColor(displayImage, displayImage, cv::COLOR_GRAY2BGR);

    // Draw
    if (drawState == Tracking::NOT_INITIALIZED) // INITIALIZING
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

                cv::line(displayImage,
                         drawPoint1,
                         drawPoint2,
                         cv::Scalar(0, 255, 0));
            }
        }
    }
    else if (drawState == Tracking::OK) // TRACKING
    {
        trackedCount                     = 0;
        trackedVOCount                   = 0;
        const float markerHalfSize       = 5;
        const int   currentKeyPointCount = currentKeysRight.size();
        const int   leftKeyPointCount    = currentKeys.size();

        for (int keyPointIndex = 0; keyPointIndex < currentKeyPointCount;
             keyPointIndex++)
        {
            if (visualOdometryFlags[keyPointIndex + leftKeyPointCount] ||
                mapPointFlags[keyPointIndex + leftKeyPointCount])
            {
                cv::Point2f drawPoint1, drawPoint2;
                cv::Point2f keyPointPixel;
                if (imageScale_in != 1.f)
                {
                    keyPointPixel =
                        currentKeysRight[keyPointIndex].pt / imageScale_in;
                    float pixelX =
                        currentKeysRight[keyPointIndex].pt.x / imageScale_in;
                    float pixelY =
                        currentKeysRight[keyPointIndex].pt.y / imageScale_in;
                    drawPoint1.x = pixelX - markerHalfSize;
                    drawPoint1.y = pixelY - markerHalfSize;
                    drawPoint2.x = pixelX + markerHalfSize;
                    drawPoint2.y = pixelY + markerHalfSize;
                }
                else
                {
                    keyPointPixel = currentKeysRight[keyPointIndex].pt;
                    drawPoint1.x =
                        currentKeysRight[keyPointIndex].pt.x - markerHalfSize;
                    drawPoint1.y =
                        currentKeysRight[keyPointIndex].pt.y - markerHalfSize;
                    drawPoint2.x =
                        currentKeysRight[keyPointIndex].pt.x + markerHalfSize;
                    drawPoint2.y =
                        currentKeysRight[keyPointIndex].pt.y + markerHalfSize;
                }

                // This is a match to a MapPoint in the map
                if (mapPointFlags[keyPointIndex + leftKeyPointCount])
                {
                    cv::rectangle(displayImage,
                                  drawPoint1,
                                  drawPoint2,
                                  cv::Scalar(0, 255, 0));
                    cv::circle(displayImage,
                               keyPointPixel,
                               2,
                               cv::Scalar(0, 255, 0),
                               -1);
                    trackedCount++;
                }
                else // This is match to a "visual odometry" MapPoint created in
                     // the last frame
                {
                    cv::rectangle(displayImage,
                                  drawPoint1,
                                  drawPoint2,
                                  cv::Scalar(255, 0, 0));
                    cv::circle(displayImage,
                               keyPointPixel,
                               2,
                               cv::Scalar(255, 0, 0),
                               -1);
                    trackedVOCount++;
                }
            }
        }
    }

    cv::Mat imageWithInformation;
    drawTextInfo(displayImage, drawState, imageWithInformation);

    return imageWithInformation;
}

} // namespace core
} // namespace vs_graphs

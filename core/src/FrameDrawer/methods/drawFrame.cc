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

namespace vs_graphs
{
namespace core
{

cv::Mat FrameDrawer::drawFrame(float imageScale_in)
{
    cv::Mat displayImage;
    vector<cv::KeyPoint>
        initialKeyPoints; // Initialization: KeyPoints in reference frame
    vector<int> initialMatchIndices; // Initialization: correspondeces with
                                     // reference keypoints
    vector<cv::KeyPoint> currentKeyPoints; // KeyPoints in current frame
    vector<bool>         visualOdometryFlags,
        mapPointFlags; // Tracked MapPoints in current frame
    vector<pair<cv::Point2f, cv::Point2f>> initialTracks;
    int                                    drawState; // Tracking state
    vector<float>                          currentDepthValues;
    float                                  depthCutoff;

    Frame                               drawnFrame;
    vector<MapPoint *>                  localMapPoints;
    vector<cv::KeyPoint>                matchedKeyPoints;
    vector<MapPoint *>                  matchedMapPoints;
    vector<cv::KeyPoint>                outlierKeyPoints;
    vector<MapPoint *>                  outlierMapPoints;
    map<long unsigned int, cv::Point2f> projectedPointMap;
    map<long unsigned int, cv::Point2f> matchedInImageMap;

    cv::Scalar standardColor(0, 255, 0);
    cv::Scalar odometryColor(255, 0, 0);

    // Copy variables within scoped mutex
    {
        unique_lock<mutex> stateLock(frameStateMutex);
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
        cvtColor(displayImage, displayImage, cv::COLOR_GRAY2BGR);

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
        for (vector<pair<cv::Point2f, cv::Point2f>>::iterator trackIt =
                 initialTracks.begin();
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
    drawTextInfo(displayImage, drawState, imageWithInformation);

    return imageWithInformation;
}

} // namespace core
} // namespace vs_graphs

/*!
 * @file         update.cc
 *
 * @brief        Implements FrameDrawer::update declared in FrameDrawer.h.
 */

#include "FrameDrawer.h"
#include "Tracking.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

FrameDrawerStatus FrameDrawer::update(Tracking *p_tracker_in)
{
    unique_lock<mutex> stateLock(frameStateMutex);
    p_tracker_in->imageGray.copyTo(image);
    currentKeys    = p_tracker_in->currentFrame.keyPoints;
    depthThreshold = p_tracker_in->currentFrame.depthThreshold;
    currentDepths  = p_tracker_in->currentFrame.depths;

    if (shouldDrawBothImages)
    {
        currentKeysRight = p_tracker_in->currentFrame.keyPointsRight;
        p_tracker_in->imageRight.copyTo(imageRight);
        keyPointCount = currentKeys.size() + currentKeysRight.size();
    }
    else
    {
        keyPointCount = currentKeys.size();
    }

    isVisualOdometryPoint = vector<bool>(keyPointCount, false);
    isTrackedMapPoint     = vector<bool>(keyPointCount, false);
    isTrackingOnlyMode    = p_tracker_in->isTrackingOnlyMode;

    // Variables for the new visualization
    currentFrame  = p_tracker_in->currentFrame;
    projectPoints = currentFrame.projectedPoints;
    matchedInImage.clear();

    std::vector<MapPoint *> trackerLocalMapPoints{};
    if (p_tracker_in->getLocalMapPoints(trackerLocalMapPoints) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getLocalMapPoints returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    localMap = trackerLocalMapPoints;
    matchedKeys.clear();
    matchedKeys.reserve(keyPointCount);
    matchedMPs.clear();
    matchedMPs.reserve(keyPointCount);
    outlierKeys.clear();
    outlierKeys.reserve(keyPointCount);
    outlierMPs.clear();
    outlierMPs.reserve(keyPointCount);

    if (p_tracker_in->lastProcessedState == Tracking::NOT_INITIALIZED)
    {
        iniKeys    = p_tracker_in->initialFrame.keyPoints;
        iniMatches = p_tracker_in->iniMatches;
    }
    else if (p_tracker_in->lastProcessedState == Tracking::OK)
    {
        for (int keyPointIndex = 0; keyPointIndex < keyPointCount;
             keyPointIndex++)
        {
            MapPoint *p_mapPoint =
                p_tracker_in->currentFrame.mapPoints[keyPointIndex];
            if (p_mapPoint)
            {
                if (!p_tracker_in->currentFrame.outlierFlags[keyPointIndex])
                {
                    int mapPointObservationCount{};
                    if (p_mapPoint->getObservationCount(
                            mapPointObservationCount) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getObservationCount returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (mapPointObservationCount > 0)
                        isTrackedMapPoint[keyPointIndex] = true;
                    else
                        isVisualOdometryPoint[keyPointIndex] = true;

                    matchedInImage[p_mapPoint->id] =
                        currentKeys[keyPointIndex].pt;
                }
                else
                {
                    outlierMPs.push_back(p_mapPoint);
                    outlierKeys.push_back(currentKeys[keyPointIndex]);
                }
            }
        }
    }
    state = static_cast<int>(p_tracker_in->lastProcessedState);

    return FrameDrawerStatus::FRAME_DRAWER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

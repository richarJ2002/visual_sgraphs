/*!
 * @file         update.cc
 *
 * @brief        Implements FrameDrawer::update declared in FrameDrawer.h.
 */

#include "FrameDrawer.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

void FrameDrawer::update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->imageGray.copyTo(image);
    currentKeys    = pTracker->currentFrame.keyPoints;
    depthThreshold = pTracker->currentFrame.depthThreshold;
    currentDepths  = pTracker->currentFrame.depths;

    if (both)
    {
        currentKeysRight = pTracker->currentFrame.keyPointsRight;
        pTracker->imageRight.copyTo(imageRight);
        N = currentKeys.size() + currentKeysRight.size();
    }
    else
    {
        N = currentKeys.size();
    }

    mvbVO        = vector<bool>(N, false);
    mvbMap       = vector<bool>(N, false);
    onlyTracking = pTracker->onlyTracking;

    // Variables for the new visualization
    currentFrame  = pTracker->currentFrame;
    projectPoints = currentFrame.projectedPoints;
    matchedInImage.clear();

    localMap = pTracker->getLocalMapPoints();
    matchedKeys.clear();
    matchedKeys.reserve(N);
    matchedMPs.clear();
    matchedMPs.reserve(N);
    outlierKeys.clear();
    outlierKeys.reserve(N);
    outlierMPs.clear();
    outlierMPs.reserve(N);

    if (pTracker->lastProcessedState == Tracking::NOT_INITIALIZED)
    {
        iniKeys    = pTracker->initialFrame.keyPoints;
        iniMatches = pTracker->iniMatches;
    }
    else if (pTracker->lastProcessedState == Tracking::OK)
    {
        for (int i = 0; i < N; i++)
        {
            MapPoint *pMP = pTracker->currentFrame.mapPoints[i];
            if (pMP)
            {
                if (!pTracker->currentFrame.outlierFlags[i])
                {
                    if (pMP->getObservationCount() > 0)
                        mvbMap[i] = true;
                    else
                        mvbVO[i] = true;

                    matchedInImage[pMP->mnId] = currentKeys[i].pt;
                }
                else
                {
                    outlierMPs.push_back(pMP);
                    outlierKeys.push_back(currentKeys[i]);
                }
            }
        }
    }
    state = static_cast<int>(pTracker->lastProcessedState);
}

} // namespace core
} // namespace vs_graphs

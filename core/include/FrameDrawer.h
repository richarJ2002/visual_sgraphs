/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Atlas.h"
#include "Frame.h"
#include "FrameDrawerStatus.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <mutex>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{
class MapPoint;
} // namespace core
} // namespace vs_graphs

namespace vs_graphs
{
namespace core
{

class Tracking;
class Viewer;

class FrameDrawer
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FrameDrawer(Atlas *p_atlas_in);

    // Update info from the last processed frame.
    [[nodiscard]] FrameDrawerStatus update(Tracking *p_tracker_in);

    // Draw last processed frame.
    [[nodiscard]] FrameDrawerStatus drawFrame(cv::Mat &frameImage_out,
                                              float    imageScale_in = 1.f);
    [[nodiscard]] FrameDrawerStatus drawRightFrame(cv::Mat &frameImage_out,
                                                   float imageScale_in = 1.f);

    bool shouldDrawBothImages;

  protected:
    [[nodiscard]] FrameDrawerStatus drawTextInfo(cv::Mat &sourceImage_in,
                                                 int      trackingState_in,
                                                 cv::Mat &annotatedImage_out);

    // Info of the frame to be drawn
    cv::Mat              image, imageRight;
    int                  keyPointCount;
    vector<cv::KeyPoint> currentKeys, currentKeysRight;
    vector<bool>         isTrackedMapPoint, isVisualOdometryPoint;
    bool                 isTrackingOnlyMode;
    int                  trackedCount, trackedVOCount;
    vector<cv::KeyPoint> iniKeys;
    vector<int>          iniMatches;
    int                  state;
    std::vector<float>   currentDepths;
    float                depthThreshold;

    Atlas *p_atlas;

    std::mutex                             frameStateMutex;
    vector<pair<cv::Point2f, cv::Point2f>> tracks;

    Frame                currentFrame;
    vector<MapPoint *>   localMap;
    vector<cv::KeyPoint> matchedKeys;
    vector<MapPoint *>   matchedMPs;
    vector<cv::KeyPoint> outlierKeys;
    vector<MapPoint *>   outlierMPs;

    map<long unsigned int, cv::Point2f> projectPoints;
    map<long unsigned int, cv::Point2f> matchedInImage;
};

} // namespace core
} // namespace vs_graphs

#endif // FRAMEDRAWER_H

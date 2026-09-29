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

#include "LoopClosing.h"

#include "ORBmatcher.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

LoopClosingStatus LoopClosing::searchAndFuse(
    const std::vector<KeyFrame *> &conectedKeyFrames_in,
    vector<MapPoint *>            &mapPoints_in)
{
    ORBmatcher matcher(0.8);

    int totalReplaces = 0;

    // cout << "FUSE-POSE: Initially there are " << vpMapPoints.size() << " MPs"
    // << endl; cout << "FUSE-POSE: Intially there are " << vConectedKFs.size()
    // << " KFs" << endl;
    for (auto mit  = conectedKeyFrames_in.begin(),
              mend = conectedKeyFrames_in.end();
         mit != mend;
         mit++)
    {
        int       replaceCount = 0;
        KeyFrame *p_keyFrame   = (*mit);
        Map      *p_map        = nullptr;
        if (p_keyFrame->getMap(p_map) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Sophus::SE3f Tcw{};
        if (p_keyFrame->getPose(Tcw) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Sophus::Sim3f Scw(Tcw.unit_quaternion(), Tcw.translation());
        Scw.setScale(1.f);
        /*std::cout << "These should be zeros: " <<
            Scw.rotationMatrix() - Tcw.rotationMatrix() << std::endl <<
            Scw.translation() - Tcw.translation() << std::endl <<
            Scw.scale() - 1.f << std::endl;*/
        vector<MapPoint *> replacePoints(mapPoints_in.size(),
                                         static_cast<MapPoint *>(nullptr));
        int                matcherFusedCount{};
        if (matcher.fuse(p_keyFrame,
                         Scw,
                         mapPoints_in,
                         4,
                         replacePoints,
                         matcherFusedCount) !=
            ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: fuse returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        // Get Map Mutex
        unique_lock<mutex> lock(p_map->mapUpdateMutex);
        const int          lpCount = mapPoints_in.size();
        for (int lpIndex = 0; lpIndex < lpCount; lpIndex++)
        {
            MapPoint *p_rep = replacePoints[lpIndex];
            if (p_rep)
            {
                replaceCount += 1;
                if (p_rep->replace(mapPoints_in[lpIndex]) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: replace returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
            }
        }
        /*cout << "FUSE-POSE: KF " << pKF->id << " ->" << num_replaces << "
        MPs fused" << endl; total_replaces += num_replaces;*/
    }
    // cout << "FUSE-POSE: " << total_replaces << " MPs had been fused" << endl;

    return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

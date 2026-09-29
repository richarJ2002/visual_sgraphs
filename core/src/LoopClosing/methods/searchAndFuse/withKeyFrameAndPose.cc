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
#include "Utils/Converter/objects/Converter.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void LoopClosing::searchAndFuse(const KeyFrameAndPose &correctedPosesMap_in,
                                vector<MapPoint *>    &mapPoints_in)
{
    ORBmatcher matcher(0.8);

    int totalReplaces = 0;

    // cout << "[FUSE]: Initially there are " << vpMapPoints.size() << " MPs" <<
    // endl; cout << "FUSE: Intially there are " << CorrectedPosesMap.size() <<
    // " KFs" << endl;
    for (KeyFrameAndPose::const_iterator mit  = correctedPosesMap_in.begin(),
                                         mend = correctedPosesMap_in.end();
         mit != mend;
         mit++)
    {
        int       replaceCount = 0;
        KeyFrame *p_keyFrame   = mit->first;
        Map      *p_map        = nullptr;
        if (p_keyFrame->getMap(p_map) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        g2o::Sim3     g2oScw = mit->second;
        Sophus::Sim3f Scw{};
        if (utils::converter::Converter::toSophus(g2oScw, Scw) !=
            utils::converter::ConverterStatus::CONVERTER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: toSophus returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        vector<MapPoint *> replacePoints(mapPoints_in.size(),
                                         static_cast<MapPoint *>(nullptr));
        int                fusedCount =
            matcher.fuse(p_keyFrame, Scw, mapPoints_in, 4, replacePoints);

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

        totalReplaces += replaceCount;
    }
    // cout << "[FUSE]: " << total_replaces << " MPs had been fused" << endl;
}

} // namespace core
} // namespace vs_graphs

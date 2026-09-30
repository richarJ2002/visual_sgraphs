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
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

LoopClosingStatus LoopClosing::findMatchesByProjection(
    KeyFrame                *p_currentKeyFrame_in,
    KeyFrame                *p_matchedKFw_in,
    g2o::Sim3               &g2oScw_in,
    std::set<MapPoint *>    &matchedMPinOrigins_in,
    std::vector<MapPoint *> &mapPoints_out,
    std::vector<MapPoint *> &matchedMapPoints_out,
    int                     &matches_out)
{
    int                     countCovisibleCount = 10;
    std::vector<KeyFrame *> covisibleKeyFrames{};
    if (p_matchedKFw_in->getBestCovisibilityKeyFrames(countCovisibleCount,
                                                      covisibleKeyFrames) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBestCovisibilityKeyFrames returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    int initialCovisibleCount = covisibleKeyFrames.size();
    covisibleKeyFrames.push_back(p_matchedKFw_in);
    std::set<KeyFrame *> checkKeyFrames(covisibleKeyFrames.begin(),
                                        covisibleKeyFrames.end());
    std::set<KeyFrame *> currentCovisbles{};
    if (p_currentKeyFrame_in->getConnectedKeyFrames(currentCovisbles) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getConnectedKeyFrames returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (initialCovisibleCount < countCovisibleCount)
    {
        for (int covisibleIndex = 0; covisibleIndex < initialCovisibleCount;
             ++covisibleIndex)
        {
            std::vector<KeyFrame *> keyFrames{};
            if (covisibleKeyFrames[covisibleIndex]
                    ->getBestCovisibilityKeyFrames(countCovisibleCount,
                                                   keyFrames) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getBestCovisibilityKeyFrames returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
            int         insertedCount = 0;
            std::size_t j             = 0;
            while (j < keyFrames.size() && insertedCount < countCovisibleCount)
            {
                if (checkKeyFrames.find(keyFrames[j]) == checkKeyFrames.end() &&
                    currentCovisbles.find(keyFrames[j]) ==
                        currentCovisbles.end())
                {
                    checkKeyFrames.insert(keyFrames[j]);
                    ++insertedCount;
                }
                ++j;
            }
            covisibleKeyFrames.insert(covisibleKeyFrames.end(),
                                      keyFrames.begin(),
                                      keyFrames.end());
        }
    }
    std::set<MapPoint *> mapPoints;
    mapPoints_out.clear();
    matchedMapPoints_out.clear();
    for (KeyFrame *p_keyFrame : covisibleKeyFrames)
    {
        std::vector<MapPoint *> keyFrameMapPointMatches{};
        if (p_keyFrame->getMapPointMatches(keyFrameMapPointMatches) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPointMatches returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (MapPoint *p_candidateMapPoint : keyFrameMapPointMatches)
        {
            bool candidateMapPointIsBad{};
            if (!(!p_candidateMapPoint) &&
                p_candidateMapPoint->isBad(candidateMapPointIsBad) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!p_candidateMapPoint || candidateMapPointIsBad)
                continue;

            if (mapPoints.find(p_candidateMapPoint) == mapPoints.end())
            {
                mapPoints.insert(p_candidateMapPoint);
                mapPoints_out.push_back(p_candidateMapPoint);
            }
        }
    }

    Sophus::Sim3f correctedPose{};
    if (utils::converter::Converter::toSophus(g2oScw_in, correctedPose) !=
        utils::converter::ConverterStatus::CONVERTER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: toSophus returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    ORBmatcher matcher(0.9, true);

    std::vector<MapPoint *> currentKeyFrameMapPointMatches{};
    if (p_currentKeyFrame_in->getMapPointMatches(
            currentKeyFrameMapPointMatches) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPointMatches returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    matchedMapPoints_out.resize(currentKeyFrameMapPointMatches.size(),
                                static_cast<MapPoint *>(nullptr));
    int matchCount{};
    if (matcher.searchByProjection(p_currentKeyFrame_in,
                                   correctedPose,
                                   mapPoints_out,
                                   matchedMapPoints_out,
                                   3,
                                   matchCount,
                                   1.5) !=
        ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: searchByProjection returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    matches_out = matchCount;
    return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

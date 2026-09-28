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

namespace vs_graphs
{
namespace core
{

int LoopClosing::findMatchesByProjection(
    KeyFrame           *p_currentKeyFrame_in,
    KeyFrame           *p_matchedKFw_in,
    g2o::Sim3          &g2oScw_in,
    set<MapPoint *>    &matchedMPinOrigins_in,
    vector<MapPoint *> &mapPoints_out,
    vector<MapPoint *> &matchedMapPoints_out)
{
    int                countCovisibleCount = 10;
    vector<KeyFrame *> covisibleKeyFrames =
        p_matchedKFw_in->getBestCovisibilityKeyFrames(countCovisibleCount);
    int initialCovisibleCount = covisibleKeyFrames.size();
    covisibleKeyFrames.push_back(p_matchedKFw_in);
    set<KeyFrame *> checkKeyFrames(covisibleKeyFrames.begin(),
                                   covisibleKeyFrames.end());
    set<KeyFrame *> currentCovisbles =
        p_currentKeyFrame_in->getConnectedKeyFrames();
    if (initialCovisibleCount < countCovisibleCount)
    {
        for (int covisibleIndex = 0; covisibleIndex < initialCovisibleCount;
             ++covisibleIndex)
        {
            vector<KeyFrame *> keyFrames =
                covisibleKeyFrames[covisibleIndex]
                    ->getBestCovisibilityKeyFrames(countCovisibleCount);
            int insertedCount = 0;
            int j             = 0;
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
    set<MapPoint *> mapPoints;
    mapPoints_out.clear();
    matchedMapPoints_out.clear();
    for (KeyFrame *p_keyFrame : covisibleKeyFrames)
    {
        for (MapPoint *p_candidateMapPoint : p_keyFrame->getMapPointMatches())
        {
            if (!p_candidateMapPoint || p_candidateMapPoint->isBad())
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
        // toSophus cannot fail; continue as before.
    }
    ORBmatcher matcher(0.9, true);

    matchedMapPoints_out.resize(
        p_currentKeyFrame_in->getMapPointMatches().size(),
        static_cast<MapPoint *>(nullptr));
    int matchCount = matcher.searchByProjection(p_currentKeyFrame_in,
                                                correctedPose,
                                                mapPoints_out,
                                                matchedMapPoints_out,
                                                3,
                                                1.5);

    return matchCount;
}

} // namespace core
} // namespace vs_graphs

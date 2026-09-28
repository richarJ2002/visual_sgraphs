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

namespace vs_graphs
{
namespace core
{

bool LoopClosing::detectCommonRegionsFromLastKF(
    KeyFrame                *p_currentKeyFrame_in,
    KeyFrame                *p_matchedKeyFrame_in,
    g2o::Sim3               &gScw_inout,
    int                     &countProjectionMatchCount_out,
    std::vector<MapPoint *> &mapPoints_inout,
    std::vector<MapPoint *> &matchedMapPoints_inout)
{
    set<MapPoint *> alreadyMatchedMapPoints(matchedMapPoints_inout.begin(),
                                            matchedMapPoints_inout.end());
    countProjectionMatchCount_out =
        findMatchesByProjection(p_currentKeyFrame_in,
                                p_matchedKeyFrame_in,
                                gScw_inout,
                                alreadyMatchedMapPoints,
                                mapPoints_inout,
                                matchedMapPoints_inout);

    int projectionMatchCount = 30;
    if (countProjectionMatchCount_out >= projectionMatchCount)
    {
        return true;
    }

    return false;
}

} // namespace core
} // namespace vs_graphs

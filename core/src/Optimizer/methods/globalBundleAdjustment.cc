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

#include "Optimizer.h"

namespace vs_graphs
{
namespace core
{

void Optimizer::globalBundleAdjustment(
    Map                    *pMap,
    int                     nIterations,
    bool                   *pbStopFlag,
    const unsigned long     nLoopKF,
    const bool              bRobust,
    double                  markerImpact,
    const std::atomic_bool *pStopRequested_in)
{
    std::vector<vs_graphs::core::semantic::Room *> allRooms =
        pMap->getAllRooms();
    std::vector<vs_graphs::core::semantic::Floor *> allFloors =
        pMap->getAllFloors();
    std::vector<vs_graphs::core::geometric::Plane *> allPlanes =
        pMap->getAllPlanes();
    std::vector<vs_graphs::core::semantic::Marker *> allMarkers =
        pMap->getAllMarkers();
    std::vector<vs_graphs::core::semantic::Passage *> allPassages =
        pMap->getAllPassages();
    std::vector<vs_graphs::core::MapPoint *> allMapPoints =
        pMap->getAllMapPoints();
    std::vector<vs_graphs::core::KeyFrame *> allKeyFrames =
        pMap->getAllKeyFrames();

    bundleAdjustment(allKeyFrames,
                     allMapPoints,
                     allMarkers,
                     allPlanes,
                     allPassages,
                     allRooms,
                     allFloors,
                     nIterations,
                     pbStopFlag,
                     nLoopKF,
                     bRobust,
                     markerImpact,
                     pStopRequested_in);
}

} // namespace core
} // namespace vs_graphs

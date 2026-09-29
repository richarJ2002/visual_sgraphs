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

#include "MapPoint.h"

#include "Map.h"
#include "ORBmatcher.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

/* NOTE: out-of-line to break the KeyFrame<->MapPoint include cycle:
 * MapPoint.h cannot see complete KeyFrame/Map from every include order. */

MapPoint::MapPoint(const Eigen::Vector3f &Pos_in,
                   KeyFrame              *p_referenceKeyFrame_in,
                   Map                   *p_map_in) :
    firstKeyFrameId(p_referenceKeyFrame_in->id),
    firstFrameId(p_referenceKeyFrame_in->frameId),
    observationCount(0),
    trackReferenceFrameId(0),
    lastSeenFrameId(0),
    baLocalKeyFrameId(0),
    fuseCandidateKeyFrameId(0),
    loopPointKeyFrameId(0),
    correctedByKeyFrameId(0),
    correctedReferenceKeyFrameId(0),
    baGlobalKeyFrameId(0),
    originMapId(p_map_in->getId()),
    p_referenceKeyFrame(p_referenceKeyFrame_in),
    visibleCount(1),
    foundCount(1),
    isFlaggedBad(false),
    p_replaced(static_cast<MapPoint *>(nullptr)),
    minDistance(0),
    maxDistance(0),
    p_map(p_map_in)
{
    setWorldPos(Pos_in);

    normalVector.setZero();

    isTrackedInRightView = false;
    isTrackedInView      = false;

    // MapPoints can be created from Tracking and Local Mapping. This mutex
    // avoid conflicts with id.
    unique_lock<mutex> lock(p_map->pointCreationMutex);
    id = nextId++;
}

} // namespace core
} // namespace vs_graphs

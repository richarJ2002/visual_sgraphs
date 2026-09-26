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

#include "ORBmatcher.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

/* NOTE: out-of-line to break the KeyFrame<->MapPoint include cycle:
 * MapPoint.h cannot see complete KeyFrame/Map from every include order. */

MapPoint::MapPoint(const Eigen::Vector3f &Pos,
                   Map                   *pMap,
                   Frame                 *pFrame,
                   const int             &idxF) :
    firstKeyFrameId(-1),
    firstFrameId(pFrame->mnId),
    observationCount(0),
    trackReferenceFrameId(0),
    lastSeenFrameId(0),
    baLocalKeyFrameId(0),
    fuseCandidateKeyFrameId(0),
    loopPointKeyFrameId(0),
    correctedByKeyFrameId(0),
    correctedReferenceKeyFrameId(0),
    baGlobalKeyFrameId(0),
    originMapId(pMap->getId()),
    p_referenceKeyFrame(static_cast<KeyFrame *>(nullptr)),
    visibleCount(1),
    foundCount(1),
    mbBad(false),
    p_replaced(nullptr),
    p_map(pMap)
{
    setWorldPos(Pos);

    Eigen::Vector3f Ow;
    if (pFrame->Nleft == -1 || idxF < pFrame->Nleft)
    {
        Ow = pFrame->getCameraCenter();
    }
    else
    {
        Eigen::Matrix3f Rwl = pFrame->getRotationRwc();
        Eigen::Vector3f tlr = pFrame->getRelativePoseTlr().translation();
        Eigen::Vector3f twl = pFrame->getCenterOw();

        Ow = Rwl * tlr + twl;
    }
    normalVector = worldPos - Ow;
    normalVector = normalVector / normalVector.norm();

    Eigen::Vector3f PC   = worldPos - Ow;
    const float     dist = PC.norm();
    const int       level =
        (pFrame->Nleft == -1) ? pFrame->keyPointsUndistorted[idxF].octave
              : (idxF < pFrame->Nleft) ? pFrame->keyPoints[idxF].octave
                                       : pFrame->keyPointsRight[idxF].octave;
    const float levelScaleFactor = pFrame->scaleFactors[level];
    const int   nLevels          = pFrame->scaleLevelCount;

    maxDistance = dist * levelScaleFactor;
    minDistance = maxDistance / pFrame->scaleFactors[nLevels - 1];

    pFrame->descriptors.row(idxF).copyTo(descriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex
    // avoid conflicts with id.
    unique_lock<mutex> lock(p_map->mMutexPointCreation);
    mnId = nNextId++;
}

} // namespace core
} // namespace vs_graphs

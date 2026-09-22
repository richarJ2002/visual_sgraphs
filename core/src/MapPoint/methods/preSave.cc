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

void MapPoint::PreSave(set<KeyFrame *> &spKF, set<MapPoint *> &spMP)
{
    backupReplacedId = -1;

    backupObservationIds1.clear();
    backupObservationIds2.clear();

    // Snapshot the observation map and replaced pointer under the feature lock.
    // Dropped keyframes are erased below, after the lock is released, because
    // EraseObservation() takes mMutexFeatures again.
    std::map<KeyFrame *, std::tuple<int, int>> tmp_mObservations;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if (p_replaced && spMP.find(p_replaced) != spMP.end())
            backupReplacedId = p_replaced->mnId;

        tmp_mObservations.insert(observations.begin(), observations.end());
    }

    for (std::map<KeyFrame *, std::tuple<int, int>>::const_iterator
             it  = tmp_mObservations.begin(),
             end = tmp_mObservations.end();
         it != end;
         ++it)
    {
        KeyFrame *pKFi = it->first;
        if (spKF.find(pKFi) != spKF.end())
        {
            backupObservationIds1[it->first->mnId] = get<0>(it->second);
            backupObservationIds2[it->first->mnId] = get<1>(it->second);
        }
        else
        {
            eraseObservation(pKFi);
        }
    }

    // Save the id of the reference KF
    unique_lock<mutex> lock(mMutexFeatures);
    if (spKF.find(p_referenceKeyFrame) != spKF.end())
    {
        backupRefKeyFrameId = p_referenceKeyFrame->mnId;
    }
}

} // namespace core
} // namespace vs_graphs

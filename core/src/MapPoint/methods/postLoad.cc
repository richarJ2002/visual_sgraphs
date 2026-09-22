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

void MapPoint::PostLoad(map<long unsigned int, KeyFrame *> &mpKFid,
                        map<long unsigned int, MapPoint *> &mpMPid)
{
    p_referenceKeyFrame = mpKFid[backupRefKeyFrameId];
    if (!p_referenceKeyFrame)
    {
        cout << "ERROR: MP without KF reference " << backupRefKeyFrameId
             << "; Num obs: " << observationCount << endl;
    }
    p_replaced = static_cast<MapPoint *>(nullptr);
    if (backupReplacedId >= 0)
    {
        map<long unsigned int, MapPoint *>::iterator it =
            mpMPid.find(backupReplacedId);
        if (it != mpMPid.end())
            p_replaced = it->second;
    }

    observations.clear();

    for (map<long unsigned int, int>::const_iterator
             it  = backupObservationIds1.begin(),
             end = backupObservationIds1.end();
         it != end;
         ++it)
    {
        KeyFrame                                   *pKFi = mpKFid[it->first];
        map<long unsigned int, int>::const_iterator it2 =
            backupObservationIds2.find(it->first);
        std::tuple<int, int> indexes = tuple<int, int>(it->second, it2->second);
        if (pKFi)
        {
            observations[pKFi] = indexes;
        }
    }

    backupObservationIds1.clear();
    backupObservationIds2.clear();
}

} // namespace core
} // namespace vs_graphs

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

namespace vs_graphs
{
namespace core
{

void LoopClosing::searchAndFuse(const vector<KeyFrame *> &vConectedKFs,
                                vector<MapPoint *>       &vpMapPoints)
{
    ORBmatcher matcher(0.8);

    int total_replaces = 0;

    // cout << "FUSE-POSE: Initially there are " << vpMapPoints.size() << " MPs"
    // << endl; cout << "FUSE-POSE: Intially there are " << vConectedKFs.size()
    // << " KFs" << endl;
    for (auto mit = vConectedKFs.begin(), mend = vConectedKFs.end();
         mit != mend;
         mit++)
    {
        int           num_replaces = 0;
        KeyFrame     *pKF          = (*mit);
        Map          *pMap         = pKF->getMap();
        Sophus::SE3f  Tcw          = pKF->getPose();
        Sophus::Sim3f Scw(Tcw.unit_quaternion(), Tcw.translation());
        Scw.setScale(1.f);
        /*std::cout << "These should be zeros: " <<
            Scw.rotationMatrix() - Tcw.rotationMatrix() << std::endl <<
            Scw.translation() - Tcw.translation() << std::endl <<
            Scw.scale() - 1.f << std::endl;*/
        vector<MapPoint *> vpReplacePoints(vpMapPoints.size(),
                                           static_cast<MapPoint *>(nullptr));
        matcher.fuse(pKF, Scw, vpMapPoints, 4, vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        const int          nLP = vpMapPoints.size();
        for (int i = 0; i < nLP; i++)
        {
            MapPoint *pRep = vpReplacePoints[i];
            if (pRep)
            {
                num_replaces += 1;
                pRep->replace(vpMapPoints[i]);
            }
        }
        /*cout << "FUSE-POSE: KF " << pKF->mnId << " ->" << num_replaces << "
        MPs fused" << endl; total_replaces += num_replaces;*/
    }
    // cout << "FUSE-POSE: " << total_replaces << " MPs had been fused" << endl;
}

} // namespace core
} // namespace vs_graphs

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

namespace vs_graphs
{
namespace core
{

void LoopClosing::searchAndFuse(const KeyFrameAndPose &CorrectedPosesMap,
                                vector<MapPoint *>    &vpMapPoints)
{
    ORBmatcher matcher(0.8);

    int total_replaces = 0;

    // cout << "[FUSE]: Initially there are " << vpMapPoints.size() << " MPs" <<
    // endl; cout << "FUSE: Intially there are " << CorrectedPosesMap.size() <<
    // " KFs" << endl;
    for (KeyFrameAndPose::const_iterator mit  = CorrectedPosesMap.begin(),
                                         mend = CorrectedPosesMap.end();
         mit != mend;
         mit++)
    {
        int       num_replaces = 0;
        KeyFrame *pKFi         = mit->first;
        Map      *pMap         = pKFi->getMap();

        g2o::Sim3     g2oScw = mit->second;
        Sophus::Sim3f Scw    = utils::converter::Converter::toSophus(g2oScw);

        vector<MapPoint *> vpReplacePoints(vpMapPoints.size(),
                                           static_cast<MapPoint *>(nullptr));
        int numFused = matcher.fuse(pKFi, Scw, vpMapPoints, 4, vpReplacePoints);

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

        total_replaces += num_replaces;
    }
    // cout << "[FUSE]: " << total_replaces << " MPs had been fused" << endl;
}

} // namespace core
} // namespace vs_graphs

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

#include "KeyFrame.h"

#include "ImuTypes.h"
#include "Utils/Converter/objects/Converter.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

void KeyFrame::updateConnections(bool upParent)
{
    map<KeyFrame *, int> KFcounter;

    vector<MapPoint *> vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mapPoints;
    }

    // for all plane observations in the keyframe check in which other keyframes
    // are they seen increase counter for those keyframes
    if (types::SystemParams::getParams()->planeBasedCovisibility.enabled)
    {
        unsigned int scorePerPlane = types::SystemParams::getParams()
                                         ->planeBasedCovisibility.scorePerPlane;
        for (vector<geometric::Plane *>::iterator vit  = mapPlanes.begin(),
                                                  vend = mapPlanes.end();
             vit != vend;
             vit++)
        {
            geometric::Plane *pPlane = *vit;

            if (!pPlane)
                continue;

            map<KeyFrame *, vs_graphs::core::geometric::Plane::Observation>
                observations = pPlane->getObservations();

            for (map<KeyFrame *,
                     vs_graphs::core::geometric::Plane::Observation>::iterator
                     mit  = observations.begin(),
                     mend = observations.end();
                 mit != mend;
                 mit++)
            {
                if (mit->first->mnId == mnId || mit->first->isBad() ||
                    mit->first->getMap() != p_map)
                    continue;

                if (pPlane->getPlaneType() ==
                    geometric::Plane::PlaneVariant::UNDEFINED)
                    KFcounter[mit->first] += static_cast<int>(
                        scorePerPlane *
                        0.2); // undefined planes have less weight
                else
                    KFcounter[mit->first] += scorePerPlane;
            }
        }
    }

    // For all map points in keyframe check in which other keyframes are they
    // seen Increase counter for those keyframes
    for (vector<MapPoint *>::iterator vit = vpMP.begin(), vend = vpMP.end();
         vit != vend;
         vit++)
    {
        MapPoint *pMP = *vit;

        if (!pMP)
            continue;

        if (pMP->isBad())
            continue;

        map<KeyFrame *, tuple<int, int>> observations = pMP->getObservations();

        for (map<KeyFrame *, tuple<int, int>>::iterator
                 mit  = observations.begin(),
                 mend = observations.end();
             mit != mend;
             mit++)
        {
            if (mit->first->mnId == mnId || mit->first->isBad() ||
                mit->first->getMap() != p_map)
                continue;
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    if (KFcounter.empty())
        return;

    // If the counter is greater than threshold add connection
    // In case no keyframe counter is over threshold add the one with maximum
    // counter
    int       nmax   = 0;
    KeyFrame *pKFmax = nullptr;
    int       th     = 15;

    vector<pair<int, KeyFrame *>> vPairs;
    vPairs.reserve(KFcounter.size());
    if (!upParent)
        cout << "UPDATE_CONN: current KF " << mnId << endl;
    for (map<KeyFrame *, int>::iterator mit  = KFcounter.begin(),
                                        mend = KFcounter.end();
         mit != mend;
         mit++)
    {
        if (!upParent)
            cout << "  UPDATE_CONN: KF " << mit->first->mnId
                 << " ; num matches: " << mit->second << endl;
        if (mit->second > nmax)
        {
            nmax   = mit->second;
            pKFmax = mit->first;
        }
        if (mit->second >= th)
        {
            vPairs.push_back(make_pair(mit->second, mit->first));
            (mit->first)->addConnection(this, mit->second);
        }
    }

    if (vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax, pKFmax));
        pKFmax->addConnection(this, nmax);
    }

    sort(vPairs.begin(), vPairs.end());
    list<KeyFrame *> lKFs;
    list<int>        lWs;
    for (size_t i = 0; i < vPairs.size(); i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        connectedKeyFrameWeights = KFcounter;
        orderedConnectedKeyFrames =
            vector<KeyFrame *>(lKFs.begin(), lKFs.end());
        orderedWeights = vector<int>(lWs.begin(), lWs.end());

        if (firstConnection && mnId != p_map->getInitKeyFrameId())
        {
            p_parent = orderedConnectedKeyFrames.front();
            p_parent->addChild(this);
            firstConnection = false;
        }
    }
}

} // namespace core
} // namespace vs_graphs

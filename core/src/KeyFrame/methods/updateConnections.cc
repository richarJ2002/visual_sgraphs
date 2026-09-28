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

void KeyFrame::updateConnections(bool upParent_in)
{
    map<KeyFrame *, int> keyFrameCounter;

    vector<MapPoint *> keyFrameMapPoints;

    {
        unique_lock<mutex> lockMapPoints(featuresMutex);
        keyFrameMapPoints = mapPoints;
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
            geometric::Plane *p_plane = *vit;

            if (!p_plane)
                continue;

            map<KeyFrame *, vs_graphs::core::geometric::Plane::Observation>
                observations = p_plane->getObservations();

            for (map<KeyFrame *,
                     vs_graphs::core::geometric::Plane::Observation>::iterator
                     mit  = observations.begin(),
                     mend = observations.end();
                 mit != mend;
                 mit++)
            {
                if (mit->first->id == id || mit->first->isBad() ||
                    mit->first->getMap() != p_map)
                    continue;

                if (p_plane->getPlaneType() ==
                    geometric::Plane::PlaneVariant::UNDEFINED)
                    keyFrameCounter[mit->first] += static_cast<int>(
                        scorePerPlane *
                        0.2); // undefined planes have less weight
                else
                    keyFrameCounter[mit->first] += scorePerPlane;
            }
        }
    }

    // For all map points in keyframe check in which other keyframes are they
    // seen Increase counter for those keyframes
    for (vector<MapPoint *>::iterator vit  = keyFrameMapPoints.begin(),
                                      vend = keyFrameMapPoints.end();
         vit != vend;
         vit++)
    {
        MapPoint *p_mapPoint = *vit;

        if (!p_mapPoint)
            continue;

        if (p_mapPoint->isBad())
            continue;

        map<KeyFrame *, tuple<int, int>> observations =
            p_mapPoint->getObservations();

        for (map<KeyFrame *, tuple<int, int>>::iterator
                 mit  = observations.begin(),
                 mend = observations.end();
             mit != mend;
             mit++)
        {
            if (mit->first->id == id || mit->first->isBad() ||
                mit->first->getMap() != p_map)
                continue;
            keyFrameCounter[mit->first]++;
        }
    }

    // This should not happen
    if (keyFrameCounter.empty())
        return;

    // If the counter is greater than threshold add connection
    // In case no keyframe counter is over threshold add the one with maximum
    // counter
    int       nmax              = 0;
    KeyFrame *p_keyFrameMaximum = nullptr;
    int       threshold         = 15;

    vector<pair<int, KeyFrame *>> pairs;
    pairs.reserve(keyFrameCounter.size());
    if (!upParent_in)
        cout << "UPDATE_CONN: current KF " << id << endl;
    for (map<KeyFrame *, int>::iterator mit  = keyFrameCounter.begin(),
                                        mend = keyFrameCounter.end();
         mit != mend;
         mit++)
    {
        if (!upParent_in)
            cout << "  UPDATE_CONN: KF " << mit->first->id
                 << " ; num matches: " << mit->second << endl;
        if (mit->second > nmax)
        {
            nmax              = mit->second;
            p_keyFrameMaximum = mit->first;
        }
        if (mit->second >= threshold)
        {
            pairs.push_back(make_pair(mit->second, mit->first));
            (mit->first)->addConnection(this, mit->second);
        }
    }

    if (pairs.empty())
    {
        pairs.push_back(make_pair(nmax, p_keyFrameMaximum));
        p_keyFrameMaximum->addConnection(this, nmax);
    }

    sort(pairs.begin(), pairs.end());
    list<KeyFrame *> keyFrames;
    list<int>        weights;
    for (size_t pairIndex = 0; pairIndex < pairs.size(); pairIndex++)
    {
        keyFrames.push_front(pairs[pairIndex].second);
        weights.push_front(pairs[pairIndex].first);
    }

    {
        unique_lock<mutex> lockCon(connectionsMutex);

        connectedKeyFrameWeights = keyFrameCounter;
        orderedConnectedKeyFrames =
            vector<KeyFrame *>(keyFrames.begin(), keyFrames.end());
        orderedWeights = vector<int>(weights.begin(), weights.end());

        if (isFirstConnection && id != p_map->getInitKeyFrameId())
        {
            p_parent = orderedConnectedKeyFrames.front();
            p_parent->addChild(this);
            isFirstConnection = false;
        }
    }
}

} // namespace core
} // namespace vs_graphs

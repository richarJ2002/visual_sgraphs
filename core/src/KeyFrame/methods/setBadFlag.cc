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

void KeyFrame::setBadFlag()
{
    {
        unique_lock<mutex> lock(connectionsMutex);
        if (id == p_map->getInitKeyFrameId())
        {
            return;
        }
        else if (isEraseProtected)
        {
            isPendingErase = true;
            return;
        }
    }

    for (map<KeyFrame *, int>::iterator mit  = connectedKeyFrameWeights.begin(),
                                        mend = connectedKeyFrameWeights.end();
         mit != mend;
         mit++)
    {
        mit->first->eraseConnection(this);
    }

    /*
     * Semantic observations store non-owning KeyFrame pointers. Detach this
     * keyframe before LocalMapping is allowed to delete it; otherwise a later
     * merge, GBA, or observation insertion can dereference freed memory.
     */
    const std::vector<geometric::Plane *> observedPlanes = getMapPlanes();
    for (geometric::Plane *p_plane : observedPlanes)
    {
        if (p_plane != nullptr)
        {
            p_plane->eraseObservation(this);
        }
    }

    const std::vector<semantic::Marker *> observedMarkers = getMapMarkers();
    for (semantic::Marker *p_marker : observedMarkers)
    {
        if (p_marker != nullptr)
        {
            p_marker->eraseObservation(this);
        }
    }

    for (size_t mapPointIndex = 0; mapPointIndex < mapPoints.size();
         mapPointIndex++)
    {
        if (mapPoints[mapPointIndex])
        {
            mapPoints[mapPointIndex]->eraseObservation(this);
        }
    }

    {
        unique_lock<mutex> lock(connectionsMutex);
        unique_lock<mutex> lock1(featuresMutex);

        connectedKeyFrameWeights.clear();
        orderedConnectedKeyFrames.clear();
        mapPlanes.clear();
        mapMarkers.clear();

        // Update Spanning Tree
        set<KeyFrame *> parentCandidates;
        if (p_parent)
            parentCandidates.insert(p_parent);

        // Assign at each iteration one children with a parent (the pair with
        // highest covisibility weight) Include that children as new parent
        // candidate for the rest
        while (!childrens.empty())
        {
            bool shouldContinue = false;

            int       maximum = -1;
            KeyFrame *pC;
            KeyFrame *pP;

            for (set<KeyFrame *>::iterator sit  = childrens.begin(),
                                           send = childrens.end();
                 sit != send;
                 sit++)
            {
                KeyFrame *p_keyFrame = *sit;
                if (p_keyFrame->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<KeyFrame *> connecteds =
                    p_keyFrame->getVectorCovisibleKeyFrames();
                for (size_t mapPointIndex = 0, iend = connecteds.size();
                     mapPointIndex < iend;
                     mapPointIndex++)
                {
                    for (set<KeyFrame *>::iterator
                             spcit  = parentCandidates.begin(),
                             spcend = parentCandidates.end();
                         spcit != spcend;
                         spcit++)
                    {
                        if (connecteds[mapPointIndex]->id == (*spcit)->id)
                        {
                            int w = p_keyFrame->getWeight(
                                connecteds[mapPointIndex]);
                            if (w > maximum)
                            {
                                pC             = p_keyFrame;
                                pP             = connecteds[mapPointIndex];
                                maximum        = w;
                                shouldContinue = true;
                            }
                        }
                    }
                }
            }

            if (shouldContinue)
            {
                pC->changeParent(pP);
                parentCandidates.insert(pC);
                childrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate,
        // assign to the original parent of this KF
        if (!childrens.empty())
        {
            for (set<KeyFrame *>::iterator sit = childrens.begin();
                 sit != childrens.end();
                 sit++)
            {
                (*sit)->changeParent(p_parent);
            }
        }

        if (p_parent)
        {
            p_parent->eraseChild(this);
            tcp = poseTcw * p_parent->getPoseInverse();
        }
        isFlaggedBad = true;
    }

    p_map->eraseKeyFrame(this);
    p_keyFrameDatabase->erase(this);
}

} // namespace core
} // namespace vs_graphs

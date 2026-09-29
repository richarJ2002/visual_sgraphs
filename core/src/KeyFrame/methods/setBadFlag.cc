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

#include "Geometric/Plane.h"
#include "Geometric/PlaneStatus.h"
#include "ImuTypes.h"
#include "KeyFrameDatabase.h"
#include "Map.h"
#include "MapPoint.h"
#include "Semantic/Marker.h"
#include "Semantic/MarkerStatus.h"
#include "Utils/Converter/objects/Converter.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

KeyFrameStatus KeyFrame::setBadFlag()
{
    {
        unique_lock<mutex> lock(connectionsMutex);
        unsigned long      mapInitKeyFrameId{};
        if (p_map->getInitKeyFrameId(mapInitKeyFrameId) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getInitKeyFrameId returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (id == mapInitKeyFrameId)
        {
            return KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS;
        }
        else if (isEraseProtected)
        {
            isPendingErase = true;
            return KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS;
        }
    }

    for (map<KeyFrame *, int>::iterator mit  = connectedKeyFrameWeights.begin(),
                                        mend = connectedKeyFrameWeights.end();
         mit != mend;
         mit++)
    {
        if (mit->first->eraseConnection(this) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: eraseConnection returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /*
     * Semantic observations store non-owning KeyFrame pointers. Detach this
     * keyframe before LocalMapping is allowed to delete it; otherwise a later
     * merge, GBA, or observation insertion can dereference freed memory.
     */
    std::vector<geometric::Plane *> observedPlanes{};
    if (getMapPlanes(observedPlanes) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (geometric::Plane *p_plane : observedPlanes)
    {
        if (p_plane != nullptr)
        {
            if (p_plane->eraseObservation(this) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseObservation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    std::vector<semantic::Marker *> observedMarkers{};
    if (getMapMarkers(observedMarkers) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapMarkers returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (semantic::Marker *p_marker : observedMarkers)
    {
        if (p_marker != nullptr)
        {
            if (p_marker->eraseObservation(this) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseObservation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    for (size_t mapPointIndex = 0; mapPointIndex < mapPoints.size();
         mapPointIndex++)
    {
        if (mapPoints[mapPointIndex])
        {
            if (mapPoints[mapPointIndex]->eraseObservation(this) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseObservation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
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
                bool      keyFrameIsBad{};
                if (p_keyFrame->isBad(keyFrameIsBad) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (keyFrameIsBad)
                    continue;

                // Check if a parent candidate is connected to the keyframe
                std::vector<KeyFrame *> connecteds{};
                if (p_keyFrame->getVectorCovisibleKeyFrames(connecteds) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getVectorCovisibleKeyFrames returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }
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
                            int w{};
                            if (p_keyFrame->getWeight(connecteds[mapPointIndex],
                                                      w) !=
                                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                             "%s: getWeight returned a failure "
                                             "status although it cannot fail; "
                                             "continuing as before.",
                                             __func__);
                            }
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
                if (pC->changeParent(pP) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: changeParent returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
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
                if ((*sit)->changeParent(p_parent) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: changeParent returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
            }
        }

        if (p_parent)
        {
            if (p_parent->eraseChild(this) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseChild returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Sophus::SE3f parentPoseInverse{};
            if (p_parent->getPoseInverse(parentPoseInverse) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPoseInverse returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            tcp = poseTcw * parentPoseInverse;
        }
        isFlaggedBad = true;
    }

    if (p_map->eraseKeyFrame(this) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: eraseKeyFrame returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    p_keyFrameDatabase->erase(this);

    return KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

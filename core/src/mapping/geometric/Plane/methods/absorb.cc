/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            absorb.cc
 *
 * @brief           Implements Plane::absorb(), declared in
 *                  mapping/geometric/Plane/objects/PlaneClass.h.
 */

#include "mapping/geometric/Plane/objects/PlaneClass.h"

#include <cstddef>
#include <iterator>
#include <map>
#include <mutex>
#include <set>
#include <utility>

#include <rclcpp/logging.hpp>

#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map_point/map_point.h"
#include "map_state/map_point/objects/MapPointStatusEnum.h"

namespace vs_graphs
{
namespace core
{
namespace geometric
{

PlaneStatus Plane::absorb(Plane *p_retiredPlane_inout)
{
    if (p_retiredPlane_inout == nullptr || p_retiredPlane_inout == this)
    {
        return PlaneStatus::PLANE_STATUS_SUCCESS;
    }

    /*!
     * Keyframe and map-point validity checks take those objects' own locks,
     * so the retired plane's records are copied and filtered before any
     * plane lock is held.
     */
    std::map<keyframe::KeyFrame *, Observation> retiredObservations{};
    if (p_retiredPlane_inout->getObservations(retiredObservations) !=
        PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getObservations returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    for (std::map<keyframe::KeyFrame *, Observation>::iterator observationIt =
             retiredObservations.begin();
         observationIt != retiredObservations.end();)
    {
        bool keyFrameIsBad{true};
        if (observationIt->first != nullptr &&
            observationIt->first->isBad(keyFrameIsBad) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        observationIt = keyFrameIsBad ? retiredObservations.erase(observationIt)
                                      : std::next(observationIt);
    }

    std::set<map_point::MapPoint *> retiredMapPoints{};
    if (p_retiredPlane_inout->getMapPoints(retiredMapPoints) !=
        PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPoints returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (std::set<map_point::MapPoint *>::iterator mapPointIt =
             retiredMapPoints.begin();
         mapPointIt != retiredMapPoints.end();)
    {
        bool mapPointIsBad{true};
        if (*mapPointIt != nullptr &&
            (*mapPointIt)->isBad(mapPointIsBad) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        mapPointIt = mapPointIsBad ? retiredMapPoints.erase(mapPointIt)
                                   : std::next(mapPointIt);
    }

    Plane *p_retiredTwinFace = nullptr;
    {
        /* std::scoped_lock orders the five mutexes without deadlock. */
        std::scoped_lock lock(positionMutex,
                              typeMutex,
                              featuresMutex,
                              p_retiredPlane_inout->positionMutex,
                              p_retiredPlane_inout->featuresMutex);

        /* One record per keyframe: a keyframe both planes saw is combined. */
        for (const std::pair<keyframe::KeyFrame *const, Observation>
                 &retiredRecord : retiredObservations)
        {
            const Observation &retiredObservation = retiredRecord.second;
            const std::map<keyframe::KeyFrame *, Observation>::iterator
                observationIt = observations.find(retiredRecord.first);
            if (observationIt == observations.end())
            {
                observations.emplace(retiredRecord.first, retiredObservation);
                observationCount++;
                continue;
            }

            /*!
             * The record carries the sum of both records' label evidence.
             * confidence is the optimiser edge weight only and is summed
             * separately; it never feeds the vote.
             */
            Observation &retainedObservation = observationIt->second;
            const double retainedConfidence  = retainedObservation.confidence;
            retainedObservation.pointPlaneConstraintMatrix +=
                retiredObservation.pointPlaneConstraintMatrix;
            for (const std::pair<const PlaneVariant, double> &evidence :
                 retiredObservation.semanticEvidence)
            {
                retainedObservation.semanticEvidence[evidence.first] +=
                    evidence.second;
            }
            retainedObservation.confidence += retiredObservation.confidence;
            if (retiredObservation.confidence > retainedConfidence)
            {
                retainedObservation.localPlane = retiredObservation.localPlane;
                retainedObservation.semanticType =
                    retiredObservation.semanticType;
            }
        }

        /* Keep the earliest valid reference keyframe of the two planes. */
        keyframe::KeyFrame *p_retiredRefKeyFrame =
            p_retiredPlane_inout->p_refKeyFrame;
        if (retiredObservations.count(p_retiredRefKeyFrame) != 0U &&
            (p_refKeyFrame == nullptr ||
             p_retiredRefKeyFrame->id < p_refKeyFrame->id))
        {
            p_refKeyFrame = p_retiredRefKeyFrame;
        }

        /* Both faces passed the same-side check, so one origin serves. */
        if (!observationOrigin_world_m.has_value())
        {
            observationOrigin_world_m =
                p_retiredPlane_inout->observationOrigin_world_m;
        }

        /* No plane may stay linked to the retired face. */
        p_retiredTwinFace                = p_retiredPlane_inout->p_twinFace;
        p_retiredPlane_inout->p_twinFace = nullptr;
        if (p_twinFace == p_retiredPlane_inout)
        {
            p_twinFace = nullptr;
        }
        if (p_retiredTwinFace == this)
        {
            p_retiredTwinFace = nullptr;
        }

        mapPoints.insert(retiredMapPoints.begin(), retiredMapPoints.end());

        if (p_retiredPlane_inout->planeCloud != nullptr)
        {
            planeCloud->points.insert(
                planeCloud->points.end(),
                p_retiredPlane_inout->planeCloud->points.begin(),
                p_retiredPlane_inout->planeCloud->points.end());
        }
        if (thinMapCloudWithoutLock() != PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: thinMapCloudWithoutLock returned a failure "
                         "status although it cannot fail; continuing as "
                         "before.",
                         __func__);
        }

        /* This generation is fitted here; a later refit has nothing new. */
        ++cloudGeneration;
        lastRefitAttemptGeneration = cloudGeneration;

        Eigen::Vector3d fittedCentroid_world_m = Eigen::Vector3d::Zero();
        Eigen::Vector4d fittedEquation_world   = Eigen::Vector4d::Zero();
        std::size_t     finitePointCount       = 0U;
        bool            wasFitFound            = false;
        if (fitEquationToCloud(*planeCloud,
                               globalEquation.coeffs(),
                               fittedCentroid_world_m,
                               fittedEquation_world,
                               finitePointCount,
                               wasFitFound) !=
            PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: fitEquationToCloud returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (wasFitFound)
        {
            centroid       = fittedCentroid_world_m;
            globalEquation = g2o::Plane3D(fittedEquation_world);
            lastSuccessfulRefitFinitePointCount = finitePointCount;
            successfulRefitGeneration           = cloudGeneration;
        }

        /* Bounds and labels follow the combined cloud and records. */
        if (updatePlaneBoundsWithoutLock() != PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updatePlaneBoundsWithoutLock returned a failure "
                         "status although it cannot fail; continuing as "
                         "before.",
                         __func__);
        }
        if (rebuildSemanticVotesWithoutLock() !=
            PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: rebuildSemanticVotesWithoutLock returned a "
                         "failure status although it cannot fail; continuing "
                         "as before.",
                         __func__);
        }
    }

    /* The retired face's twin is another plane; its link is cleared here. */
    if (p_retiredTwinFace != nullptr)
    {
        Plane *p_twinFaceLink = nullptr;
        if (p_retiredTwinFace->getTwinFace(p_twinFaceLink) !=
            PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getTwinFace returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_twinFaceLink == p_retiredPlane_inout &&
            p_retiredTwinFace->clearTwinFace() !=
                PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: clearTwinFace returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
    }

    return PlaneStatus::PLANE_STATUS_SUCCESS;
}

} /* namespace geometric */
} /* namespace core */
} /* namespace vs_graphs */

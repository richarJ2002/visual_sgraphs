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
 * @file            reconcileWallFacePairs.cc
 *
 * @brief           Implements SemanticsManager::reconcileWallFacePairs(),
 *                  declared in SemanticsManager.h.
 */

#include "SemanticsManager.h"

#include "../private_functions.h"
#include "Semantic/ValueOrder.h"

#include <algorithm>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::reconcileWallFacePairs(void)
{
    geometric::Plane *p_groundPlane = nullptr;
    if (p_atlas->getBiggestGroundPlane(p_groundPlane) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector3d groundNormal_world = Eigen::Vector3d::Zero();
    bool            groundPlaneIsBad{};
    if ((p_groundPlane != nullptr) &&
        p_groundPlane->isBad(groundPlaneIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_groundPlane != nullptr && !groundPlaneIsBad)
    {
        g2o::Plane3D groundPlaneGetGlobalEquation{};
        if (p_groundPlane->getGlobalEquation(groundPlaneGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector4d groundEq = groundPlaneGetGlobalEquation.coeffs();
        const double          groundNorm = groundEq.head<3>().norm();
        if (groundEq.allFinite() && groundNorm > 1e-8)
        {
            groundNormal_world = groundEq.head<3>() / groundNorm;
        }
    }

    const double minimumThickness_m =
        static_cast<double>(p_sysParams->semSeg.wallPairing.minimumThickness_m);
    const double maximumThickness_m =
        static_cast<double>(p_sysParams->semSeg.wallPairing.maximumThickness_m);
    const double minimumOverlapRatio = static_cast<double>(
        p_sysParams->semSeg.wallPairing.minimumOverlapRatio);

    std::vector<geometric::Plane *>                  wallPlanes;
    std::vector<vs_graphs::core::geometric::Plane *> atlasAllPlanes{};
    if (p_atlas->getAllPlanes(atlasAllPlanes) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (geometric::Plane *p_plane : atlasAllPlanes)
    {
        bool planeIsBad{};
        if ((p_plane != nullptr) &&
            p_plane->isBad(planeIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        geometric::Plane::PlaneVariant planeType{};
        if ((p_plane != nullptr && !planeIsBad) &&
            p_plane->getPlaneType(planeType) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPlaneType returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_plane != nullptr && !planeIsBad &&
            planeType == geometric::Plane::PlaneVariant::WALL)
        {
            wallPlanes.push_back(p_plane);
        }
    }
    /* Lock two Planes in ascending id order to avoid a lock-order hazard,
     * matching the convention already used for passages in
     * admitWallToRoom(). */
    std::sort(wallPlanes.begin(),
              wallPlanes.end(),
              semantic::isEntityIdLess<geometric::Plane>);

    for (geometric::Plane *p_wall : wallPlanes)
    {
        geometric::Plane *p_existingTwin = nullptr;
        if (p_wall->getTwinFace(p_existingTwin) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getTwinFace returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_existingTwin != nullptr)
        {
            /* Cheap common case: re-validate rather than search again. A
             * lower-id plane already validated (and, if still plausible,
             * re-linked) this pair when it was itself visited. */
            int wallGetId{};
            if (p_wall->getId(wallGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int existingTwinGetId{};
            if (p_existingTwin->getId(existingTwinGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (wallGetId < existingTwinGetId)
            {
                continue;
            }

            bool arePlausibleTwinWallFaces2{};
            if (arePlausibleTwinWallFaces(p_wall,
                                          p_existingTwin,
                                          minimumThickness_m,
                                          maximumThickness_m,
                                          minimumOverlapRatio,
                                          groundNormal_world,
                                          arePlausibleTwinWallFaces2) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: arePlausibleTwinWallFaces returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (arePlausibleTwinWallFaces2)
            {
                continue;
            }

            /* Pairing is no longer plausible (e.g. one side drifted after a
             * refit) -- unlink both sides rather than leave a stale
             * one-directional pointer. */
            if (p_existingTwin->clearTwinFace() !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: clearTwinFace returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_wall->clearTwinFace() !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: clearTwinFace returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    for (std::size_t firstIndex = 0; firstIndex < wallPlanes.size();
         ++firstIndex)
    {
        geometric::Plane *p_first            = wallPlanes[firstIndex];
        geometric::Plane *p_firstGetTwinFace = nullptr;
        if (p_first->getTwinFace(p_firstGetTwinFace) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getTwinFace returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_firstGetTwinFace != nullptr)
        {
            continue;
        }

        geometric::Plane *p_bestMatch         = nullptr;
        double            bestOverlapRatio_m2 = -1.0;

        for (std::size_t secondIndex = firstIndex + 1;
             secondIndex < wallPlanes.size();
             ++secondIndex)
        {
            geometric::Plane *p_second            = wallPlanes[secondIndex];
            geometric::Plane *p_secondGetTwinFace = nullptr;
            if (p_second->getTwinFace(p_secondGetTwinFace) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getTwinFace returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_secondGetTwinFace != nullptr)
            {
                continue;
            }

            bool arePlausibleTwinWallFaces3{};
            if (arePlausibleTwinWallFaces(p_first,
                                          p_second,
                                          minimumThickness_m,
                                          maximumThickness_m,
                                          minimumOverlapRatio,
                                          groundNormal_world,
                                          arePlausibleTwinWallFaces3) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: arePlausibleTwinWallFaces returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (!arePlausibleTwinWallFaces3)
            {
                continue;
            }

            /* Prefer the most-overlapping plausible candidate when more
             * than one exists, using observation count as a simple,
             * deterministic tiebreaker proxy for "most overlap". */
            std::size_t secondGetObservationCount{};
            if (p_second->getObservationCount(secondGetObservationCount) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getObservationCount returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            const double candidateScore =
                static_cast<double>(secondGetObservationCount);
            if (p_bestMatch == nullptr || candidateScore > bestOverlapRatio_m2)
            {
                p_bestMatch         = p_second;
                bestOverlapRatio_m2 = candidateScore;
            }
        }

        if (p_bestMatch != nullptr)
        {
            if (p_first->setTwinFace(p_bestMatch) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setTwinFace returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_bestMatch->setTwinFace(p_first) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setTwinFace returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            int firstGetId{};
            if (p_first->getId(firstGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int bestMatchGetId{};
            if (p_bestMatch->getId(bestMatchGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] Linked Wall#" << firstGetId << " and Wall#"
                      << bestMatchGetId
                      << " as opposite faces of one physical wall."
                      << std::endl;
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

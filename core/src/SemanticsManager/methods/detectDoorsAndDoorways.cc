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

#include "GeoSemHelpers.h"
#include "GeoSemHelpersStatus.h"
#include "SemanticsManager.h"
#include "Utils/Utils/objects/Utils.h"
#include "Utils/Utils/objects/UtilsStatus.h"

#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::detectDoorsAndDoorways(vs_graphs::core::Atlas *p_atlas_in)
{
    /* Confirm that the Atlas is valid */
    if (p_atlas_in == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Extract all planes from the current map */
    std::vector<vs_graphs::core::geometric::Plane *> allPlanes{};
    if (p_atlas_in->getAllPlanes(allPlanes) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Initialise lists of valid wall and door planes */
    std::vector<vs_graphs::core::geometric::Plane *> wallPlanes;
    std::vector<vs_graphs::core::geometric::Plane *> doorPlanes;

    wallPlanes.reserve(allPlanes.size());
    doorPlanes.reserve(allPlanes.size());

    /* Separate mapped planes according to their confirmed semantic type */
    for (vs_graphs::core::geometric::Plane *p_plane : allPlanes)
    {
        /* Skip invalid mapped planes */
        bool planeIsBad{};
        if (!(p_plane == nullptr) &&
            p_plane->isBad(planeIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_plane == nullptr || planeIsBad)
        {
            continue;
        }

        /* Store confirmed wall planes */
        geometric::Plane::PlaneVariant planeType{};
        if (p_plane->getPlaneType(planeType) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPlaneType returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (planeType == vs_graphs::core::geometric::Plane::PlaneVariant::WALL)
        {
            wallPlanes.push_back(p_plane);
            continue;
        }

        /* Store confirmed door planes */
        geometric::Plane::PlaneVariant planeType2{};
        if (p_plane->getPlaneType(planeType2) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPlaneType returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (planeType2 == vs_graphs::core::geometric::Plane::PlaneVariant::DOOR)
        {
            doorPlanes.push_back(p_plane);
        }
    }

    /* Filter wall planes to those with sufficient observations.
     * Provisional rooms are valid evidence - don't require CONFIRMED room
     * association. */
    std::vector<vs_graphs::core::geometric::Plane *> confirmedWallPlanes;
    confirmedWallPlanes.reserve(wallPlanes.size());

    for (vs_graphs::core::geometric::Plane *p_wall : wallPlanes)
    {
        bool wallIsBad{};
        if (!(p_wall == nullptr) &&
            p_wall->isBad(wallIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_wall == nullptr || wallIsBad)
        {
            continue;
        }

        // Quality gate: minimum observations, not room confirmation status
        std::size_t wallGetObservationCount{};
        if (p_wall->getObservationCount(wallGetObservationCount) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getObservationCount returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (wallGetObservationCount >=
            p_sysParams->roomSeg.minimumWallObservationCount)
        {
            confirmedWallPlanes.push_back(p_wall);
        }
        else
        {
            int wallGetId{};
            if (p_wall->getId(wallGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::size_t wallGetObservationCount2{};
            if (p_wall->getObservationCount(wallGetObservationCount2) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getObservationCount returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            std::cout << "[SemMgr] Skipping wall " << wallGetId
                      << " for passage detection: insufficient observations ("
                      << wallGetObservationCount2 << " < "
                      << p_sysParams->roomSeg.minimumWallObservationCount
                      << ")." << std::endl;
        }
    }

    /*  Detect blocked passages represented by closed semantic door planes */
    for (vs_graphs::core::geometric::Plane *p_door : doorPlanes)
    {
        /* Skip invalid door planes */
        bool doorIsBad{};
        if (!(p_door == nullptr) &&
            p_door->isBad(doorIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_door == nullptr || doorIsBad)
        {
            continue;
        }

        for (vs_graphs::core::geometric::Plane *p_wall : confirmedWallPlanes)
        {
            /* Skip invalid wall planes */
            bool wallIsBad2{};
            if (!(p_wall == nullptr) &&
                p_wall->isBad(wallIsBad2) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_wall == nullptr || wallIsBad2)
            {
                continue;
            }

            /* Door and wall must be parallel */
            bool arePlanesParallel2{};
            if (utils::utils::Utils::arePlanesParallel(p_door,
                                                       p_wall,
                                                       arePlanesParallel2) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: arePlanesParallel returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (!arePlanesParallel2)
            {
                continue;
            }

            /* Door must lie close to the supporting wall */
            bool arePlanesApartEnough2{};
            if (utils::utils::Utils::arePlanesApartEnough(
                    p_door,
                    p_wall,
                    p_sysParams->semSeg.maxWallDoorDistance,
                    arePlanesApartEnough2) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: arePlanesApartEnough returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (arePlanesApartEnough2)
            {
                continue;
            }

            /* Create a blocked passage associated with the supporting wall */
            if (GeoSemHelpers::createMapPassage(p_atlas,
                                                p_door,
                                                p_wall,
                                                false) !=
                GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: createMapPassage returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    /*!
     * Detect open passages from connected Voxblox skeleton edges which breach
     * finite mapped wall surfaces.
     *
     * @note        Camera trajectory crossings are deliberately not used.
     */
    if (detectOpenPassagesFromSkeletonEdges(confirmedWallPlanes) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: detectOpenPassagesFromSkeletonEdges returned a failure status "
            "although it cannot fail; continuing as before.",
            __func__);
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

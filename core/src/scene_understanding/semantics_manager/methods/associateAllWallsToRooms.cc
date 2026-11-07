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
 * @file            associateAllWallsToRooms.cc
 *
 * @brief           Implements SemanticsManager::associateAllWallsToRooms(),
 *                  declared in SemanticsManager.h.
 */

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/Room/Room.h"
#include "scene_understanding/scene_graph/Room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <rclcpp/logging.hpp>

#include "execution/profiling/profiling.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::associateAllWallsToRooms(void)
{
    PROFILE_FUNCTION();

    /*!
     * A wall must be observed from several keyframes before it is inserted into
     * the higher-level semantic hierarchy.
     */

    /* Initialize list to hold all planes from the map */
    std::vector<vs_graphs::core::geometric::Plane *> allPlanes{};

    /* Extract all planes from the current map */
    if (p_atlas->getAllPlanes(allPlanes) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /*!
     * The atlas hands planes out in pointer order. Admit nearer walls first
     * so a wall seen through a doorway meets the doorway wall already in
     * the room (see admitWallToRoom()'s see-through gate).
     */
    if (sortWallsByCameraDistance(allPlanes) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: sortWallsByCameraDistance returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Initialize list to hold all rooms from the map */
    std::vector<vs_graphs::core::semantic::Room *> allRooms{};

    /* Extract all current rooms and provisional structural elements */
    if (p_atlas->getAllRooms(allRooms) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Init a pointer to track current active map */
    map::Map *p_activeMap = nullptr;

    /* Extract the current active map */
    if (p_atlas->getCurrentMap(p_activeMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Init a pointer to track the current room the UAV is in */
    semantic::Room *p_currentRoom = nullptr;

    /* Extract the room object that the robot is currently in */
    if (getCurrentRoom(p_currentRoom) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentRoom returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /* Init pointer to hold largest ground plane */
    geometric::Plane *p_groundPlaneForEvidence = nullptr;

    /* Extract the largest ground plane from the map */
    if (p_atlas->getBiggestGroundPlane(p_groundPlaneForEvidence) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Init vector for ground plane normal */
    Eigen::Vector3d groundNormalForEvidence_world = Eigen::Vector3d::Zero();

    /* Init flag to indicate if ground plane is bad */
    bool groundPlaneForEvidenceIsBad{};

    /* Extract ground plane */
    if ((p_groundPlaneForEvidence != nullptr) &&
        p_groundPlaneForEvidence->isBad(groundPlaneForEvidenceIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    /* Confirm ground plane is good, and if so get global equation */
    if (p_groundPlaneForEvidence != nullptr && !groundPlaneForEvidenceIsBad)
    {
        /* Init variable to hold ground plane equation */
        g2o::Plane3D groundPlaneForEvidenceGetGlobalEquation{};

        /* Extract ground plane global equation */
        if (p_groundPlaneForEvidence->getGlobalEquation(
                groundPlaneForEvidenceGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Extract coefficients of ground plane equation */
        const Eigen::Vector4d groundEquation =
            groundPlaneForEvidenceGetGlobalEquation.coeffs();

        /* Extract ground plane normal */
        const double groundEquationNormalNorm = groundEquation.head<3>().norm();

        /* Normalize the ground plane equation */
        if (groundEquation.allFinite() && groundEquationNormalNorm > 1e-8)
        {
            groundNormalForEvidence_world =
                groundEquation.head<3>() / groundEquationNormalNorm;
        }
    }

    /* Iterate through every mapped plane */
    for (vs_graphs::core::geometric::Plane *p_plane : allPlanes)
    {
        /* Init flag to indicate if wall is bad */
        bool planeIsBad{};

        /* Check is wall is bad */
        if (!(p_plane == nullptr) &&
            p_plane->isBad(planeIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        /* Skip invalid planes */
        if (p_plane == nullptr || planeIsBad)
        {
            continue;
        }

        /* Init variable which will track the wall id */
        int planeId{};

        /* Get the ID number of the plane */
        if (p_plane->getId(planeId) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        /* Init object which will hold the plane variant */
        geometric::Plane::PlaneVariant planeVariant{};

        /* Extract the variant of the plane */
        if (p_plane->getPlaneType(planeVariant) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPlaneType returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Define a variable to get string output of plane variant type */
        const char *variantTypeName = "UNDEFINED";

        /* Extract the string which describes the variant type */
        if (getPlaneVariantName(planeVariant, variantTypeName) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPlaneVariantName returned a failure "
                         "status although it cannot fail; continuing as "
                         "before.",
                         __func__);
        }

        /*!
         * Only walls are associated to rooms. A plane that is no longer
         * labelled WALL leaves the room owning it, in this cycle. It returns
         * through the normal admission paths once it is a wall again.
         */
        if (planeVariant != geometric::Plane::PlaneVariant::WALL)
        {
            /* A plane that is no longer a wall has no rejection to remember */
            loggedWallRejectionReasons.erase(planeId);

            /* Declare variable which will store room holding plane */
            semantic::Room *p_owningRoom = nullptr;

            /* Find the room which holds the plane */
            if (findWallRoom(allRooms, p_plane, p_owningRoom) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: findWallRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            /* Remove the plane from that room and log the removal */
            if (p_owningRoom != nullptr)
            {
                /* Init flag which will confirm if the plane was removed */
                bool wasWallRemoved{};

                if (p_owningRoom->removeWall(p_plane, wasWallRemoved) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                "%s: removeWall rejected its input; "
                                "continuing as before.",
                                __func__);
                    wasWallRemoved = false;
                }

                if (wasWallRemoved)
                {
                    /* Init integer to hold the id of the room */
                    int owningRoomId{};

                    if (p_owningRoom->getId(owningRoomId) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: getId returned a failure status "
                                     "although it cannot fail; continuing as "
                                     "before.",
                                     __func__);
                    }

                    RCLCPP_INFO(rclcpp::get_logger("vs_graphs"),
                                "[SemMgr] roomwall remove wall=%d room=%d: "
                                "label is %s, not WALL.",
                                planeId,
                                owningRoomId,
                                variantTypeName);
                }
            }

            /* Continue onto next plane */
            continue;
        }

        /* Declare long integer which will track the id of the active map */
        unsigned long activeMapId{};

        /* Extract the id of the active map */
        if ((p_activeMap != nullptr) && p_activeMap->getId(activeMapId) !=
                                            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        /* Machine-readable records report -1 when there is no active map. */
        const long long activeMapIdRecord =
            (p_activeMap != nullptr) ? static_cast<long long>(activeMapId) : -1;

        /* Declare struct which will contain evidence on wall admission */
        WallAdmissionEvidence admissionEvidence{};

        /* Assess whether a wall should be admitted to a room */
        if (evaluateWallAdmissionEvidence(p_plane,
                                          p_sysParams,
                                          groundNormalForEvidence_world,
                                          admissionEvidence) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: evaluateWallAdmissionEvidence returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* Walls that fail the admission evidence are rejected for now. */
        if (!admissionEvidence.isAdmissible)
        {
            /* Init variable which will hold the voted variant of the plane */
            geometric::Plane::PlaneVariant votedPlanVariant{};

            /* Extract the expected plane variation */
            if (p_plane->getExpectedPlaneType(votedPlanVariant) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getExpectedPlaneType returned a failure "
                             "status although it cannot fail; continuing as "
                             "before.",
                             __func__);
            }

            /* Init variable which will track reason plane should be rejected */
            std::string rejectionReason;

            /* The votes outrank the fit, which outranks the observation age */
            if (votedPlanVariant != geometric::Plane::PlaneVariant::WALL)
            {
                rejectionReason = "CLASS_NOT_WALL";
            }
            else if (!admissionEvidence.hasAdequateFiniteFit)
            {
                rejectionReason = "INADEQUATE_FINITE_FIT";
            }
            else
            {
                rejectionReason = "INSUFFICIENT_OBSERVATIONS";
            }

            /* Emit the rejection record only when the reason changes. */
            if (loggedWallRejectionReasons[planeId] != rejectionReason)
            {
                loggedWallRejectionReasons[planeId] = rejectionReason;
                std::cout << "SG_PIPELINE {\"event\":\"wall_rejection\","
                             "\"map_id\":"
                          << activeMapIdRecord
                          << ",\"semantic_cycle\":" << pipelineSemanticCycle
                          << ",\"wall_id\":" << planeId << ",\"class\":\""
                          << variantTypeName
                          << "\",\"lifecycle\":\"REJECTED\","
                             "\"owner\":\"NONE\",\"reason\":\""
                          << rejectionReason << "\",\"support\":"
                          << admissionEvidence.fittedPointCount
                          << ",\"observations\":"
                          << admissionEvidence.observationCount << "}"
                          << std::endl;
            }
            continue;
        }

        /* The wall is admissible again, so forget its earlier rejection. */
        loggedWallRejectionReasons.erase(planeId);

        /* Init variable which will track the room the wall belongs to */
        semantic::Room *p_roomContainingWall = nullptr;

        /* Find room containing the current wall */
        if (findWallRoom(allRooms, p_plane, p_roomContainingWall) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: findWallRoom returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* If the room containing wall is invalid, skip to next plane */
        if (p_roomContainingWall != nullptr)
        {
            continue;
        }

        /* Declare flag to indicate wall should be admitted to current room */
        bool wasWallAdmitted{};

        /* Admit wall to current room */
        if ((p_currentRoom != nullptr) &&
            admitWallToRoom(p_currentRoom, p_plane, wasWallAdmitted) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: admitWallToRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Init flag to indicate if the wall was admitted */
        const bool isWallAdmitted = p_currentRoom != nullptr && wasWallAdmitted;

        /*!
         * Look the owner up again in the atlas rooms. It may be a room other
         * than the current one (passage far-side precedence).
         */
        std::vector<semantic::Room *> atlasAllRooms{};
        if (p_atlas->getAllRooms(atlasAllRooms) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllRooms returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        semantic::Room *p_selectedOwner = nullptr;
        if (findWallRoom(atlasAllRooms, p_plane, p_selectedOwner) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: findWallRoom returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /*!
         * Register the wall in the atlas room-wall collection, whether or
         * not a room owns it yet, so a later passage or room detection can
         * still find it.
         */
        geometric::Plane *p_atlasRoomWallPlane = nullptr;
        if (p_atlas->getRoomWallPlaneById(planeId, p_atlasRoomWallPlane) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomWallPlaneById returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_atlasRoomWallPlane == nullptr &&
            p_atlas->addRoomWallPlane(p_plane) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addRoomWallPlane returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* The wall is committed: clear its pending bookkeeping and record. */
        if (isWallAdmitted && p_selectedOwner != nullptr)
        {
            undefendedWalls.erase(planeId);
            loggedOrphanWallIds.erase(planeId);

            int selectedOwnerId{};
            if (p_selectedOwner->getId(selectedOwnerId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }

            std::cout << "SG_PIPELINE {\"event\":\"wall_admission\","
                         "\"map_id\":"
                      << activeMapIdRecord
                      << ",\"semantic_cycle\":" << pipelineSemanticCycle
                      << ",\"wall_id\":" << planeId
                      << ",\"class\":\"WALL\","
                         "\"lifecycle\":\"COMMITTED\",\"owner_room_id\":"
                      << selectedOwnerId << ",\"reason\":\""
                      << (p_selectedOwner == p_currentRoom
                              ? "CURRENT_ROOM_OBSERVATION"
                              : "PASSAGE_FAR_SIDE_PRECEDENCE")
                      << "\",\"support\":" << admissionEvidence.fittedPointCount
                      << ",\"observations\":"
                      << admissionEvidence.observationCount << "}" << std::endl;
            continue;
        }

        /*!
         * No room took the wall. It stays registered above for future
         * association via passages or room detection. No provisional room is
         * created for an orphan wall - only passages create prospective
         * rooms. The pending record is emitted once per wall.
         */
        if (loggedOrphanWallIds.insert(planeId).second)
        {
            std::cout << "SG_PIPELINE {\"event\":\"wall_pending\","
                         "\"map_id\":"
                      << activeMapIdRecord
                      << ",\"semantic_cycle\":" << pipelineSemanticCycle
                      << ",\"wall_id\":" << planeId
                      << ",\"class\":\"WALL\","
                         "\"lifecycle\":\"PENDING\",\"owner\":\"PENDING\","
                         "\"reason\":\""
                      << (p_currentRoom == nullptr ? "NO_CURRENT_ROOM"
                                                   : "SAFE_ADMISSION_REJECTED")
                      << "\",\"support\":" << admissionEvidence.fittedPointCount
                      << ",\"observations\":"
                      << admissionEvidence.observationCount
                      << ",\"pending_age\":0}" << std::endl;
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */

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
 * @file            refreshDuplicatePassage.cc
 *
 * @brief           Implements SemanticsManager::refreshDuplicatePassage(),
 * declared in SemanticsManagerClass.h.
 */

#include "configuration/types/objects/SystemParamsStatusEnum.h"
#include "configuration/types/types.h"
#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <rclcpp/logging.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::refreshDuplicatePassage(
    const std::vector<vs_graphs::core::semantic::Passage *> &allPassages_in,
    vs_graphs::core::geometric::Plane                       *p_doorPlane_in,
    vs_graphs::core::geometric::Plane                       *p_wallPlane_in,
    bool                                                     isOpenPassage_in,
    const Eigen::Vector3d &passageCentroid_world_m_in,
    const g2o::Plane3D    &passageEquation_in,
    bool                  &isCreationNeeded_out)
{
    isCreationNeeded_out = false;

    /* Extract the duplicate distacne threshold */
    types::SystemParams *p_params6 = nullptr;
    if (types::SystemParams::getParams(p_params6) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    const double duplicateDistanceThreshold =
        p_params6->semSeg.passageCentroidDistanceThresh;

    /* Extract parameters from passage equation */
    Eigen::Vector4d candidateEquation = passageEquation_in.coeffs();

    /* Extract the normal norm from the candidate equation */
    const double candidateNormalNorm = candidateEquation.head<3>().norm();

    /* Confirm norm is valid */
    if (candidateNormalNorm < 1e-8)
    {
        std::cerr << "[GeoSemHelper] Cannot create pasasge: invalid candidate "
                     "plane equation"
                  << std::endl;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Extract the unit norm of the plane */
    const Eigen::Vector3d candidateNormal =
        candidateEquation.head<3>() / candidateNormalNorm;

    /* Iterate through existing passages and see if any passage matches */
    for (vs_graphs::core::semantic::Passage *p_existingPassage : allPassages_in)
    {
        /* Confirm that existing passage is valid*/
        if (p_existingPassage == nullptr)
        {
            continue;
        }

        /* Find the distance from centroid to passage */
        Eigen::Vector3d existingPassageCentroid{};
        if (p_existingPassage->getCentroid(existingPassageCentroid) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3d centroidDistanceVector =
            (passageCentroid_world_m_in - existingPassageCentroid);

        /* Find distance from candidate passage to existing passage centroid */
        const double centroidDistance = centroidDistanceVector.norm();

        /* If distance is greater than threshold, skip */
        if (centroidDistance >= duplicateDistanceThreshold)
        {
            continue;
        }

        /* Extract the equation of the existing passage */
        g2o::Plane3D existingPassageGlobalEquation{};
        if (p_existingPassage->getGlobalEquation(
                existingPassageGlobalEquation) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector4d existingEquation =
            existingPassageGlobalEquation.coeffs();

        /* Find the normal norm of the existing plane */
        const double existingNormalNorm = existingEquation.head<3>().norm();

        /* Confirm that the norm is valid, otherwise skip */
        if (existingNormalNorm < 1e-8)
        {
            continue;
        }

        /* Extract the unit vector of the norm */
        const Eigen::Vector3d existingNormal =
            existingEquation.head<3>() / existingNormalNorm;

        /*!
         * Find the absolute dot product handles opposite representations of the
         * same plane normal.
         */
        const double normalAlignment =
            std::abs(candidateNormal.dot(existingNormal));

        /* Define a constant expression for the parrallel normal threshold */
        constexpr double parallelNormalThreshold = 0.95;

        /*!
         * If normal alignment is outside threshold skip.
         *
         * @note            normalAlignment = cos(theta),
         *                  normalAlignment = 1 @ theta = 0
         */
        if (normalAlignment < parallelNormalThreshold)
        {
            continue;
        }

        /*
         * Parallel openings on unrelated nearby walls are not duplicates.
         * Permit the two observed faces of one physical wall, but require the
         * passage centroids to remain close to the opposite passage plane.
         */
        constexpr double      maximumSupportingPlaneSeparation_m = 0.30;
        const Eigen::Vector4d normalizedCandidateEquation =
            candidateEquation / candidateNormalNorm;
        const Eigen::Vector4d normalizedExistingEquation =
            existingEquation / existingNormalNorm;
        Eigen::Vector3d existingPassageCentroid2{};
        if (p_existingPassage->getCentroid(existingPassageCentroid2) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const double candidatePlaneResidual_m =
            std::abs(normalizedCandidateEquation.head<3>().dot(
                         existingPassageCentroid2) +
                     normalizedCandidateEquation(3));
        const double existingPlaneResidual_m =
            std::abs(normalizedExistingEquation.head<3>().dot(
                         passageCentroid_world_m_in) +
                     normalizedExistingEquation(3));

        if (candidatePlaneResidual_m > maximumSupportingPlaneSeparation_m ||
            existingPlaneResidual_m > maximumSupportingPlaneSeparation_m)
        {
            continue;
        }

        /*
         * Passage state is evidence, not identity. Confirmed connected free
         * space promotes a prior passage hypothesis to traversable; later door
         * observations never downgrade that stronger evidence.
         *
         * A passage is expected to have TWO observed wall faces that offset a
         * physical wall (the near face and the opposite face). Geometry must
         * never be re-anchored onto the face that happens to have been seen
         * last, or the passage plane flips between the two faces from cycle to
         * cycle and every side-dependent decision (far-side holds, prospective
         * placement) flips with it. Therefore:
         *   - When this wall is already an associated supporting face, refresh
         *     centroid/equation normally (same-face refinement).
         *   - When this is the FIRST time we see the *opposite* face of the
         *     same physical wall (not yet among the supporting faces), only add
         *     it to the passage. The passage plane stays anchored to the face
         *     that framed the passage; a downstream mid-plane pass pairs the
         *     two faces and recomputes a stable aperture plane.
         */
        std::vector<vs_graphs::core::geometric::Plane *>
            existingSupportingWalls{};
        if (p_existingPassage->getAssociateWalls(existingSupportingWalls) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAssociateWalls returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const bool isKnownSupportingFace =
            std::find(existingSupportingWalls.begin(),
                      existingSupportingWalls.end(),
                      p_wallPlane_in) != existingSupportingWalls.end();

        if (isOpenPassage_in)
        {
            if (p_existingPassage->setPassable(true) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setPassable returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_existingPassage->setCentroid(passageCentroid_world_m_in) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (isKnownSupportingFace)
            {
                if (p_existingPassage->setGlobalEquation(passageEquation_in) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setGlobalEquation returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
        }

        if (p_existingPassage->addAssociateWall(p_wallPlane_in) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addAssociateWall returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        vs_graphs::core::geometric::Plane *p_existingPassageAssociateDoor =
            nullptr;
        if ((p_doorPlane_in != nullptr) &&
            p_existingPassage->getAssociateDoor(
                p_existingPassageAssociateDoor) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAssociateDoor returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_doorPlane_in != nullptr &&
            p_existingPassageAssociateDoor == nullptr)
        {
            if (p_existingPassage->setAssociateDoor(p_doorPlane_in) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setAssociateDoor returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        int existingPassageId{};
        if (p_existingPassage->getId(existingPassageId) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        bool existingPassageIsPassable{};
        if (p_existingPassage->isPassable(existingPassageIsPassable) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isPassable returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "[GeoSemHelper] Updated existing Passage#"
                  << existingPassageId
                  << ": centroid distance=" << centroidDistance
                  << " m, normal alignment=" << normalAlignment << ", state="
                  << (existingPassageIsPassable ? "open" : "blocked") << "."
                  << std::endl;

        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    isCreationNeeded_out = true;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */

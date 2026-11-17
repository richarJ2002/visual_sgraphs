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
 * @file            reanchorOpenPassageToWalls.cc
 *
 * @brief           Implements SemanticsManager::reanchorOpenPassageToWalls(),
 * declared in SemanticsManagerClass.h.
 */

#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::reanchorOpenPassageToWalls(
    semantic::Passage                 *p_passage_inout,
    vs_graphs::core::geometric::Plane *p_groundPlane_in)
{
    /*
     * Open passages are derived from a crossing of their supporting
     * wall. Re-anchor them after every optimization so an independently
     * corrected plane cannot leave the passage or graph edge behind.
     *
     * A passage is expected to be framed by TWO observed wall faces
     * (the two surfaces of the same physical wall, offset by the wall
     * thickness). The stable aperture plane for side discrimination is
     * the MID-PLANE between the two faces, so that a wall on either
     * side of the opening always lies the wall-thickness away from the
     * passage plane (never ON it). Anchoring to only the closest face
     * - or switching between nearest and farthest face across cycles -
     * makes the passage normal flip and breaks the far-side holds.
     */
    std::vector<vs_graphs::core::geometric::Plane *> supportingFaces{};
    if (p_passage_inout->getAssociateWalls(supportingFaces) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAssociateWalls returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    std::vector<Eigen::Vector4d> validFaceEquations;
    validFaceEquations.reserve(2);

    for (vs_graphs::core::geometric::Plane *p_candidateWall : supportingFaces)
    {
        bool candidateWallIsBad{};
        if (!(p_candidateWall == nullptr) &&
            p_candidateWall->isBad(candidateWallIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_candidateWall == nullptr || candidateWallIsBad)
        {
            return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
        }

        g2o::Plane3D candidateWallGetGlobalEquation{};
        if (p_candidateWall->getGlobalEquation(
                candidateWallGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector4d candidateWallEquation =
            candidateWallGetGlobalEquation.coeffs();

        if (!candidateWallEquation.allFinite())
        {
            return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
        }

        const double candidateNormalNorm =
            candidateWallEquation.head<3>().norm();

        if (!std::isfinite(candidateNormalNorm) || candidateNormalNorm < 1e-8)
        {
            return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
        }

        candidateWallEquation /= candidateNormalNorm;

        if (validFaceEquations.size() < 2)
        {
            validFaceEquations.push_back(candidateWallEquation);
        }
    }

    Eigen::Vector3d passageCentroid_world_m{};
    if (p_passage_inout->getCentroid(passageCentroid_world_m) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    if (validFaceEquations.size() == 2)
    {
        /*
         * Two faces of the same physical wall have normals that are
         * antiparallel (each oriented away from its own room). Orient
         * them consistently as (n, d) along a shared unit normal and
         * take the mid-plane. The lower face is used as the reference
         * orientation so the resulting aperture plane does not depend
         * on which face happened to produce the passage first.
         */
        Eigen::Vector4d firstFace  = validFaceEquations[0];
        Eigen::Vector4d secondFace = validFaceEquations[1];

        /*
         * Orient both faces to the same unit normal: use the normal of
         * the first face as the shared reference orientation.
         */
        const double alignmentSecondFace =
            firstFace.head<3>().dot(secondFace.head<3>());

        if (alignmentSecondFace < 0.0)
        {
            secondFace *= -1.0;
        }

        /* The mid-plane offset keeps the face centroids symmetric. */
        const double nearFaceDistance_m =
            firstFace.head<3>().dot(passageCentroid_world_m) + firstFace(3);
        const double farFaceDistance_m =
            secondFace.head<3>().dot(passageCentroid_world_m) + secondFace(3);

        const double midPlaneDistance_m =
            0.5 * (nearFaceDistance_m + farFaceDistance_m);

        Eigen::Vector4d midPlaneEquation;
        midPlaneEquation.head<3>() = firstFace.head<3>();
        midPlaneEquation(3)        = midPlaneDistance_m -
                              firstFace.head<3>().dot(passageCentroid_world_m);

        vs_graphs::core::geometric::Plane passagePlane;
        if (passagePlane.setGlobalEquation(g2o::Plane3D(midPlaneEquation)) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        bool arePlanesPerpendicular2{};
        if (utils::utils::Utils::arePlanesPerpendicular(
                &passagePlane,
                p_groundPlane_in,
                arePlanesPerpendicular2) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: arePlanesPerpendicular returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (arePlanesPerpendicular2)
        {
            if (p_passage_inout->setGlobalEquation(
                    g2o::Plane3D(midPlaneEquation)) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setGlobalEquation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            /*
             * The centre lies on the mid-plane, like the single-face
             * case lies on its face. The face normal is a unit vector,
             * so midPlaneDistance_m is the signed distance from the
             * centre to the mid-plane. Moving the centre by that
             * distance makes it follow the optimised wall planes
             * instead of staying where an earlier cycle left it.
             */
            const Eigen::Vector3d midPlaneCentroid_world_m =
                passageCentroid_world_m -
                midPlaneDistance_m * firstFace.head<3>();
            if (p_passage_inout->setCentroid(midPlaneCentroid_world_m) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
        else
        {
            /* Mid-plane deviates from vertical; fall back to the
             * single-face anchoring for this cycle. */
            Eigen::Vector4d referenceEquation        = validFaceEquations[0];
            Eigen::Vector3d anchoredCentroid_world_m = passageCentroid_world_m;
            const double    wallResidual_m =
                referenceEquation.head<3>().dot(anchoredCentroid_world_m) +
                referenceEquation(3);

            anchoredCentroid_world_m -=
                wallResidual_m * referenceEquation.head<3>();

            if (p_passage_inout->setCentroid(anchoredCentroid_world_m) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_passage_inout->setGlobalEquation(
                    g2o::Plane3D(referenceEquation)) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setGlobalEquation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }
    else if (validFaceEquations.size() == 1)
    {
        /*
         * Only one face of the physical wall has been observed so far.
         * Anchor the passage plane to that face.
         */
        Eigen::Vector4d referenceEquation        = validFaceEquations[0];
        Eigen::Vector3d anchoredCentroid_world_m = passageCentroid_world_m;
        const double    wallResidual_m =
            referenceEquation.head<3>().dot(anchoredCentroid_world_m) +
            referenceEquation(3);

        anchoredCentroid_world_m -=
            wallResidual_m * referenceEquation.head<3>();

        if (p_passage_inout->setCentroid(anchoredCentroid_world_m) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: setCentroid returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }
        if (p_passage_inout->setGlobalEquation(
                g2o::Plane3D(referenceEquation)) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    vs_graphs::core::geometric::Plane passagePlane;
    g2o::Plane3D                      passageGlobalEquation{};
    if (p_passage_inout->getGlobalEquation(passageGlobalEquation) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalEquation returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (passagePlane.setGlobalEquation(passageGlobalEquation) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setGlobalEquation returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    bool arePlanesPerpendicular3{};
    if (utils::utils::Utils::arePlanesPerpendicular(&passagePlane,
                                                    p_groundPlane_in,
                                                    arePlanesPerpendicular3) !=
        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: arePlanesPerpendicular returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (!arePlanesPerpendicular3)
    {
        /*!
         * Project the passage normal onto the horizontal plane to
         * remove tilt
         */
        g2o::Plane3D groundPlaneGetGlobalEquation{};
        if (p_groundPlane_in->getGlobalEquation(groundPlaneGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector3d groundNormal =
            groundPlaneGetGlobalEquation.coeffs().head<3>().normalized();
        g2o::Plane3D passageGlobalEquation2{};
        if (p_passage_inout->getGlobalEquation(passageGlobalEquation2) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector4d globalEq      = passageGlobalEquation2.coeffs();
        Eigen::Vector3d passageNormal = globalEq.head<3>().normalized();

        Eigen::Vector3d correctedNormal =
            (passageNormal - passageNormal.dot(groundNormal) * groundNormal)
                .normalized();
        if (correctedNormal.norm() < 1e-6)
            return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
        correctedNormal.normalize();

        /* Recompute d so the plane still passes through the centroid */
        Eigen::Vector3d passageCentroid{};
        if (p_passage_inout->getCentroid(passageCentroid) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getCentroid returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }
        const Eigen::Vector3d centroid = passageCentroid.cast<double>();
        const double          d        = -correctedNormal.dot(centroid);

        Eigen::Vector4d correctedCoeffs;
        correctedCoeffs.head<3>() = correctedNormal;
        correctedCoeffs(3)        = d;

        if (p_passage_inout->setGlobalEquation(g2o::Plane3D(correctedCoeffs)) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */

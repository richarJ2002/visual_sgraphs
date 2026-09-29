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

#include "SemanticsManager.h"
#include "Utils/Utils/objects/Utils.h"
#include "Utils/Utils/objects/UtilsStatus.h"

#include <cmath>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::updatePassages(vs_graphs::core::Atlas *p_atlas_in)
{
    // Get the ground plane
    vs_graphs::core::geometric::Plane *p_groundPlane =
        p_atlas_in->getBiggestGroundPlane();
    if (p_groundPlane == nullptr)
        return;

    // Get all passages and update their global pose to be consistent with the
    // ground plane
    std::vector<vs_graphs::core::semantic::Passage *> allPassages =
        p_atlas_in->getAllPassages();

    for (const auto &passage : allPassages)
    {
        bool passageIsBad{};
        if (!(passage == nullptr) &&
            passage->isBad(passageIsBad) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            // isBad cannot fail; continue as before.
        }
        if (passage == nullptr || passageIsBad)
        {
            continue;
        }

        // Updating the dimensions of the passage based on the associated door
        // plane
        vs_graphs::core::geometric::Plane *p_doorPlane = nullptr;
        if (passage->getAssociateDoor(p_doorPlane) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            // getAssociateDoor cannot fail; continue as before.
        }

        // Blocked passages (closed doors) should be aligned with the ground
        // plane normal
        bool passageIsPassable{};
        if (passage->isPassable(passageIsPassable) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            // isPassable cannot fail; continue as before.
        }
        if (!passageIsPassable)
        {
            if (p_doorPlane == nullptr)
            {
                continue;
            }

            /* Extract width height supple of door */
            std::pair<double, double>          widthHeight{};
            geometric::Plane::GeometrySnapshot doorPlaneGetGeometrySnapshot{};
            if (p_doorPlane->getGeometrySnapshot(
                    doorPlaneGetGeometrySnapshot) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getGeometrySnapshot cannot fail; continue as before.
            }
            if (utils::utils::Utils::computePlaneWidthHeight(
                    doorPlaneGetGeometrySnapshot.supportCloud,
                    widthHeight) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                // computePlaneWidthHeight cannot fail; continue as before.
            }

            /* Extract the measured height and width */
            const double measuredWidth  = widthHeight.first;
            const double measuredHeight = widthHeight.second;

            /* Extract the max width */
            const double maximumWidth =
                static_cast<double>(p_sysParams->semSeg.maxDoorWidth);

            /* Extract the max height */
            const double maximumHeight =
                static_cast<double>(p_sysParams->semSeg.maxDoorHeight);

            /* Determine if dimensions are valid */
            const bool validDimensions =
                std::isfinite(measuredWidth) && std::isfinite(measuredHeight) &&
                measuredWidth > 0.0 && measuredHeight > 0.0 &&
                measuredWidth <= maximumWidth &&
                measuredHeight <= maximumHeight;

            if (!validDimensions)
            {
                int passageId{};
                if (passage->getId(passageId) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                int doorPlaneGetId{};
                if (p_doorPlane->getId(doorPlaneGetId) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                std::cout << "[SemanticsManager] Rejecting door plane "
                          << doorPlaneGetId << " for passage " << passageId
                          << ": measured dimensions " << measuredWidth << "x"
                          << measuredHeight << " m exceed limits "
                          << maximumWidth << "x" << maximumHeight << " m."
                          << std::endl;

                continue;
            }

            /* Set centroid of the door plane */
            Eigen::Vector3d doorPlaneGetCentroid{};
            if (p_doorPlane->getCentroid(doorPlaneGetCentroid) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getCentroid cannot fail; continue as before.
            }
            if (passage->setCentroid(doorPlaneGetCentroid) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // setCentroid cannot fail; continue as before.
            }

            /* Get the plane global equation */
            g2o::Plane3D doorPlaneGetGlobalEquation{};
            if (p_doorPlane->getGlobalEquation(doorPlaneGetGlobalEquation) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getGlobalEquation cannot fail; continue as before.
            }
            if (passage->setGlobalEquation(doorPlaneGetGlobalEquation) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // setGlobalEquation cannot fail; continue as before.
            }

            /* Set width & height of the passage */
            if (passage->setWidth(measuredWidth) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // setWidth cannot fail; continue as before.
            }
            if (passage->setHeight(measuredHeight) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // setHeight cannot fail; continue as before.
            }
        }
        else
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
            if (passage->getAssociateWalls(supportingFaces) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getAssociateWalls cannot fail; continue as before.
            }

            std::vector<Eigen::Vector4d> validFaceEquations;
            validFaceEquations.reserve(2);

            for (vs_graphs::core::geometric::Plane *p_candidateWall :
                 supportingFaces)
            {
                bool candidateWallIsBad{};
                if (!(p_candidateWall == nullptr) &&
                    p_candidateWall->isBad(candidateWallIsBad) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // isBad cannot fail; continue as before.
                }
                if (p_candidateWall == nullptr || candidateWallIsBad)
                {
                    continue;
                }

                g2o::Plane3D candidateWallGetGlobalEquation{};
                if (p_candidateWall->getGlobalEquation(
                        candidateWallGetGlobalEquation) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // getGlobalEquation cannot fail; continue as before.
                }
                Eigen::Vector4d candidateWallEquation =
                    candidateWallGetGlobalEquation.coeffs();

                if (!candidateWallEquation.allFinite())
                {
                    continue;
                }

                const double candidateNormalNorm =
                    candidateWallEquation.head<3>().norm();

                if (!std::isfinite(candidateNormalNorm) ||
                    candidateNormalNorm < 1e-8)
                {
                    continue;
                }

                candidateWallEquation /= candidateNormalNorm;

                if (validFaceEquations.size() < 2)
                {
                    validFaceEquations.push_back(candidateWallEquation);
                }
            }

            Eigen::Vector3d passageCentroid_World_m{};
            if (passage->getCentroid(passageCentroid_World_m) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getCentroid cannot fail; continue as before.
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
                    firstFace.head<3>().dot(passageCentroid_World_m) +
                    firstFace(3);
                const double farFaceDistance_m =
                    secondFace.head<3>().dot(passageCentroid_World_m) +
                    secondFace(3);

                const double midPlaneDistance_m =
                    0.5 * (nearFaceDistance_m + farFaceDistance_m);

                Eigen::Vector4d midPlaneEquation;
                midPlaneEquation.head<3>() = firstFace.head<3>();
                midPlaneEquation(3) =
                    midPlaneDistance_m -
                    firstFace.head<3>().dot(passageCentroid_World_m);

                vs_graphs::core::geometric::Plane passagePlane;
                if (passagePlane.setGlobalEquation(
                        g2o::Plane3D(midPlaneEquation)) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // setGlobalEquation cannot fail; continue as before.
                }

                bool arePlanesPerpendicular2{};
                if (utils::utils::Utils::arePlanesPerpendicular(
                        &passagePlane,
                        p_groundPlane,
                        arePlanesPerpendicular2) !=
                    utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
                {
                    // arePlanesPerpendicular cannot fail; continue as before.
                }
                if (arePlanesPerpendicular2)
                {
                    if (passage->setGlobalEquation(
                            g2o::Plane3D(midPlaneEquation)) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        // setGlobalEquation cannot fail; continue as before.
                    }
                }
                else
                {
                    /* Mid-plane deviates from vertical; fall back to the
                     * single-face anchoring for this cycle. */
                    Eigen::Vector4d referenceEquation = validFaceEquations[0];
                    Eigen::Vector3d anchoredCentroid_World_m =
                        passageCentroid_World_m;
                    const double wallResidual_m =
                        referenceEquation.head<3>().dot(
                            anchoredCentroid_World_m) +
                        referenceEquation(3);

                    anchoredCentroid_World_m -=
                        wallResidual_m * referenceEquation.head<3>();

                    if (passage->setCentroid(anchoredCentroid_World_m) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        // setCentroid cannot fail; continue as before.
                    }
                    if (passage->setGlobalEquation(
                            g2o::Plane3D(referenceEquation)) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        // setGlobalEquation cannot fail; continue as before.
                    }
                }
            }
            else if (validFaceEquations.size() == 1)
            {
                /*
                 * Only one face of the physical wall has been observed so far.
                 * Anchor the passage plane to that face.
                 */
                Eigen::Vector4d referenceEquation = validFaceEquations[0];
                Eigen::Vector3d anchoredCentroid_World_m =
                    passageCentroid_World_m;
                const double wallResidual_m =
                    referenceEquation.head<3>().dot(anchoredCentroid_World_m) +
                    referenceEquation(3);

                anchoredCentroid_World_m -=
                    wallResidual_m * referenceEquation.head<3>();

                if (passage->setCentroid(anchoredCentroid_World_m) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // setCentroid cannot fail; continue as before.
                }
                if (passage->setGlobalEquation(
                        g2o::Plane3D(referenceEquation)) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // setGlobalEquation cannot fail; continue as before.
                }
            }

            vs_graphs::core::geometric::Plane passagePlane;
            g2o::Plane3D                      passageGlobalEquation{};
            if (passage->getGlobalEquation(passageGlobalEquation) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getGlobalEquation cannot fail; continue as before.
            }
            if (passagePlane.setGlobalEquation(passageGlobalEquation) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // setGlobalEquation cannot fail; continue as before.
            }
            bool arePlanesPerpendicular3{};
            if (utils::utils::Utils::arePlanesPerpendicular(
                    &passagePlane,
                    p_groundPlane,
                    arePlanesPerpendicular3) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                // arePlanesPerpendicular cannot fail; continue as before.
            }
            if (!arePlanesPerpendicular3)
            {
                // Project the passage normal onto the horizontal plane to
                // remove tilt
                g2o::Plane3D groundPlaneGetGlobalEquation{};
                if (p_groundPlane->getGlobalEquation(
                        groundPlaneGetGlobalEquation) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // getGlobalEquation cannot fail; continue as before.
                }
                const Eigen::Vector3d groundNormal =
                    groundPlaneGetGlobalEquation.coeffs()
                        .head<3>()
                        .normalized();
                g2o::Plane3D passageGlobalEquation2{};
                if (passage->getGlobalEquation(passageGlobalEquation2) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // getGlobalEquation cannot fail; continue as before.
                }
                Eigen::Vector4d globalEq      = passageGlobalEquation2.coeffs();
                Eigen::Vector3d passageNormal = globalEq.head<3>().normalized();

                Eigen::Vector3d correctedNormal =
                    (passageNormal -
                     passageNormal.dot(groundNormal) * groundNormal)
                        .normalized();
                if (correctedNormal.norm() < 1e-6)
                    continue;
                correctedNormal.normalize();

                // Recompute d so the plane still passes through the centroid
                Eigen::Vector3d passageCentroid{};
                if (passage->getCentroid(passageCentroid) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // getCentroid cannot fail; continue as before.
                }
                const Eigen::Vector3d centroid = passageCentroid.cast<double>();
                const double          d        = -correctedNormal.dot(centroid);

                Eigen::Vector4d correctedCoeffs;
                correctedCoeffs.head<3>() = correctedNormal;
                correctedCoeffs(3)        = d;

                if (passage->setGlobalEquation(g2o::Plane3D(correctedCoeffs)) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // setGlobalEquation cannot fail; continue as before.
                }
            }
        }
    }
}

} // namespace core
} // namespace vs_graphs

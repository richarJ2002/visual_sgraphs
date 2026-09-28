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
        if (passage == nullptr || passage->isBad())
        {
            continue;
        }

        // Updating the dimensions of the passage based on the associated door
        // plane
        vs_graphs::core::geometric::Plane *p_doorPlane =
            passage->getAssociateDoor();

        // Blocked passages (closed doors) should be aligned with the ground
        // plane normal
        if (!passage->isPassable())
        {
            if (p_doorPlane == nullptr)
            {
                continue;
            }

            /* Extract width height supple of door */
            std::pair<double, double> widthHeight =
                utils::utils::Utils::computePlaneWidthHeight(
                    p_doorPlane->getGeometrySnapshot().supportCloud);

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
                std::cout << "[SemanticsManager] Rejecting door plane "
                          << p_doorPlane->getId() << " for passage "
                          << passage->getId() << ": measured dimensions "
                          << measuredWidth << "x" << measuredHeight
                          << " m exceed limits " << maximumWidth << "x"
                          << maximumHeight << " m." << std::endl;

                continue;
            }

            /* Set centroid of the door plane */
            passage->setCentroid(p_doorPlane->getCentroid());

            /* Get the plane global equation */
            passage->setGlobalEquation(p_doorPlane->getGlobalEquation());

            /* Set width & height of the passage */
            passage->setWidth(measuredWidth);
            passage->setHeight(measuredHeight);
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
            const std::vector<vs_graphs::core::geometric::Plane *>
                supportingFaces = passage->getAssociateWalls();

            std::vector<Eigen::Vector4d> validFaceEquations;
            validFaceEquations.reserve(2);

            for (vs_graphs::core::geometric::Plane *p_candidateWall :
                 supportingFaces)
            {
                if (p_candidateWall == nullptr || p_candidateWall->isBad())
                {
                    continue;
                }

                Eigen::Vector4d candidateWallEquation =
                    p_candidateWall->getGlobalEquation().coeffs();

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

            const Eigen::Vector3d passageCentroid_World_m =
                passage->getCentroid();

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
                passagePlane.setGlobalEquation(g2o::Plane3D(midPlaneEquation));

                if (utils::utils::Utils::arePlanesPerpendicular(&passagePlane,
                                                                p_groundPlane))
                {
                    passage->setGlobalEquation(g2o::Plane3D(midPlaneEquation));
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

                    passage->setCentroid(anchoredCentroid_World_m);
                    passage->setGlobalEquation(g2o::Plane3D(referenceEquation));
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

                passage->setCentroid(anchoredCentroid_World_m);
                passage->setGlobalEquation(g2o::Plane3D(referenceEquation));
            }

            vs_graphs::core::geometric::Plane passagePlane;
            passagePlane.setGlobalEquation(passage->getGlobalEquation());
            if (!utils::utils::Utils::arePlanesPerpendicular(&passagePlane,
                                                             p_groundPlane))
            {
                // Project the passage normal onto the horizontal plane to
                // remove tilt
                const Eigen::Vector3d groundNormal =
                    p_groundPlane->getGlobalEquation()
                        .coeffs()
                        .head<3>()
                        .normalized();
                Eigen::Vector4d globalEq =
                    passage->getGlobalEquation().coeffs();
                Eigen::Vector3d passageNormal = globalEq.head<3>().normalized();

                Eigen::Vector3d correctedNormal =
                    (passageNormal -
                     passageNormal.dot(groundNormal) * groundNormal)
                        .normalized();
                if (correctedNormal.norm() < 1e-6)
                    continue;
                correctedNormal.normalize();

                // Recompute d so the plane still passes through the centroid
                const Eigen::Vector3d centroid =
                    passage->getCentroid().cast<double>();
                const double d = -correctedNormal.dot(centroid);

                Eigen::Vector4d correctedCoeffs;
                correctedCoeffs.head<3>() = correctedNormal;
                correctedCoeffs(3)        = d;

                passage->setGlobalEquation(g2o::Plane3D(correctedCoeffs));
            }
        }
    }
}

} // namespace core
} // namespace vs_graphs

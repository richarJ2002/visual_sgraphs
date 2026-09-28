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
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNSS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

#include "GeoSemHelpers.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace vs_graphs
{
namespace core
{

GeoSemHelpersStatus GeoSemHelpers::createMapPassage(
    vs_graphs::core::Atlas            *p_atlas_inout,
    vs_graphs::core::geometric::Plane *p_doorPlane_in,
    vs_graphs::core::geometric::Plane *p_wallPlane_in,
    bool                               isOpenPassage_in,
    Eigen::Vector3d                    passageCentroid_World_m_in)
{
    /* ---------------------------------------------------------------------- *
     * VALIDATE REQUIRED INPUTS
     * ---------------------------------------------------------------------- */

    if (p_atlas_inout == nullptr)
    {
        std::cerr << "[GeoSemHelper] Cannot create passage: Atlas is null."
                  << std::endl;
        return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
    }

    /*!
     * Every passage must be associated with a wall.
     *
     * Closed door:
     *     p_doorPlane_in != nullptr
     *     p_wallPlane_in != nullptr
     *
     * Open passage:
     *     p_doorPlane_in == nullptr
     *     p_wallPlane_in != nullptr
     */
    if (p_wallPlane_in == nullptr)
    {
        std::cerr << "[GeoSemHelper] Cannot create passage: wall plane is null"
                  << std::endl;
        return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
    }

    if (p_wallPlane_in->isBad())
    {
        std::cerr << "[GeoSemHelper] Cannot create passage: wall plane"
                  << p_wallPlane_in->getId() << " is bad." << std::endl;
        return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
    }

    /*!
     * Passage creation requires the supporting wall to have sufficient
     * observations. This prevents spurious passages on isolated wall segments
     * that have no evidence.
     */
    bool                 wallHasConfirmedRoom = false;
    types::SystemParams *p_params             = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        // getParams cannot fail; continue as before.
    }
    const size_t minimumObservation =
        p_params->roomSeg.minimumWallObservationCount;
    if (p_wallPlane_in->getObservationCount() >= minimumObservation)
    {
        wallHasConfirmedRoom = true;
    }

    if (!wallHasConfirmedRoom)
    {
        std::cerr << "[GeoSemHelper] Cannot create passage: wall plane "
                  << p_wallPlane_in->getId()
                  << " has insufficient observations ("
                  << p_wallPlane_in->getObservationCount() << " < "
                  << minimumObservation << ")." << std::endl;
        return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
    }

    /* Extract all passages */
    const std::vector<vs_graphs::core::semantic::Passage *> allPassages =
        p_atlas_inout->getAllPassages();

    /* ---------------------------------------------------------------------- *
     * DETERMINE THE PASSAGE GEOMETRY
     * ---------------------------------------------------------------------- */

    /* Extract the max door height and width */
    types::SystemParams *p_params2 = nullptr;
    if (types::SystemParams::getParams(p_params2) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        // getParams cannot fail; continue as before.
    }
    double               width     = p_params2->semSeg.maxDoorWidth;
    types::SystemParams *p_params3 = nullptr;
    if (types::SystemParams::getParams(p_params3) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        // getParams cannot fail; continue as before.
    }
    double height = p_params3->semSeg.maxDoorHeight;

    /* Initialize variables to define the passage */
    Eigen::Vector3d centroid;
    g2o::Plane3D    passageEquation;

    if (p_doorPlane_in != nullptr)
    {
        /* Confirm the door plane is not bad */
        if (p_doorPlane_in->isBad())
        {
            std::cerr << "[GeoSemHelper] Cannot create passage: door plane "
                      << p_doorPlane_in->getId() << " is bad." << std::endl;
            return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
        }

        /* Extract centroid and plane equation */
        centroid        = p_doorPlane_in->getCentroid();
        passageEquation = p_doorPlane_in->getGlobalEquation();

        /* Extract point cloud of door */
        const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_doorCloud =
            p_doorPlane_in->getGeometrySnapshot().supportCloud;

        /* Use measured door dimensions when a valid point cloud if available */
        if (p_doorCloud != nullptr && !p_doorCloud->empty())
        {
            /* Compute the dimensions of the door */
            std::pair<double, double> measuredDimensions{};
            if (utils::utils::Utils::computePlaneWidthHeight(
                    p_doorCloud,
                    measuredDimensions) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                // computePlaneWidthHeight cannot fail; continue as before.
            }

            /* Extract the dimensions of the door from the tuple */
            const double measuredWidth  = measuredDimensions.first;
            const double measuredHeight = measuredDimensions.second;

            /* Clip the width dimension of the door */
            if (std::isfinite(measuredWidth) && measuredWidth > 0.0)
            {
                types::SystemParams *p_params4 = nullptr;
                if (types::SystemParams::getParams(p_params4) !=
                    types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
                {
                    // getParams cannot fail; continue as before.
                }
                width = std::min(
                    measuredWidth,
                    static_cast<double>(p_params4->semSeg.maxDoorWidth));
            }

            /* Clip the height dimension of the door */
            if (std::isfinite(measuredHeight) && measuredHeight > 0.0)
            {
                types::SystemParams *p_params5 = nullptr;
                if (types::SystemParams::getParams(p_params5) !=
                    types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
                {
                    // getParams cannot fail; continue as before.
                }
                height = std::min(
                    measuredHeight,
                    static_cast<double>(p_params5->semSeg.maxDoorHeight));
            }
        }
    }
    else
    {
        /*!
         * No door exists. This is an open passage detected from the camera
         * trajectory crossing a wall plane.
         */
        centroid = passageCentroid_World_m_in;

        /* Extract the plane coefficients of the wall */
        Eigen::Vector4d wallEquation =
            p_wallPlane_in->getGlobalEquation().coeffs();

        /* Extract the magnitude of the norm from the coefficients */
        const double normalNorm = wallEquation.head<3>().norm();

        if (!std::isfinite(normalNorm) || normalNorm < 1e-8)
        {
            std::cerr << "[GeoSemHelper] Cannot create open passage: wall "
                      << p_wallPlane_in->getId()
                      << " has an invalid plane equation." << std::endl;
            return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
        }

        /* Normalize the norm vector */
        const Eigen::Vector3d normalizedNormal =
            wallEquation.head<3>() / normalNorm;

        /* Create parrallel plane, passing through detected wall centroid */
        Eigen::Vector4d correctionEquation;
        correctionEquation.head<3>() = normalizedNormal;
        correctionEquation(3) = -normalizedNormal.dot(centroid.cast<double>());

        /* Create an equation for the passage plane */
        passageEquation = g2o::Plane3D(correctionEquation);
    }

    /* Confirm the centroid of the plane is valid */
    if (!centroid.allFinite())
    {
        std::cerr << "[GeoSemHelper] Cannot create passage: invalid centroid."
                  << std::endl;
        return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
    }

    /* ---------------------------------------------------------------------- *
     * CHECK FOR AN EXISTING DUPLICATE BEFORE ALLOCATING ANYTHING
     * ---------------------------------------------------------------------- */

    /* Extract the duplicate distacne threshold */
    types::SystemParams *p_params6 = nullptr;
    if (types::SystemParams::getParams(p_params6) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        // getParams cannot fail; continue as before.
    }
    const double duplicateDistanceThreshold =
        p_params6->semSeg.passageCentroidDistanceThresh;

    /* Extract parameters from passage equation */
    Eigen::Vector4d candidateEquation = passageEquation.coeffs();

    /* Extract the normal norm from the candidate equation */
    const double candidateNormalNorm = candidateEquation.head<3>().norm();

    /* Confirm norm is valid */
    if (candidateNormalNorm < 1e-8)
    {
        std::cerr << "[GeoSemHelper] Cannot create pasasge: invalid candidate "
                     "plane equation"
                  << std::endl;
        return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
    }

    /* Extract the unit norm of the plane */
    const Eigen::Vector3d candidateNormal =
        candidateEquation.head<3>() / candidateNormalNorm;

    /* Iterate through existing passages and see if any passage matches */
    for (vs_graphs::core::semantic::Passage *p_existingPassage : allPassages)
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
            // getCentroid cannot fail; continue as before.
        }
        Eigen::Vector3d centroidDistanceVector =
            (centroid - existingPassageCentroid);

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
            // getGlobalEquation cannot fail; continue as before.
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
         * @note        normalAlignment = cos(theta),
         *              normalAlignment = 1 @ theta = 0
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
            // getCentroid cannot fail; continue as before.
        }
        const double candidatePlaneResidual_m =
            std::abs(normalizedCandidateEquation.head<3>().dot(
                         existingPassageCentroid2) +
                     normalizedCandidateEquation(3));
        const double existingPlaneResidual_m =
            std::abs(normalizedExistingEquation.head<3>().dot(centroid) +
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
            // getAssociateWalls cannot fail; continue as before.
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
                // setPassable cannot fail; continue as before.
            }
            if (p_existingPassage->setCentroid(centroid) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // setCentroid cannot fail; continue as before.
            }

            if (isKnownSupportingFace)
            {
                if (p_existingPassage->setGlobalEquation(passageEquation) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // setGlobalEquation cannot fail; continue as before.
                }
            }
        }

        if (p_existingPassage->addAssociateWall(p_wallPlane_in) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            // addAssociateWall cannot fail; continue as before.
        }

        vs_graphs::core::geometric::Plane *p_existingPassageAssociateDoor =
            nullptr;
        if ((p_doorPlane_in != nullptr) &&
            p_existingPassage->getAssociateDoor(
                p_existingPassageAssociateDoor) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            // getAssociateDoor cannot fail; continue as before.
        }
        if (p_doorPlane_in != nullptr &&
            p_existingPassageAssociateDoor == nullptr)
        {
            if (p_existingPassage->setAssociateDoor(p_doorPlane_in) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // setAssociateDoor cannot fail; continue as before.
            }
        }

        int existingPassageId{};
        if (p_existingPassage->getId(existingPassageId) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            // getId cannot fail; continue as before.
        }
        bool existingPassageIsPassable{};
        if (p_existingPassage->isPassable(existingPassageIsPassable) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            // isPassable cannot fail; continue as before.
        }
        std::cout << "[GeoSemHelper] Updated existing semantic::Passage#"
                  << existingPassageId
                  << ": centroid distance=" << centroidDistance
                  << " m, normal alignment=" << normalAlignment << ", state="
                  << (existingPassageIsPassable ? "open" : "blocked") << "."
                  << std::endl;

        return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
    }

    /* ---------------------------------------------------------------------- *
     * NO PASSAGE MATCH FOUND. GENERATING NEW PASSAGE
     * ---------------------------------------------------------------------- */

    /* Passage identities belong to the mission, not to an active SLAM map. */
    const int passageId = p_atlas_inout->reservePassageIdentity();

    /* Initialize passage object */
    vs_graphs::core::semantic::Passage *p_newMapPassage =
        new vs_graphs::core::semantic::Passage();

    /* Fill passage object */
    if (p_newMapPassage->setId(passageId) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        // setId cannot fail; continue as before.
    }
    if (p_newMapPassage->setMap(p_atlas_inout->getCurrentMap()) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        // setMap cannot fail; continue as before.
    }

    if (p_newMapPassage->setCentroid(centroid) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        // setCentroid cannot fail; continue as before.
    }
    if (p_newMapPassage->setGlobalEquation(passageEquation) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        // setGlobalEquation cannot fail; continue as before.
    }

    if (p_newMapPassage->setWidth(width) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        // setWidth cannot fail; continue as before.
    }
    if (p_newMapPassage->setHeight(height) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        // setHeight cannot fail; continue as before.
    }
    if (p_newMapPassage->setPassable(isOpenPassage_in) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        // setPassable cannot fail; continue as before.
    }

    if (p_newMapPassage->addAssociateWall(p_wallPlane_in) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        // addAssociateWall cannot fail; continue as before.
    }

    /*!
     * Both a detected closed door and a trajectory-detected open
     * passage represent a doorway.
     */
    if (p_newMapPassage->setPassageType(
            vs_graphs::core::semantic::Passage::PassageVariant::DOORWAY) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        // setPassageType cannot fail; continue as before.
    }

    if (p_doorPlane_in != nullptr)
    {
        if (p_newMapPassage->setAssociateDoor(p_doorPlane_in) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            // setAssociateDoor cannot fail; continue as before.
        }
    }

    /* -------------------------------------------------------------- *
     * Insert into the Atlas
     * -------------------------------------------------------------- */

    std::ostringstream informationStream;

    informationStream << (isOpenPassage_in ? "open" : "blocked") << ", "
                      << std::fixed << std::setprecision(2) << width << "x"
                      << height << "m";

    std::cout << "[GeoSemHelper] Creating semantic::Passage#" << passageId
              << " associated with wall " << p_wallPlane_in->getId();

    if (p_doorPlane_in != nullptr)
    {
        std::cout << " and door plane " << p_doorPlane_in->getId();
    }

    std::cout << " (" << informationStream.str()
              << "), centroid=" << centroid.transpose() << "." << std::endl;

    p_atlas_inout->addMapPassage(p_newMapPassage);

    std::cout << "[GeoSemHelper] Atlas now contains "
              << p_atlas_inout->getAllPassages().size() << " passages."
              << std::endl;

    return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

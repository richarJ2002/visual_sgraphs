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
 * @file            mergeFromDuplicate.cc
 *
 * @brief           Implements Passage::mergeFromDuplicate(), declared in
 *                  Semantic/Passage.h.
 */

#include "Semantic/Passage.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

PassageStatus Passage::mergeFromDuplicate(Passage *p_duplicate_inout,
                                          bool    &wasGeometryReplaced_out)
{
    int duplicate_inoutId{};
    if (!(p_duplicate_inout == nullptr || p_duplicate_inout == this) &&
        p_duplicate_inout->getId(duplicate_inoutId) !=
            PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    int duplicateId{};
    if (!(p_duplicate_inout == nullptr || p_duplicate_inout == this) &&
        getId(duplicateId) != PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_duplicate_inout == nullptr || p_duplicate_inout == this ||
        duplicate_inoutId != duplicateId)
    {
        return PassageStatus::PASSAGE_STATUS_INVALID_ARGUMENT;
    }

    bool canonicalIsRecoveryProxy = false;
    {
        std::lock_guard<std::mutex> typeLock(typeMutex);
        canonicalIsRecoveryProxy = isMarkedRecoveryProxy;
    }

    bool           duplicateIsRecoveryProxy = false;
    bool           duplicateIsPassable      = false;
    PassageVariant duplicatePassageType     = PassageVariant::UNDEFINED;
    std::size_t    duplicateKnownToFarCount = 0U;
    std::size_t    duplicateFarToKnownCount = 0U;
    std::size_t    duplicateUnknownCount    = 0U;
    {
        std::lock_guard<std::mutex> duplicateTypeLock(
            p_duplicate_inout->typeMutex);
        duplicateIsRecoveryProxy = p_duplicate_inout->isMarkedRecoveryProxy;
        duplicateIsPassable      = p_duplicate_inout->isMarkedPassable;
        duplicatePassageType     = p_duplicate_inout->passageType;
        duplicateKnownToFarCount = p_duplicate_inout->traversalKnownToFarCount;
        duplicateFarToKnownCount = p_duplicate_inout->traversalFarToKnownCount;
        duplicateUnknownCount    = p_duplicate_inout->traversalUnknownCount;
    }

    Eigen::Vector3d                 duplicateCentroid = Eigen::Vector3d::Zero();
    g2o::Plane3D                    duplicateEquation;
    double                          duplicateWidth_m  = 0.0;
    double                          duplicateHeight_m = 0.0;
    geometric::Plane               *p_duplicateDoor   = nullptr;
    std::vector<geometric::Plane *> duplicateWalls;
    Room                           *p_duplicateProspectiveRoom = nullptr;
    KnownSideProvenance             duplicateKnownSide;
    {
        std::lock_guard<std::mutex> duplicateGeometryLock(
            p_duplicate_inout->geometryMutex);
        duplicateCentroid          = p_duplicate_inout->centroid;
        duplicateEquation          = p_duplicate_inout->globalEquation;
        duplicateWidth_m           = p_duplicate_inout->width;
        duplicateHeight_m          = p_duplicate_inout->height;
        p_duplicateDoor            = p_duplicate_inout->p_associatedDoor;
        duplicateWalls             = p_duplicate_inout->associateWalls;
        p_duplicateProspectiveRoom = p_duplicate_inout->p_prospectiveRoom;
        duplicateKnownSide         = p_duplicate_inout->knownSideProvenance;
    }

    const Eigen::Vector4d duplicateEquationCoefficients =
        duplicateEquation.coeffs();
    const double duplicateNormalNorm =
        duplicateEquationCoefficients.head<3>().norm();
    const bool duplicateHasValidObservedGeometry =
        !duplicateIsRecoveryProxy && duplicateCentroid.allFinite() &&
        duplicateEquationCoefficients.allFinite() &&
        std::isfinite(duplicateNormalNorm) && duplicateNormalNorm > 1e-8 &&
        std::isfinite(duplicateWidth_m) && duplicateWidth_m > 0.0 &&
        std::isfinite(duplicateHeight_m) && duplicateHeight_m > 0.0;

    bool replacedGeometry = false;
    {
        std::lock_guard<std::mutex> geometryLock(geometryMutex);
        const Eigen::Vector4d       canonicalEquationCoefficients =
            globalEquation.coeffs();
        const double canonicalNormalNorm =
            canonicalEquationCoefficients.head<3>().norm();
        const bool canonicalHasValidGeometry =
            centroid.allFinite() && canonicalEquationCoefficients.allFinite() &&
            std::isfinite(canonicalNormalNorm) && canonicalNormalNorm > 1e-8 &&
            std::isfinite(width) && width > 0.0 && std::isfinite(height) &&
            height > 0.0;

        replacedGeometry =
            duplicateHasValidObservedGeometry &&
            (canonicalIsRecoveryProxy || !canonicalHasValidGeometry);
        if (replacedGeometry)
        {
            centroid         = duplicateCentroid;
            globalEquation   = duplicateEquation;
            width            = duplicateWidth_m;
            height           = duplicateHeight_m;
            p_associatedDoor = p_duplicateDoor;
        }
        else if (p_associatedDoor == nullptr)
        {
            p_associatedDoor = p_duplicateDoor;
        }

        for (geometric::Plane *p_duplicateWall : duplicateWalls)
        {
            if (p_duplicateWall != nullptr &&
                std::find(associateWalls.begin(),
                          associateWalls.end(),
                          p_duplicateWall) == associateWalls.end())
            {
                associateWalls.push_back(p_duplicateWall);
            }
        }

        if (p_prospectiveRoom == nullptr)
        {
            p_prospectiveRoom = p_duplicateProspectiveRoom;
        }
        if (knownSideProvenance.p_room == nullptr)
        {
            knownSideProvenance.p_room = duplicateKnownSide.p_room;
        }
        bool knownSideHasDirection{};
        if (knownSideProvenance.hasDirection(knownSideHasDirection) !=
            KnownSideProvenanceStatus::KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasDirection returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        bool duplicateHasDirection{};
        if (duplicateKnownSide.hasDirection(duplicateHasDirection) !=
            KnownSideProvenanceStatus::KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasDirection returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (replacedGeometry)
        {
            knownSideProvenance.direction_World =
                duplicateKnownSide.direction_World;
        }
        else if (!knownSideHasDirection && duplicateHasDirection)
        {
            knownSideProvenance.direction_World =
                duplicateKnownSide.direction_World;
        }
    }

    {
        std::lock_guard<std::mutex> typeLock(typeMutex);
        traversalKnownToFarCount =
            std::max(traversalKnownToFarCount, duplicateKnownToFarCount);
        traversalFarToKnownCount =
            std::max(traversalFarToKnownCount, duplicateFarToKnownCount);
        traversalUnknownCount =
            std::max(traversalUnknownCount, duplicateUnknownCount);
        if (replacedGeometry)
        {
            isMarkedPassable      = duplicateIsPassable;
            passageType           = duplicatePassageType;
            isMarkedRecoveryProxy = false;
        }
        else if (passageType == PassageVariant::UNDEFINED)
        {
            passageType = duplicatePassageType;
        }
    }

    wasGeometryReplaced_out = replacedGeometry;
    return PassageStatus::PASSAGE_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs

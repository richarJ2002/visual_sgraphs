/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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
 * @file            getGeometryMetadataSnapshot.cc
 *
 * @brief           Implements Plane::getGeometryMetadataSnapshot(), declared
 *                  in Geometric/Plane.h.
 */

#include "Geometric/Plane.h"

namespace vs_graphs
{
namespace core
{
namespace geometric
{

PlaneStatus Plane::getGeometryMetadataSnapshot(
    PlaneGeometryMetadataSnapshot &getGeometryMetadataSnapshot_out) const
{
    std::scoped_lock              lock(positionMutex, featuresMutex);
    PlaneGeometryMetadataSnapshot snapshot;
    snapshot.planeEquation_world       = globalEquation.coeffs();
    snapshot.planeCentroid_world_m     = centroid;
    snapshot.minPlaneU_m               = minPlaneU;
    snapshot.maxPlaneU_m               = maxPlaneU;
    snapshot.minPlaneV_m               = minPlaneV;
    snapshot.maxPlaneV_m               = maxPlaneV;
    snapshot.finiteSupportCount        = lastSuccessfulRefitFinitePointCount;
    snapshot.observationCount          = observationCount;
    snapshot.cloudGeneration           = cloudGeneration;
    snapshot.successfulRefitGeneration = successfulRefitGeneration;
    getGeometryMetadataSnapshot_out    = snapshot;
    return PlaneStatus::PLANE_STATUS_SUCCESS;
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs

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

PlaneGeometryMetadataSnapshot Plane::getGeometryMetadataSnapshot(void) const
{
    std::scoped_lock              lock(mMutexPos, mMutexFeatures);
    PlaneGeometryMetadataSnapshot snapshot;
    snapshot.equation_World            = globalEquation.coeffs();
    snapshot.centroid_World_m          = centroid;
    snapshot.minPlaneU_m               = minPlaneU;
    snapshot.maxPlaneU_m               = maxPlaneU;
    snapshot.minPlaneV_m               = minPlaneV;
    snapshot.maxPlaneV_m               = maxPlaneV;
    snapshot.finiteSupportCount        = lastSuccessfulRefitFinitePointCount;
    snapshot.observationCount          = observationCount;
    snapshot.cloudGeneration           = cloudGeneration;
    snapshot.successfulRefitGeneration = successfulRefitGeneration;
    return snapshot;
}

} // namespace geometric
} // namespace core
} // namespace vs_graphs

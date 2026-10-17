/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
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
 * @file            isValueLessForCollisionTiebreak.cc
 *
 * @brief           Implements all isValueLessForCollisionTiebreak() overloads
 *                  (floor, passage, room, wall), declared in
 *                  private_functions/private_functions.h.
 */

#include "semantic/semantic_graph_snapshot/private_functions/private_functions.h"
#include "semantic/floor/floor.h"
#include "semantic/room/room.h"

#include <algorithm>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

bool isValueLessForCollisionTiebreak(const FloorRecord &lhs_in,
                                     const FloorRecord &rhs_in)
{
    if (lhs_in.declaredMapId.has_value() != rhs_in.declaredMapId.has_value())
    {
        return lhs_in.declaredMapId.has_value();
    }
    if (lhs_in.declaredMapId.has_value() &&
        *lhs_in.declaredMapId != *rhs_in.declaredMapId)
    {
        return *lhs_in.declaredMapId < *rhs_in.declaredMapId;
    }
    if (isVector3dLess(lhs_in.floorCentroid_world_m,
                       rhs_in.floorCentroid_world_m) ||
        isVector3dLess(rhs_in.floorCentroid_world_m,
                       lhs_in.floorCentroid_world_m))
    {
        return isVector3dLess(lhs_in.floorCentroid_world_m,
                              rhs_in.floorCentroid_world_m);
    }
    if (lhs_in.planeIdentity.has_value() != rhs_in.planeIdentity.has_value())
    {
        return lhs_in.planeIdentity.has_value();
    }
    if (lhs_in.planeIdentity.has_value())
    {
        const Floor::PlaneIdentity &lhsIdentity = *lhs_in.planeIdentity;
        const Floor::PlaneIdentity &rhsIdentity = *rhs_in.planeIdentity;
        if (isVector4dLess(lhsIdentity.planeEquation_world,
                           rhsIdentity.planeEquation_world) ||
            isVector4dLess(rhsIdentity.planeEquation_world,
                           lhsIdentity.planeEquation_world))
        {
            return isVector4dLess(lhsIdentity.planeEquation_world,
                                  rhsIdentity.planeEquation_world);
        }
        if (lhsIdentity.finiteSupportCount != rhsIdentity.finiteSupportCount)
        {
            return lhsIdentity.finiteSupportCount <
                   rhsIdentity.finiteSupportCount;
        }
        if (lhsIdentity.observationCount != rhsIdentity.observationCount)
        {
            return lhsIdentity.observationCount < rhsIdentity.observationCount;
        }
    }
    return std::lexicographical_compare(lhs_in.roomRefs.begin(),
                                        lhs_in.roomRefs.end(),
                                        rhs_in.roomRefs.begin(),
                                        rhs_in.roomRefs.end(),
                                        &isEntityRefLess);
}

bool isValueLessForCollisionTiebreak(const PassageRecord &lhs_in,
                                     const PassageRecord &rhs_in)
{
    if (lhs_in.isLive != rhs_in.isLive)
    {
        return static_cast<int>(lhs_in.isLive) <
               static_cast<int>(rhs_in.isLive);
    }
    if (lhs_in.declaredMapId.has_value() != rhs_in.declaredMapId.has_value())
    {
        return lhs_in.declaredMapId.has_value();
    }
    if (lhs_in.declaredMapId.has_value() &&
        *lhs_in.declaredMapId != *rhs_in.declaredMapId)
    {
        return *lhs_in.declaredMapId < *rhs_in.declaredMapId;
    }
    if (lhs_in.passageType != rhs_in.passageType)
    {
        return lhs_in.passageType < rhs_in.passageType;
    }
    if (isVector4dLess(lhs_in.planeEquation_world,
                       rhs_in.planeEquation_world) ||
        isVector4dLess(rhs_in.planeEquation_world, lhs_in.planeEquation_world))
    {
        return isVector4dLess(lhs_in.planeEquation_world,
                              rhs_in.planeEquation_world);
    }
    if (isVector3dLess(lhs_in.passageCentroid_world_m,
                       rhs_in.passageCentroid_world_m) ||
        isVector3dLess(rhs_in.passageCentroid_world_m,
                       lhs_in.passageCentroid_world_m))
    {
        return isVector3dLess(lhs_in.passageCentroid_world_m,
                              rhs_in.passageCentroid_world_m);
    }
    if (isDoubleLess(lhs_in.width_m, rhs_in.width_m) ||
        isDoubleLess(rhs_in.width_m, lhs_in.width_m))
    {
        return isDoubleLess(lhs_in.width_m, rhs_in.width_m);
    }
    if (isDoubleLess(lhs_in.height_m, rhs_in.height_m) ||
        isDoubleLess(rhs_in.height_m, lhs_in.height_m))
    {
        return isDoubleLess(lhs_in.height_m, rhs_in.height_m);
    }
    if (lhs_in.isPassable != rhs_in.isPassable)
    {
        return static_cast<int>(lhs_in.isPassable) <
               static_cast<int>(rhs_in.isPassable);
    }
    if (std::lexicographical_compare(lhs_in.associateWallRefs.begin(),
                                     lhs_in.associateWallRefs.end(),
                                     rhs_in.associateWallRefs.begin(),
                                     rhs_in.associateWallRefs.end(),
                                     &isRawPlaneRefLess) ||
        std::lexicographical_compare(rhs_in.associateWallRefs.begin(),
                                     rhs_in.associateWallRefs.end(),
                                     lhs_in.associateWallRefs.begin(),
                                     lhs_in.associateWallRefs.end(),
                                     &isRawPlaneRefLess))
    {
        return std::lexicographical_compare(lhs_in.associateWallRefs.begin(),
                                            lhs_in.associateWallRefs.end(),
                                            rhs_in.associateWallRefs.begin(),
                                            rhs_in.associateWallRefs.end(),
                                            &isRawPlaneRefLess);
    }
    if (isRawPlaneRefLess(lhs_in.associateDoorRef, rhs_in.associateDoorRef) ||
        isRawPlaneRefLess(rhs_in.associateDoorRef, lhs_in.associateDoorRef))
    {
        return isRawPlaneRefLess(lhs_in.associateDoorRef,
                                 rhs_in.associateDoorRef);
    }
    if (isEntityRefLess(lhs_in.knownSideRoomRef, rhs_in.knownSideRoomRef) ||
        isEntityRefLess(rhs_in.knownSideRoomRef, lhs_in.knownSideRoomRef))
    {
        return isEntityRefLess(lhs_in.knownSideRoomRef,
                               rhs_in.knownSideRoomRef);
    }
    if (lhs_in.knownSideDirection_world.has_value() !=
        rhs_in.knownSideDirection_world.has_value())
    {
        return lhs_in.knownSideDirection_world.has_value();
    }
    if (lhs_in.knownSideDirection_world.has_value() &&
        (isVector3dLess(*lhs_in.knownSideDirection_world,
                        *rhs_in.knownSideDirection_world) ||
         isVector3dLess(*rhs_in.knownSideDirection_world,
                        *lhs_in.knownSideDirection_world)))
    {
        return isVector3dLess(*lhs_in.knownSideDirection_world,
                              *rhs_in.knownSideDirection_world);
    }
    if (isEntityRefLess(lhs_in.prospectiveRoomRef, rhs_in.prospectiveRoomRef) ||
        isEntityRefLess(rhs_in.prospectiveRoomRef, lhs_in.prospectiveRoomRef))
    {
        return isEntityRefLess(lhs_in.prospectiveRoomRef,
                               rhs_in.prospectiveRoomRef);
    }
    if (lhs_in.traversalKnownToFarCount != rhs_in.traversalKnownToFarCount)
    {
        return lhs_in.traversalKnownToFarCount <
               rhs_in.traversalKnownToFarCount;
    }
    if (lhs_in.traversalFarToKnownCount != rhs_in.traversalFarToKnownCount)
    {
        return lhs_in.traversalFarToKnownCount <
               rhs_in.traversalFarToKnownCount;
    }
    if (lhs_in.traversalUnknownCount != rhs_in.traversalUnknownCount)
    {
        return lhs_in.traversalUnknownCount < rhs_in.traversalUnknownCount;
    }
    return lhs_in.endpointSlotReason < rhs_in.endpointSlotReason;
}

bool isValueLessForCollisionTiebreak(const RoomRecord &lhs_in,
                                     const RoomRecord &rhs_in)
{
    if (lhs_in.isLive != rhs_in.isLive)
    {
        return static_cast<int>(lhs_in.isLive) <
               static_cast<int>(rhs_in.isLive);
    }
    if (lhs_in.isDetectedMember != rhs_in.isDetectedMember)
    {
        return static_cast<int>(lhs_in.isDetectedMember) <
               static_cast<int>(rhs_in.isDetectedMember);
    }
    if (lhs_in.isMarkerBasedMember != rhs_in.isMarkerBasedMember)
    {
        return static_cast<int>(lhs_in.isMarkerBasedMember) <
               static_cast<int>(rhs_in.isMarkerBasedMember);
    }
    if (lhs_in.declaredMapId.has_value() != rhs_in.declaredMapId.has_value())
    {
        return lhs_in.declaredMapId.has_value();
    }
    if (lhs_in.declaredMapId.has_value() &&
        *lhs_in.declaredMapId != *rhs_in.declaredMapId)
    {
        return *lhs_in.declaredMapId < *rhs_in.declaredMapId;
    }
    if (lhs_in.variant != rhs_in.variant)
    {
        return lhs_in.variant < rhs_in.variant;
    }
    if (isVector3dLess(lhs_in.roomCentroid_world_m,
                       rhs_in.roomCentroid_world_m) ||
        isVector3dLess(rhs_in.roomCentroid_world_m,
                       lhs_in.roomCentroid_world_m))
    {
        return isVector3dLess(lhs_in.roomCentroid_world_m,
                              rhs_in.roomCentroid_world_m);
    }
    if (lhs_in.boundaryStatus != rhs_in.boundaryStatus)
    {
        return lhs_in.boundaryStatus < rhs_in.boundaryStatus;
    }
    if (std::lexicographical_compare(lhs_in.boundaryCorners_world_m.begin(),
                                     lhs_in.boundaryCorners_world_m.end(),
                                     rhs_in.boundaryCorners_world_m.begin(),
                                     rhs_in.boundaryCorners_world_m.end(),
                                     &isVector3dLess) ||
        std::lexicographical_compare(rhs_in.boundaryCorners_world_m.begin(),
                                     rhs_in.boundaryCorners_world_m.end(),
                                     lhs_in.boundaryCorners_world_m.begin(),
                                     lhs_in.boundaryCorners_world_m.end(),
                                     &isVector3dLess))
    {
        return std::lexicographical_compare(
            lhs_in.boundaryCorners_world_m.begin(),
            lhs_in.boundaryCorners_world_m.end(),
            rhs_in.boundaryCorners_world_m.begin(),
            rhs_in.boundaryCorners_world_m.end(),
            &isVector3dLess);
    }
    const auto isObservationGapLess = [](const Room::ObservationGap &lhsGap_in,
                                         const Room::ObservationGap &rhsGap_in)
    {
        if (isDoubleLess(lhsGap_in.startAngle_rad, rhsGap_in.startAngle_rad) ||
            isDoubleLess(rhsGap_in.startAngle_rad, lhsGap_in.startAngle_rad))
        {
            return isDoubleLess(lhsGap_in.startAngle_rad,
                                rhsGap_in.startAngle_rad);
        }
        return isDoubleLess(lhsGap_in.spanAngle_rad, rhsGap_in.spanAngle_rad);
    };
    if (std::lexicographical_compare(lhs_in.observationGaps.begin(),
                                     lhs_in.observationGaps.end(),
                                     rhs_in.observationGaps.begin(),
                                     rhs_in.observationGaps.end(),
                                     isObservationGapLess) ||
        std::lexicographical_compare(rhs_in.observationGaps.begin(),
                                     rhs_in.observationGaps.end(),
                                     lhs_in.observationGaps.begin(),
                                     lhs_in.observationGaps.end(),
                                     isObservationGapLess))
    {
        return std::lexicographical_compare(lhs_in.observationGaps.begin(),
                                            lhs_in.observationGaps.end(),
                                            rhs_in.observationGaps.begin(),
                                            rhs_in.observationGaps.end(),
                                            isObservationGapLess);
    }
    if (std::lexicographical_compare(lhs_in.wallRefs.begin(),
                                     lhs_in.wallRefs.end(),
                                     rhs_in.wallRefs.begin(),
                                     rhs_in.wallRefs.end(),
                                     &isRawPlaneRefLess) ||
        std::lexicographical_compare(rhs_in.wallRefs.begin(),
                                     rhs_in.wallRefs.end(),
                                     lhs_in.wallRefs.begin(),
                                     lhs_in.wallRefs.end(),
                                     &isRawPlaneRefLess))
    {
        return std::lexicographical_compare(lhs_in.wallRefs.begin(),
                                            lhs_in.wallRefs.end(),
                                            rhs_in.wallRefs.begin(),
                                            rhs_in.wallRefs.end(),
                                            &isRawPlaneRefLess);
    }
    if (std::lexicographical_compare(lhs_in.passageRefs.begin(),
                                     lhs_in.passageRefs.end(),
                                     rhs_in.passageRefs.begin(),
                                     rhs_in.passageRefs.end(),
                                     &isEntityRefLess) ||
        std::lexicographical_compare(rhs_in.passageRefs.begin(),
                                     rhs_in.passageRefs.end(),
                                     lhs_in.passageRefs.begin(),
                                     lhs_in.passageRefs.end(),
                                     &isEntityRefLess))
    {
        return std::lexicographical_compare(lhs_in.passageRefs.begin(),
                                            lhs_in.passageRefs.end(),
                                            rhs_in.passageRefs.begin(),
                                            rhs_in.passageRefs.end(),
                                            &isEntityRefLess);
    }
    if (isEntityRefLess(lhs_in.floorRef, rhs_in.floorRef) ||
        isEntityRefLess(rhs_in.floorRef, lhs_in.floorRef))
    {
        return isEntityRefLess(lhs_in.floorRef, rhs_in.floorRef);
    }
    if (isRawPlaneRefLess(lhs_in.groundPlaneRef, rhs_in.groundPlaneRef) ||
        isRawPlaneRefLess(rhs_in.groundPlaneRef, lhs_in.groundPlaneRef))
    {
        return isRawPlaneRefLess(lhs_in.groundPlaneRef, rhs_in.groundPlaneRef);
    }
    return lhs_in.creationProvenanceReason < rhs_in.creationProvenanceReason;
}

bool isValueLessForCollisionTiebreak(const WallRecord &lhs_in,
                                     const WallRecord &rhs_in)
{
    if (lhs_in.isLive != rhs_in.isLive)
    {
        return static_cast<int>(lhs_in.isLive) <
               static_cast<int>(rhs_in.isLive);
    }
    if (lhs_in.declaredMapId.has_value() != rhs_in.declaredMapId.has_value())
    {
        return lhs_in.declaredMapId.has_value();
    }
    if (lhs_in.declaredMapId.has_value() &&
        *lhs_in.declaredMapId != *rhs_in.declaredMapId)
    {
        return *lhs_in.declaredMapId < *rhs_in.declaredMapId;
    }
    if (lhs_in.planeType != rhs_in.planeType)
    {
        return lhs_in.planeType < rhs_in.planeType;
    }
    if (isVector4dLess(lhs_in.planeEquation_world,
                       rhs_in.planeEquation_world) ||
        isVector4dLess(rhs_in.planeEquation_world, lhs_in.planeEquation_world))
    {
        return isVector4dLess(lhs_in.planeEquation_world,
                              rhs_in.planeEquation_world);
    }
    if (isVector3dLess(lhs_in.planeCentroid_world_m,
                       rhs_in.planeCentroid_world_m) ||
        isVector3dLess(rhs_in.planeCentroid_world_m,
                       lhs_in.planeCentroid_world_m))
    {
        return isVector3dLess(lhs_in.planeCentroid_world_m,
                              rhs_in.planeCentroid_world_m);
    }
    if (isDoubleLess(lhs_in.minPlaneU_m, rhs_in.minPlaneU_m) ||
        isDoubleLess(rhs_in.minPlaneU_m, lhs_in.minPlaneU_m))
    {
        return isDoubleLess(lhs_in.minPlaneU_m, rhs_in.minPlaneU_m);
    }
    if (isDoubleLess(lhs_in.maxPlaneU_m, rhs_in.maxPlaneU_m) ||
        isDoubleLess(rhs_in.maxPlaneU_m, lhs_in.maxPlaneU_m))
    {
        return isDoubleLess(lhs_in.maxPlaneU_m, rhs_in.maxPlaneU_m);
    }
    if (isDoubleLess(lhs_in.minPlaneV_m, rhs_in.minPlaneV_m) ||
        isDoubleLess(rhs_in.minPlaneV_m, lhs_in.minPlaneV_m))
    {
        return isDoubleLess(lhs_in.minPlaneV_m, rhs_in.minPlaneV_m);
    }
    if (isDoubleLess(lhs_in.maxPlaneV_m, rhs_in.maxPlaneV_m) ||
        isDoubleLess(rhs_in.maxPlaneV_m, lhs_in.maxPlaneV_m))
    {
        return isDoubleLess(lhs_in.maxPlaneV_m, rhs_in.maxPlaneV_m);
    }
    if (lhs_in.finiteSupportCount != rhs_in.finiteSupportCount)
    {
        return lhs_in.finiteSupportCount < rhs_in.finiteSupportCount;
    }
    if (lhs_in.observationCount != rhs_in.observationCount)
    {
        return lhs_in.observationCount < rhs_in.observationCount;
    }
    if (lhs_in.cloudGeneration != rhs_in.cloudGeneration)
    {
        return lhs_in.cloudGeneration < rhs_in.cloudGeneration;
    }
    if (lhs_in.successfulRefitGeneration != rhs_in.successfulRefitGeneration)
    {
        return lhs_in.successfulRefitGeneration <
               rhs_in.successfulRefitGeneration;
    }
    if (lhs_in.observationOrigin_world_m.has_value() !=
        rhs_in.observationOrigin_world_m.has_value())
    {
        return lhs_in.observationOrigin_world_m.has_value();
    }
    if (lhs_in.observationOrigin_world_m.has_value() &&
        (isVector3dLess(*lhs_in.observationOrigin_world_m,
                        *rhs_in.observationOrigin_world_m) ||
         isVector3dLess(*rhs_in.observationOrigin_world_m,
                        *lhs_in.observationOrigin_world_m)))
    {
        return isVector3dLess(*lhs_in.observationOrigin_world_m,
                              *rhs_in.observationOrigin_world_m);
    }
    if (lhs_in.observationSideConsensusReason !=
        rhs_in.observationSideConsensusReason)
    {
        return lhs_in.observationSideConsensusReason <
               rhs_in.observationSideConsensusReason;
    }
    if (isRawPlaneRefLess(lhs_in.twinRef, rhs_in.twinRef) ||
        isRawPlaneRefLess(rhs_in.twinRef, lhs_in.twinRef))
    {
        return isRawPlaneRefLess(lhs_in.twinRef, rhs_in.twinRef);
    }
    if (std::lexicographical_compare(lhs_in.ownerRoomRefs.begin(),
                                     lhs_in.ownerRoomRefs.end(),
                                     rhs_in.ownerRoomRefs.begin(),
                                     rhs_in.ownerRoomRefs.end(),
                                     &isEntityRefLess) ||
        std::lexicographical_compare(rhs_in.ownerRoomRefs.begin(),
                                     rhs_in.ownerRoomRefs.end(),
                                     lhs_in.ownerRoomRefs.begin(),
                                     lhs_in.ownerRoomRefs.end(),
                                     &isEntityRefLess))
    {
        return std::lexicographical_compare(lhs_in.ownerRoomRefs.begin(),
                                            lhs_in.ownerRoomRefs.end(),
                                            rhs_in.ownerRoomRefs.begin(),
                                            rhs_in.ownerRoomRefs.end(),
                                            &isEntityRefLess);
    }
    if (lhs_in.quarantineReason != rhs_in.quarantineReason)
    {
        return lhs_in.quarantineReason < rhs_in.quarantineReason;
    }
    return lhs_in.observationRayEvidenceReason <
           rhs_in.observationRayEvidenceReason;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

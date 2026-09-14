/**
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
 * @file            room.cc
 *
 * @brief           Implements the RoomRecord overload of
 *                  isValueLessForCollisionTiebreak(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticGraphSnapshot/private_functions.h"

#include <algorithm>

#include "Semantic/Room.h"

namespace ORB_SLAM3
{
namespace semantic
{

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
    if (isVector3dLess(lhs_in.centroid_World_m, rhs_in.centroid_World_m) ||
        isVector3dLess(rhs_in.centroid_World_m, lhs_in.centroid_World_m))
    {
        return isVector3dLess(lhs_in.centroid_World_m, rhs_in.centroid_World_m);
    }
    if (lhs_in.boundaryStatus != rhs_in.boundaryStatus)
    {
        return lhs_in.boundaryStatus < rhs_in.boundaryStatus;
    }
    if (std::lexicographical_compare(lhs_in.boundaryCorners_World_m.begin(),
                                     lhs_in.boundaryCorners_World_m.end(),
                                     rhs_in.boundaryCorners_World_m.begin(),
                                     rhs_in.boundaryCorners_World_m.end(),
                                     &isVector3dLess) ||
        std::lexicographical_compare(rhs_in.boundaryCorners_World_m.begin(),
                                     rhs_in.boundaryCorners_World_m.end(),
                                     lhs_in.boundaryCorners_World_m.begin(),
                                     lhs_in.boundaryCorners_World_m.end(),
                                     &isVector3dLess))
    {
        return std::lexicographical_compare(
            lhs_in.boundaryCorners_World_m.begin(),
            lhs_in.boundaryCorners_World_m.end(),
            rhs_in.boundaryCorners_World_m.begin(),
            rhs_in.boundaryCorners_World_m.end(),
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

} // namespace semantic
} // namespace ORB_SLAM3

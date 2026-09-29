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
 * @file            serializeWallRecord.cc
 *
 * @brief           Implements serializeWallRecord(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

#include <algorithm>
#include <rclcpp/logging.hpp>
#include <utility>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

nlohmann::json serializeWallRecord(const WallRecord &value_in,
                                   bool              includeGeometry_in)
{
    nlohmann::json json;
    json["key"]    = serializeEntityKey(value_in.key);
    json["isLive"] = value_in.isLive;
    if (value_in.declaredMapId.has_value())
    {
        json["declaredMapId"] = *value_in.declaredMapId;
    }
    json["planeType"] = static_cast<int>(value_in.planeType);
    json["observationSideConsensusReason"] =
        static_cast<unsigned int>(value_in.observationSideConsensusReason);
    std::string unavailableReasonName2{};
    if (unavailableReasonName(value_in.observationSideConsensusReason,
                              unavailableReasonName2) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: unavailableReasonName returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    json["observationSideConsensusReasonName"] = unavailableReasonName2;
    json["twinRef"] = serializeRawPlaneRef(value_in.twinRef);

    std::vector<EntityRef> ownerRoomRefs = value_in.ownerRoomRefs;
    std::sort(ownerRoomRefs.begin(), ownerRoomRefs.end(), &isEntityRefLess);
    nlohmann::json ownerRoomRefsJson = nlohmann::json::array();
    for (const EntityRef &ownerRoomReference : ownerRoomRefs)
    {
        ownerRoomRefsJson.push_back(serializeEntityRef(ownerRoomReference));
    }
    json["ownerRoomRefs"] = std::move(ownerRoomRefsJson);

    json["quarantineReason"] =
        static_cast<unsigned int>(value_in.quarantineReason);
    std::string unavailableReasonName3{};
    if (unavailableReasonName(value_in.quarantineReason,
                              unavailableReasonName3) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: unavailableReasonName returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    json["quarantineReasonName"] = unavailableReasonName3;
    json["observationRayEvidenceReason"] =
        static_cast<unsigned int>(value_in.observationRayEvidenceReason);
    std::string unavailableReasonName4{};
    if (unavailableReasonName(value_in.observationRayEvidenceReason,
                              unavailableReasonName4) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: unavailableReasonName returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    json["observationRayEvidenceReasonName"] = unavailableReasonName4;

    if (includeGeometry_in)
    {
        json["equation_World"]   = serializeVector4d(value_in.equation_World);
        json["centroid_World_m"] = serializeVector3d(value_in.centroid_World_m);
        json["minPlaneU_m"]      = serializeDouble(value_in.minPlaneU_m);
        json["maxPlaneU_m"]      = serializeDouble(value_in.maxPlaneU_m);
        json["minPlaneV_m"]      = serializeDouble(value_in.minPlaneV_m);
        json["maxPlaneV_m"]      = serializeDouble(value_in.maxPlaneV_m);
        json["finiteSupportCount"]        = value_in.finiteSupportCount;
        json["observationCount"]          = value_in.observationCount;
        json["cloudGeneration"]           = value_in.cloudGeneration;
        json["successfulRefitGeneration"] = value_in.successfulRefitGeneration;
        if (value_in.observationOrigin_World_m.has_value())
        {
            json["observationOrigin_World_m"] =
                serializeVector3d(*value_in.observationOrigin_World_m);
        }
    }

    return json;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs

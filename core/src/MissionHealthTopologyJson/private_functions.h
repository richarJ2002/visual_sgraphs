/*!
 * @file            private_functions.h
 *
 * @brief           Declares module-internal helpers shared between the
 *                  MissionHealthTopologyJson translation units.
 */

#ifndef VS_GRAPHS_CORE_MISSIONHEALTHTOPOLOGYJSON_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_MISSIONHEALTHTOPOLOGYJSON_PRIVATE_FUNCTIONS_H

#include "MissionHealthTopologyJson.h"
#include "MissionHealthTopologyJsonStatus.h"

#include <vector>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Projects entity keys to a sorted JSON array.
 *
 * @param[in]       keys_in
 *                  Keys to project.
 * @param[out]      json_out
 *                  JSON array of {kind, mapId, entityId} objects.
 *
 * @return          MISSION_HEALTH_TOPOLOGY_JSON_STATUS_SUCCESS.
 */
[[nodiscard]] MissionHealthTopologyJsonStatus
    entityKeysToJson(std::vector<semantic::EntityKey> keys_in,
                     nlohmann::json                  &json_out);

/*!
 * @brief           Maps a double to JSON, preserving non-finite values.
 *
 * @param[in]       value_in
 *                  Value to map.
 * @param[out]      json_out
 *                  The value, or "NaN"/"Infinity"/"-Infinity".
 *
 * @return          MISSION_HEALTH_TOPOLOGY_JSON_STATUS_SUCCESS.
 */
[[nodiscard]] MissionHealthTopologyJsonStatus
    finiteAwareDoubleToJson(double value_in, nlohmann::json &json_out);

/*!
 * @brief           Projects the fixed axiom-capability table to JSON.
 *
 * @param[out]      json_out
 *                  JSON array of capability rows.
 *
 * @return          MISSION_HEALTH_TOPOLOGY_JSON_STATUS_SUCCESS.
 */
[[nodiscard]] MissionHealthTopologyJsonStatus
    axiomCapabilitiesToJson(nlohmann::json &json_out);

} // namespace core
} // namespace vs_graphs

#endif /* VS_GRAPHS_CORE_MISSIONHEALTHTOPOLOGYJSON_PRIVATE_FUNCTIONS_H */

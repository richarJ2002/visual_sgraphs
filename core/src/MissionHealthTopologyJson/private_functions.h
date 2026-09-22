/*!
 * @file            private_functions.h
 *
 * @brief           Declares module-internal helpers shared between the
 *                  MissionHealthTopologyJson translation units.
 */

#ifndef VS_GRAPHS_CORE_MISSIONHEALTHTOPOLOGYJSON_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_MISSIONHEALTHTOPOLOGYJSON_PRIVATE_FUNCTIONS_H

#include "MissionHealthTopologyJson.h"

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
 *
 * @return          JSON array of {kind, mapId, entityId} objects.
 */
nlohmann::json entityKeysToJson(std::vector<semantic::EntityKey> keys_in);

/*!
 * @brief           Maps a double to JSON, preserving non-finite values.
 *
 * @param[in]       value_in
 *                  Value to map.
 *
 * @return          The value, or "NaN"/"Infinity"/"-Infinity".
 */
nlohmann::json finiteAwareDoubleToJson(double value_in);

/*!
 * @brief           Projects the fixed axiom-capability table to JSON.
 *
 * @return          JSON array of capability rows.
 */
nlohmann::json axiomCapabilitiesToJson();

} // namespace core
} // namespace vs_graphs

#endif /* VS_GRAPHS_CORE_MISSIONHEALTHTOPOLOGYJSON_PRIVATE_FUNCTIONS_H */

/*!
 * @file         finiteAwareDoubleToJson.cc
 *
 * @brief        Implements finiteAwareDoubleToJson declared in
 *               private_functions.h.
 */

#include "MissionHealthTopologyJson.h"

#include <cmath>

namespace vs_graphs
{
namespace core
{

MissionHealthTopologyJsonStatus
    finiteAwareDoubleToJson(double value_in, nlohmann::json &json_out)
{
    if (std::isnan(value_in))
    {
        json_out = "NaN";
        return MissionHealthTopologyJsonStatus::
            MISSION_HEALTH_TOPOLOGY_JSON_STATUS_SUCCESS;
    }
    if (std::isinf(value_in))
    {
        json_out = value_in > 0.0 ? "Infinity" : "-Infinity";
        return MissionHealthTopologyJsonStatus::
            MISSION_HEALTH_TOPOLOGY_JSON_STATUS_SUCCESS;
    }
    json_out = value_in;
    return MissionHealthTopologyJsonStatus::
        MISSION_HEALTH_TOPOLOGY_JSON_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

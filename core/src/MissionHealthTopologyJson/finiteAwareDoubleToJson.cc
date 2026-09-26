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

nlohmann::json finiteAwareDoubleToJson(double value_in)
{
    if (std::isnan(value_in))
    {
        return "NaN";
    }
    if (std::isinf(value_in))
    {
        return value_in > 0.0 ? "Infinity" : "-Infinity";
    }
    return value_in;
}

} // namespace core
} // namespace vs_graphs

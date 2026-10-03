/*!
 * @file            private_functions.h
 *
 * @brief           Declares module-internal helpers shared between the
 *                  RoomTracker translation units.
 *
 * @note            These entities were file-scope members of the
 *                  anonymous namespace of RoomTracker.cc;
 *                  external linkage here is module-internal only.
 */

#ifndef VS_GRAPHS_CORE_SEMANTIC_ROOMTRACKER_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_SEMANTIC_ROOMTRACKER_PRIVATE_FUNCTIONS_H

#include "Semantic/RoomTracker.h"
#include "Semantic/RoomTrackerStatus.h"

#include <cmath>
#include <iostream>
#include <sstream>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

[[nodiscard]] RoomTrackerStatus formatDouble(double       numericValue_in,
                                             std::string &formattedValue_out);

/*!
 * @brief           Returns the fixed text name of a tracking state.
 *
 * @param[in]       state_in
 *                  State to name.
 *
 * @param[out]      p_stateLiteral_out
 *                  Borrowed pointer to a static, null-terminated name.
 *                  "UNKNOWN" for the UNKNOWN state or an unlisted value.
 *
 * @return          ROOM_TRACKER_STATUS_SUCCESS always.
 */
[[nodiscard]] RoomTrackerStatus stateLiteral(RoomTrackingState state_in,
                                             const char *&p_stateLiteral_out);

/*!
 * @brief           Returns the fixed text name of a tracking event.
 *
 * @param[in]       event_in
 *                  Event to name.
 *
 * @param[out]      p_eventLiteral_out
 *                  Borrowed pointer to a static, null-terminated name.
 *                  "UNKNOWN_EVENT" for an unlisted value.
 *
 * @return          ROOM_TRACKER_STATUS_SUCCESS always.
 */
[[nodiscard]] RoomTrackerStatus eventLiteral(RoomTrackingEvent event_in,
                                             const char *&p_eventLiteral_out);

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // VS_GRAPHS_CORE_SEMANTIC_ROOMTRACKER_PRIVATE_FUNCTIONS_H

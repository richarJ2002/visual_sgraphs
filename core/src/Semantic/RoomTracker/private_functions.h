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

#include <cmath>
#include <iostream>
#include <sstream>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

std::string formatDouble(double numericValue_in);

const char *stateLiteral(RoomTrackingState state_in);

const char *eventLiteral(RoomTrackingEvent event_in);

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // VS_GRAPHS_CORE_SEMANTIC_ROOMTRACKER_PRIVATE_FUNCTIONS_H

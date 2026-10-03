/*!
 * @file            private_functions.h
 *
 * @brief           Declares module-internal helpers shared between the
 *                  LoopClosing translation units.
 *
 * @note            These helpers were file-scope functions inside the
 *                  anonymous namespace of LoopClosing.cc; external linkage
 *                  here is module-internal only. Names are kept verbatim
 *                  (identifier renaming is a separate step).
 */

#ifndef VS_GRAPHS_CORE_LOOPCLOSING_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_LOOPCLOSING_PRIVATE_FUNCTIONS_H
#include "LoopClosingStatus.h"

namespace vs_graphs
{
namespace core
{

class Map;

namespace semantic
{
class Floor;
} // namespace semantic

/*!
 * @brief           Merges duplicate floor evidence and rooms.
 */
[[nodiscard]] LoopClosingStatus
    mergeFloorEvidenceAndRooms(semantic::Floor *p_retainedFloor_inout,
                               semantic::Floor *p_duplicateFloor_in);

/*!
 * @brief           Collapses duplicate floors in the surviving map.
 */
[[nodiscard]] LoopClosingStatus collapseMergedFloors(Map *p_survivingMap_inout);

} // namespace core
} // namespace vs_graphs

#endif /* VS_GRAPHS_CORE_LOOPCLOSING_PRIVATE_FUNCTIONS_H */

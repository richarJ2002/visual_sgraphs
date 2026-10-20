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

#include "place_recognition_and_loop_closure/loop_closing/objects/LoopClosingStatusEnum.h"

namespace vs_graphs
{
namespace core
{

namespace map
{
class Map;
} /* namespace map */

namespace semantic
{
class Floor;
} /* namespace semantic */

/*!
 * @brief           Merges duplicate floor evidence and rooms.
 *
 * @param[in,out]   p_retainedFloor_inout
 *                  TODO
 *
 * @param[in]       p_duplicateFloor_in
 *                  TODO
 *
 * @return          TODO
 */
[[nodiscard]] loop_closing::LoopClosingStatus
    mergeFloorEvidenceAndRooms(semantic::Floor *p_retainedFloor_inout,
                               semantic::Floor *p_duplicateFloor_in);

/*!
 * @brief           Collapses duplicate floors in the surviving map.
 *
 * @param[in,out]   p_survivingMap_inout
 *                  TODO
 *
 * @return          TODO
 */
[[nodiscard]] loop_closing::LoopClosingStatus
    collapseMergedFloors(map::Map *p_survivingMap_inout);

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_LOOPCLOSING_PRIVATE_FUNCTIONS_H */

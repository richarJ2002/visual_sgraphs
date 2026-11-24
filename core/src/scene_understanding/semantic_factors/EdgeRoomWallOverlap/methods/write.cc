/*!
 * @file            write.cc
 *
 * @brief           Implements EdgeRoomWallOverlap::write(), declared in
 *                  EdgeRoomWallOverlapClass.h.
 */

#include "scene_understanding/semantic_factors/EdgeRoomWallOverlap/objects/EdgeRoomWallOverlapClass.h"

namespace vs_graphs
{
namespace core
{
namespace optimizable_types
{
namespace EdgeRoomWallOverlap
{

bool EdgeRoomWallOverlap::write(std::ostream &outputStream_inout) const
{
    return writeInformationMatrix(outputStream_inout);
}

} /* namespace EdgeRoomWallOverlap */
} /* namespace optimizable_types */
} /* namespace core */
} /* namespace vs_graphs */

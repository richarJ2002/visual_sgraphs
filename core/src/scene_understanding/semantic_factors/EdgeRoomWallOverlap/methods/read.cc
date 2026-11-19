/*!
 * @file            read.cc
 *
 * @brief           Implements EdgeRoomWallOverlap::read(), declared in
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

bool EdgeRoomWallOverlap::read(std::istream &inputStream_inout)
{
    readInformationMatrix(inputStream_inout);
    return true;
}

} /* namespace EdgeRoomWallOverlap */
} /* namespace optimizable_types */
} /* namespace core */
} /* namespace vs_graphs */

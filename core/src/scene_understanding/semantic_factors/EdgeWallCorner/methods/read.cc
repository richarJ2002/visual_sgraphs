/*!
 * @file            read.cc
 *
 * @brief           Implements EdgeWallCorner::read(), declared in
 *                  EdgeWallCornerClass.h.
 */

#include "scene_understanding/semantic_factors/EdgeWallCorner/objects/EdgeWallCornerClass.h"

namespace vs_graphs
{
namespace core
{
namespace optimizable_types
{
namespace EdgeWallCorner
{

bool EdgeWallCorner::read(std::istream &inputStream_inout)
{
    readInformationMatrix(inputStream_inout);
    return true;
}

} /* namespace EdgeWallCorner */
} /* namespace optimizable_types */
} /* namespace core */
} /* namespace vs_graphs */

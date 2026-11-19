/*!
 * @file            write.cc
 *
 * @brief           Implements EdgeWallCorner::write(), declared in
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

bool EdgeWallCorner::write(std::ostream &outputStream_inout) const
{
    return writeInformationMatrix(outputStream_inout);
}

} /* namespace EdgeWallCorner */
} /* namespace optimizable_types */
} /* namespace core */
} /* namespace vs_graphs */

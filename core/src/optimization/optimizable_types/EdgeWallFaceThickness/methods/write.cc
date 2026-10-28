/*!
 * @file            write.cc
 *
 * @brief           Implements EdgeWallFaceThickness::write(), declared in
 *                  EdgeWallFaceThicknessClass.h.
 */

#include "optimization/optimizable_types/EdgeWallFaceThickness/objects/EdgeWallFaceThicknessClass.h"

namespace vs_graphs
{
namespace core
{
namespace optimizable_types
{
namespace EdgeWallFaceThickness
{

bool EdgeWallFaceThickness::write(std::ostream &outputStream_inout) const
{
    return writeInformationMatrix(outputStream_inout);
}

} /* namespace EdgeWallFaceThickness */
} /* namespace optimizable_types */
} /* namespace core */
} /* namespace vs_graphs */

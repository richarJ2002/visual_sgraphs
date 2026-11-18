/*!
 * @file            read.cc
 *
 * @brief           Implements EdgeWallFaceThickness::read(), declared in
 *                  EdgeWallFaceThicknessClass.h.
 */

#include "scene_understanding/semantic_factors/EdgeWallFaceThickness/objects/EdgeWallFaceThicknessClass.h"

namespace vs_graphs
{
namespace core
{
namespace optimizable_types
{
namespace EdgeWallFaceThickness
{

bool EdgeWallFaceThickness::read(std::istream &inputStream_inout)
{
    readInformationMatrix(inputStream_inout);
    return true;
}

} /* namespace EdgeWallFaceThickness */
} /* namespace optimizable_types */
} /* namespace core */
} /* namespace vs_graphs */

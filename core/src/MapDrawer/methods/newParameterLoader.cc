/*!
 * @file         newParameterLoader.cc
 *
 * @brief        Implements MapDrawer::newParameterLoader declared in
 *               MapDrawer.h.
 */

#include "MapDrawer.h"

#include "Utils/Settings/objects/Settings.h"

namespace vs_graphs
{
namespace core
{

void MapDrawer::newParameterLoader(utils::settings::Settings *settings)
{
    keyFrameSize      = settings->keyFrameSize();
    keyFrameLineWidth = settings->keyFrameLineWidth();
    graphLineWidth    = settings->graphLineWidth();
    pointSize         = settings->pointSize();
    cameraSize        = settings->cameraSize();
    cameraLineWidth   = settings->cameraLineWidth();
}

} // namespace core
} // namespace vs_graphs

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

void MapDrawer::newParameterLoader(utils::settings::Settings *p_settings_inout)
{
    keyFrameSize      = p_settings_inout->keyFrameSize();
    keyFrameLineWidth = p_settings_inout->keyFrameLineWidth();
    graphLineWidth    = p_settings_inout->graphLineWidth();
    pointSize         = p_settings_inout->pointSize();
    cameraSize        = p_settings_inout->cameraSize();
    cameraLineWidth   = p_settings_inout->cameraLineWidth();
}

} // namespace core
} // namespace vs_graphs

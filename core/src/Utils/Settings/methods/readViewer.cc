/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            readViewer.cc
 *
 * @brief           Implements Settings::readViewer(), declared in
 *                  Utils/Settings/objects/Settings.h.
 */

#include "Utils/Settings/objects/Settings.h"

#include <opencv2/core/persistence.hpp>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace settings
{

void Settings::readViewer(cv::FileStorage &storage_in)
{
    bool found;

    viewerKeyFrameSize =
        readParameter<float>(storage_in, "Viewer.KeyFrameSize", found);
    viewerKeyFrameLineWidth =
        readParameter<float>(storage_in, "Viewer.KeyFrameLineWidth", found);
    viewerGraphLineWidth =
        readParameter<float>(storage_in, "Viewer.GraphLineWidth", found);
    viewerPointSize =
        readParameter<float>(storage_in, "Viewer.PointSize", found);
    viewerCameraSize =
        readParameter<float>(storage_in, "Viewer.CameraSize", found);
    viewerCameraLineWidth =
        readParameter<float>(storage_in, "Viewer.CameraLineWidth", found);
    viewerViewPointX =
        readParameter<float>(storage_in, "Viewer.ViewpointX", found);
    viewerViewPointY =
        readParameter<float>(storage_in, "Viewer.ViewpointY", found);
    viewerViewPointZ =
        readParameter<float>(storage_in, "Viewer.ViewpointZ", found);
    viewerViewPointF =
        readParameter<float>(storage_in, "Viewer.ViewpointF", found);
    viewerImageScale =
        readParameter<float>(storage_in, "Viewer.imageViewScale", found, false);

    if (!found)
        viewerImageScale = 1.0f;
}

} // namespace settings
} // namespace utils
} // namespace core
} // namespace vs_graphs

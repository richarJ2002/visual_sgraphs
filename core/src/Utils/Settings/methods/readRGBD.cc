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
 * @file            readRGBD.cc
 *
 * @brief           Implements Settings::readRGBD(), declared in
 *                  Utils/Settings/objects/Settings.h.
 */

#include "Utils/Settings/objects/Settings.h"

#include <utility>

#include <opencv2/core/persistence.hpp>

#include "Types/objects/SystemParams.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace settings
{

void Settings::readRGBD(cv::FileStorage &storage_in)
{
    bool found;

    depthMapScale =
        readParameter<float>(storage_in, "RGBD.DepthMapFactor", found);
    depthThreshold = readParameter<float>(storage_in, "Stereo.ThDepth", found);
    stereoBaseline = readParameter<float>(storage_in, "Stereo.b", found);
    baselineFocal  = stereoBaseline * calibration1->getParameter(0);
    nearThreshold  = readParameter<float>(storage_in, "RGBD.NearThresh", found);
    farThreshold   = readParameter<float>(storage_in, "RGBD.FarThresh", found);

    // set distance threshold in the system params
    types::SystemParams::getParams()->pointcloud.distanceThresh =
        std::make_pair(nearThreshold, farThreshold);
}

} // namespace settings
} // namespace utils
} // namespace core
} // namespace vs_graphs

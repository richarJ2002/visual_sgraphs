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
 * @file            parseViewerParamFile.cc
 *
 * @brief           Implements Viewer::parseViewerParamFile(), declared in
 *                  Viewer.h.
 */

#include "ResetCause.h"
#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace vs_graphs
{
namespace core
{

ViewerStatus Viewer::parseViewerParamFile(cv::FileStorage &settings_in,
                                          bool            &isParsed_out)
{
    bool isParameterMissing = false;
    imageViewerScale        = 1.f;

    float fps = settings_in["Camera.fps"];
    if (fps < 1)
        fps = 30;
    framePeriod = 1e3 / fps;

    cv::FileNode node = settings_in["Camera.width"];
    if (!node.empty())
    {
        imageWidth = node.real();
    }
    else
    {
        std::cerr
            << "*Camera.width parameter doesn't exist or is not a real number*"
            << std::endl;
        isParameterMissing = true;
    }

    node = settings_in["Camera.height"];
    if (!node.empty())
    {
        imageHeight = node.real();
    }
    else
    {
        std::cerr
            << "*Camera.height parameter doesn't exist or is not a real number*"
            << std::endl;
        isParameterMissing = true;
    }

    node = settings_in["Viewer.imageViewScale"];
    if (!node.empty())
    {
        imageViewerScale = node.real();
    }

    node = settings_in["Viewer.ViewpointX"];
    if (!node.empty())
    {
        viewpointX = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointX parameter doesn't exist or is not a "
                     "real number*"
                  << std::endl;
        isParameterMissing = true;
    }

    node = settings_in["Viewer.ViewpointY"];
    if (!node.empty())
    {
        viewpointY = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointY parameter doesn't exist or is not a "
                     "real number*"
                  << std::endl;
        isParameterMissing = true;
    }

    node = settings_in["Viewer.ViewpointZ"];
    if (!node.empty())
    {
        viewpointZ = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointZ parameter doesn't exist or is not a "
                     "real number*"
                  << std::endl;
        isParameterMissing = true;
    }

    node = settings_in["Viewer.ViewpointF"];
    if (!node.empty())
    {
        viewpointF = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointF parameter doesn't exist or is not a "
                     "real number*"
                  << std::endl;
        isParameterMissing = true;
    }

    isParsed_out = !isParameterMissing;
    return ViewerStatus::VIEWER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

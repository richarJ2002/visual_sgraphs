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
 * @file            extractOrbFeatures.cc
 *
 * @brief           Implements Frame::extractOrbFeatures(), declared in
 * frame/objects/FrameClass.h.
 */

#include "frame/objects/FrameClass.h"

#include "camera_models/geometric_camera/geometric_camera.h"
#include "camera_models/kannala_brandt8/kannala_brandt8.h"
#include "camera_models/pinhole/pinhole.h"

#include "keyframe/keyframe.h"
#include "map_point/map_point.h"
#include "orb_extractor/orb_extractor.h"
#include "orb_matcher/orb_matcher.h"
#include "stereo_match_outlier_rejection/stereo_match_outlier_rejection.h"
#include "utils/converter/converter.h"

#include <thread>

namespace vs_graphs
{
namespace core
{
namespace frame
{

FrameStatus Frame::extractOrbFeatures(int            flag_in,
                                      const cv::Mat &imageGray_in,
                                      const int      x0_in,
                                      const int      x1_in)
{
    std::vector<int> lappings = {x0_in, x1_in};
    /* Compute ORB based on the flag (0: left, 1: right) */
    if (flag_in == 0)
        monoLeft = (*p_orbExtractorLeft)(imageGray_in,
                                         cv::Mat(),
                                         keyPoints,
                                         descriptors,
                                         lappings);
    else
        monoRight = (*p_orbExtractorRight)(imageGray_in,
                                           cv::Mat(),
                                           keyPointsRight,
                                           descriptorsRight,
                                           lappings);

    return FrameStatus::FRAME_STATUS_SUCCESS;
}

} /* namespace frame */
} /* namespace core */
} /* namespace vs_graphs */

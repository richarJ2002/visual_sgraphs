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
 * @file            trackStats2File.cc
 *
 * @brief           Implements Tracking::trackStats2File(), declared in
 *                  Tracking.h.
 */

#include "Tracking.h"

#include <iomanip>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

#ifdef REGISTER_TIMES
TrackingStatus Tracking::trackStats2File()
{
    std::ofstream f;
    f.open("SessionInfo.txt");
    f << std::fixed;
    std::vector<KeyFrame *> atlasAllKeyFrames{};
    if (p_atlas->getAllKeyFrames(atlasAllKeyFrames) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    f << "Number of KFs: " << atlasAllKeyFrames.size() << std::endl;
    std::vector<MapPoint *> atlasAllMapPoints{};
    if (p_atlas->getAllMapPoints(atlasAllMapPoints) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMapPoints returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    f << "Number of MPs: " << atlasAllMapPoints.size() << std::endl;

    f << "OpenCV version: " << CV_VERSION << std::endl;

    f.close();

    f.open("TrackingTimeStats.txt");
    f << std::fixed << std::setprecision(6);

    f << "#Image Rect[ms], Image Resize[ms], ORB ext[ms], Stereo match[ms], "
         "IMU preint[ms], Pose pred[ms], LM track[ms], KF dec[ms], Total[ms]"
      << std::endl;

    for (std::size_t trackTotalTimeIndex = 0;
         trackTotalTimeIndex < trackTotalTimes_ms.size();
         ++trackTotalTimeIndex)
    {
        double stereoRectified = 0.0;
        if (!stereoRectificationTimes_ms.empty())
        {
            stereoRectified = stereoRectificationTimes_ms[trackTotalTimeIndex];
        }

        double resizeImage = 0.0;
        if (!imageResizeTimes_ms.empty())
        {
            resizeImage = imageResizeTimes_ms[trackTotalTimeIndex];
        }

        double stereoMatch = 0.0;
        if (!stereoMatchTimes_ms.empty())
        {
            stereoMatch = stereoMatchTimes_ms[trackTotalTimeIndex];
        }

        double imuPreint = 0.0;
        if (!imuIntegrationTimes_ms.empty())
        {
            imuPreint = imuIntegrationTimes_ms[trackTotalTimeIndex];
        }

        f << stereoRectified << "," << resizeImage << ","
          << orbExtractionTimes_ms[trackTotalTimeIndex] << "," << stereoMatch
          << "," << imuPreint << ","
          << posePredictionTimes_ms[trackTotalTimeIndex] << ","
          << localMapTrackTimes_ms[trackTotalTimeIndex] << ","
          << newKeyFrameTimes_ms[trackTotalTimeIndex] << ","
          << trackTotalTimes_ms[trackTotalTimeIndex] << std::endl;
    }

    f.close();

    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}
#endif

} // namespace core
} // namespace vs_graphs

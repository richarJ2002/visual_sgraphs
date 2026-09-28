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

#include "Tracking.h"

namespace vs_graphs
{
namespace core
{

#ifdef REGISTER_TIMES
void Tracking::trackStats2File()
{
    ofstream f;
    f.open("SessionInfo.txt");
    f << fixed;
    f << "Number of KFs: " << p_atlas->getAllKeyFrames().size() << endl;
    f << "Number of MPs: " << p_atlas->getAllMapPoints().size() << endl;

    f << "OpenCV version: " << CV_VERSION << endl;

    f.close();

    f.open("TrackingTimeStats.txt");
    f << fixed << setprecision(6);

    f << "#Image Rect[ms], Image Resize[ms], ORB ext[ms], Stereo match[ms], "
         "IMU preint[ms], Pose pred[ms], LM track[ms], KF dec[ms], Total[ms]"
      << endl;

    for (int trackTotalTimeIndex = 0;
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
          << trackTotalTimes_ms[trackTotalTimeIndex] << endl;
    }

    f.close();
}
#endif

} // namespace core
} // namespace vs_graphs

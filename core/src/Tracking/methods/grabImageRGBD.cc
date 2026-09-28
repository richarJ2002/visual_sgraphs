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

Sophus::SE3f Tracking::grabImageRGBD(
    const cv::Mat                                &imageRgb_in,
    const cv::Mat                                &imageD_in,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pointcloud_in,
    const double                                 &timestamp_in,
    string                                        filename_in,
    const std::vector<semantic::Marker *>         markers_in,
    const std::vector<semantic::Room *>           rooms_in)
{
    // Set arguments to local variables
    env_rooms = rooms_in;

    // Adaptive FAST threshold: adjust before feature extraction
    adjustFASTThreshold();

    imageGray          = imageRgb_in;
    cv::Mat imageDepth = imageD_in;

    if (imageGray.channels() == 3)
    {
        if (isRgbEnabled)
            cvtColor(imageGray, imageGray, cv::COLOR_RGB2GRAY);
        else
            cvtColor(imageGray, imageGray, cv::COLOR_BGR2GRAY);
    }
    else if (imageGray.channels() == 4)
    {
        if (isRgbEnabled)
            cvtColor(imageGray, imageGray, cv::COLOR_RGBA2GRAY);
        else
            cvtColor(imageGray, imageGray, cv::COLOR_BGRA2GRAY);
    }

    if ((fabs(depthMapFactor - 1.0f) > 1e-5) || imageDepth.type() != CV_32F)
        imageDepth.convertTo(imageDepth, CV_32F, depthMapFactor);

    // RGB-D
    if (sensor == System::RGBD)
        currentFrame = Frame(imageRgb_in,
                             imageGray,
                             imageDepth,
                             p_pointcloud_in,
                             timestamp_in,
                             p_orbExtractorLeft,
                             p_orbVocabulary,
                             calibrationMatrix,
                             distortionCoefficients,
                             mbf,
                             depthThreshold,
                             p_camera,
                             nullptr,
                             IMU::Calib(),
                             markers_in);
    // RGB-D Intertial
    else if (sensor == System::IMU_RGBD)
        currentFrame = Frame(imageRgb_in,
                             imageGray,
                             imageDepth,
                             p_pointcloud_in,
                             timestamp_in,
                             p_orbExtractorLeft,
                             p_orbVocabulary,
                             calibrationMatrix,
                             distortionCoefficients,
                             mbf,
                             depthThreshold,
                             p_camera,
                             &lastFrame,
                             *p_imuCalibration,
                             markers_in);

    currentFrame.fileName  = filename_in;
    currentFrame.datasetId = numDataset;

#ifdef REGISTER_TIMES
    orbExtractionTimes_ms.push_back(currentFrame.orbExtractionTime);
#endif

    track();

    return currentFrame.getPose();
}

} // namespace core
} // namespace vs_graphs

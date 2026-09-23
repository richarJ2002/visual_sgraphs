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
    const cv::Mat                                &imRGB,
    const cv::Mat                                &imD,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud,
    const double                                 &timestamp,
    string                                        filename,
    const std::vector<semantic::Marker *>         markers,
    const std::vector<semantic::Room *>           rooms)
{
    // Set arguments to local variables
    env_rooms = rooms;

    // Adaptive FAST threshold: adjust before feature extraction
    adjustFASTThreshold();

    imageGray       = imRGB;
    cv::Mat imDepth = imD;

    if (imageGray.channels() == 3)
    {
        if (rgbEnabled)
            cvtColor(imageGray, imageGray, cv::COLOR_RGB2GRAY);
        else
            cvtColor(imageGray, imageGray, cv::COLOR_BGR2GRAY);
    }
    else if (imageGray.channels() == 4)
    {
        if (rgbEnabled)
            cvtColor(imageGray, imageGray, cv::COLOR_RGBA2GRAY);
        else
            cvtColor(imageGray, imageGray, cv::COLOR_BGRA2GRAY);
    }

    if ((fabs(depthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
        imDepth.convertTo(imDepth, CV_32F, depthMapFactor);

    // RGB-D
    if (sensor == System::RGBD)
        currentFrame = Frame(imRGB,
                             imageGray,
                             imDepth,
                             pointcloud,
                             timestamp,
                             p_orbExtractorLeft,
                             p_orbVocabulary,
                             calibrationMatrix,
                             distortionCoefficients,
                             mbf,
                             depthThreshold,
                             p_camera,
                             nullptr,
                             IMU::Calib(),
                             markers);
    // RGB-D Intertial
    else if (sensor == System::IMU_RGBD)
        currentFrame = Frame(imRGB,
                             imageGray,
                             imDepth,
                             pointcloud,
                             timestamp,
                             p_orbExtractorLeft,
                             p_orbVocabulary,
                             calibrationMatrix,
                             distortionCoefficients,
                             mbf,
                             depthThreshold,
                             p_camera,
                             &lastFrame,
                             *p_imuCalibration,
                             markers);

    currentFrame.fileName  = filename;
    currentFrame.datasetId = numDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(currentFrame.orbExtractionTime);
#endif

    track();

    return currentFrame.getPose();
}

} // namespace core
} // namespace vs_graphs

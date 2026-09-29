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

TrackingStatus Tracking::changeCalibration(const string &settingPath_in)
{
    cv::FileStorage settings(settingPath_in, cv::FileStorage::READ);
    float           fx = settings["Camera.fx"];
    float           fy = settings["Camera.fy"];
    float           cx = settings["Camera.cx"];
    float           cy = settings["Camera.cy"];

    calibrationMatrixEigen.setIdentity();
    calibrationMatrixEigen(0, 0) = fx;
    calibrationMatrixEigen(1, 1) = fy;
    calibrationMatrixEigen(0, 2) = cx;
    calibrationMatrixEigen(1, 2) = cy;

    cv::Mat K         = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.copyTo(calibrationMatrix);

    cv::Mat distanceCoefficients(4, 1, CV_32F);
    distanceCoefficients.at<float>(0) = settings["Camera.k1"];
    distanceCoefficients.at<float>(1) = settings["Camera.k2"];
    distanceCoefficients.at<float>(2) = settings["Camera.p1"];
    distanceCoefficients.at<float>(3) = settings["Camera.p2"];
    const float k3                    = settings["Camera.k3"];
    if (k3 != 0)
    {
        distanceCoefficients.resize(5);
        distanceCoefficients.at<float>(4) = k3;
    }
    distanceCoefficients.copyTo(distortionCoefficients);

    mbf = settings["Camera.bf"];

    Frame::areInitialComputationsDone = true;

    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

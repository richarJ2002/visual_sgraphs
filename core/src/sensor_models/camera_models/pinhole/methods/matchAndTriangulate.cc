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
 * @file            matchAndTriangulate.cc
 *
 * @brief           Implements Pinhole::matchAndTriangulate().
 */

#include "sensor_models/camera_models/pinhole/objects/PinholeClass.h"
#include "sensor_models/camera_models/geometric_camera/geometric_camera.h"

#include <opencv2/features2d.hpp>

#include "dependency_lineage/sophus/objects/se3.hpp"

namespace vs_graphs
{
namespace core
{
namespace camera_models
{
namespace pinhole
{

bool Pinhole::matchAndTriangulate(
    [[maybe_unused]] const cv::KeyPoint                &keypoint1_in,
    [[maybe_unused]] const cv::KeyPoint                &keypoint2_in,
    [[maybe_unused]] geometric_camera::GeometricCamera *p_otherCamera_inout,
    [[maybe_unused]] Sophus::SE3f                      &pose1_inout,
    [[maybe_unused]] Sophus::SE3f                      &pose2_inout,
    [[maybe_unused]] const float                        sigmaLevel1_in,
    [[maybe_unused]] const float                        sigmaLevel2_in,
    [[maybe_unused]] Eigen::Vector3f                   &point3d_inout)
{
    return false;
}

} /* namespace pinhole */
} /* namespace camera_models */
} /* namespace core */
} /* namespace vs_graphs */

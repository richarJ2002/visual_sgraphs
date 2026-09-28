/*!
 * This file is part of ORB-SLAM3.
 * Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details:
 * https://www.gnu.org/licenses/
 */

/*!
 * @file            unprojectEig.cc
 *
 * @brief           Implements KannalaBrandt8::unprojectEig(), declared
 *                  in CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h.
 */

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

namespace vs_graphs::core::camera_models::kannalabrandt8
{
Eigen::Vector3f KannalaBrandt8::unprojectEig(const cv::Point2f &point2d_in)
{
    cv::Point3f unprojectedRay = this->unproject(point2d_in);
    return Eigen::Vector3f(unprojectedRay.x,
                           unprojectedRay.y,
                           unprojectedRay.z);
}
} // namespace vs_graphs::core::camera_models::kannalabrandt8

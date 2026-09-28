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
 * @file            epipolarConstrain.cc
 *
 * @brief           Implements KannalaBrandt8::epipolarConstrain(),
 *                  declared in
 * CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h.
 */

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace vs_graphs::core::camera_models::kannalabrandt8
{
bool KannalaBrandt8::epipolarConstrain(
    geometriccamera::GeometricCamera *p_otherCamera_in,
    const cv::KeyPoint               &keypoint1_in,
    const cv::KeyPoint               &keypoint2_in,
    const Eigen::Matrix3f            &rotation12_in,
    const Eigen::Vector3f            &translation12_in,
    const float                       sigmaLevel_in,
    const float                       uncertainty_in)
{
    Eigen::Vector3f point3d;
    float           parallax{};
    if (this->triangulateMatches(p_otherCamera_in,
                                 keypoint1_in,
                                 keypoint2_in,
                                 rotation12_in,
                                 translation12_in,
                                 sigmaLevel_in,
                                 uncertainty_in,
                                 point3d,
                                 parallax) !=
        KannalaBrandt8Status::KANNALA_BRANDT8_STATUS_SUCCESS)
    {
        // triangulateMatches cannot fail; continue as before.
    }
    return parallax > 0.0001f;
}
} // namespace vs_graphs::core::camera_models::kannalabrandt8

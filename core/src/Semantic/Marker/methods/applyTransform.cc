/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            applyTransform.cc
 *
 * @brief           Implements Marker::applyTransform(), declared in
 *                  Semantic/Marker.h.
 */

#include "Semantic/Marker.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

MarkerStatus
    Marker::applyTransform(const g2o::Sim3 &transform_oldWorldToNewWorld_in)
{
    std::lock_guard<std::mutex> lock(geometryMutex);

    const Eigen::Matrix3f rotation_oldWorldToNewWorld =
        transform_oldWorldToNewWorld_in.rotation()
            .toRotationMatrix()
            .cast<float>();

    const Eigen::Matrix3f rotation_markerToNewWorld =
        rotation_oldWorldToNewWorld * globalPose.rotationMatrix();

    const Eigen::Vector3f position_newWorld_m =
        transform_oldWorldToNewWorld_in
            .map(globalPose.translation().cast<double>())
            .cast<float>();

    globalPose = Sophus::SE3f(rotation_markerToNewWorld, position_newWorld_m);

    return MarkerStatus::MARKER_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs

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
 * @file            PlanePatchStruct.h
 *
 * @brief           Declares PlanePatch, one candidate plane found in a
 *                  keyframe's point cloud on its way to the map.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEGMENTATION_OBJECTS_PLANE_PATCH_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEGMENTATION_OBJECTS_PLANE_PATCH_STRUCT_H

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "mapping/geometric/Plane/objects/PlaneClass.h"

namespace vs_graphs
{
namespace core
{
namespace semantic_segmentation
{

/*!
 * @brief           One candidate plane patch of a keyframe, in the camera
 *                  frame of that keyframe.
 */
struct PlanePatch
{
  public:
    /*!
     * @brief           Points supporting the patch, camera frame, metres.
     *                  Shared with the map plane the patch creates or updates.
     */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_supportCloud_camera_m{};

    /*!
     * @brief           Plane equation (a, b, c, d) in the camera frame, unit
     *                  normal; d >= 0, so the normal faces the camera.
     */
    Eigen::Vector4d planeEquation_camera = Eigen::Vector4d::Zero();

    /*!
     * @brief           The segmenter's class as a guess until the plane
     *                  checkpoint, the label from geometry after it.
     */
    geometric::Plane::PlaneVariant planeLabel =
        geometric::Plane::PlaneVariant::UNDEFINED;
};

} /* namespace semantic_segmentation */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEGMENTATION_OBJECTS_PLANE_PATCH_STRUCT_H \
        */

/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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
 * @file            PlaneEvidenceStruct.h
 *
 * @brief           Declares the observed-plane evidence that
 *                  Utils::findMatchingPlane() compares with mapped planes.
 */

#ifndef VS_GRAPHS_CORE_SRC_UTILS_UTILS_OBJECTS_PLANEEVIDENCESTRUCT_H
#define VS_GRAPHS_CORE_SRC_UTILS_UTILS_OBJECTS_PLANEEVIDENCESTRUCT_H

#include "mapping/geometric/Plane/Plane.h"

#include <optional>
#include <variant>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

/*!
 * @brief           Observed plane to be matched, expressed in the world frame
 *                  of the mapped planes it is compared with.
 */
struct PlaneEvidence
{
  public:
    /*!
     * @brief           Plane equation [n, d] with n.x + d = 0, world frame; the
     *                  normal need not be unit length.
     */
    Eigen::Vector4d planeEquation_world{Eigen::Vector4d::Zero()};

    /*!
     * @brief           Support points of the plane, world frame, metres;
     *                  shared and read only. Null or empty never matches.
     */
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_supportCloud_world_m;

    /*!
     * @brief           Plane centroid, world frame, metres. OPPOSITE_FACE
     *                  requires it; SAME_FACE ignores it and uses the mean of
     *                  the support cloud.
     */
    std::optional<Eigen::Vector3d> planeCentroid_world_m;

    /*!
     * @brief           Semantic label of the observed plane; SAME_FACE only.
     */
    geometric::Plane::PlaneVariant planeLabel{
        geometric::Plane::PlaneVariant::UNDEFINED};

    /*!
     * @brief           Side the plane was observed from. SAME_FACE (walls
     *                  only) keeps faces seen from opposite sides apart:
     *                  empty means no side check, Eigen::Vector3d is one
     *                  camera centre (world frame, metres), and
     *                  ObservationSideSnapshot is the observing cameras of a
     *                  mapped plane, taken against planeEquation_world divided
     *                  by its normal length. OPPOSITE_FACE requires the
     *                  plane's stamped observation origin as Eigen::Vector3d.
     */
    std::variant<std::monostate,
                 Eigen::Vector3d,
                 geometric::Plane::ObservationSideSnapshot>
        observerSide;
};

} /* namespace utils */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_UTILS_UTILS_OBJECTS_PLANEEVIDENCESTRUCT_H */

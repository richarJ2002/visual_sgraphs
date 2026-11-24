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
 * @file            PlaneMatchSettingsStruct.h
 *
 * @brief           Declares the thresholds and rules of one
 *                  Utils::findMatchingPlane() call.
 */

#ifndef VS_GRAPHS_CORE_SRC_UTILS_UTILS_OBJECTS_PLANEMATCHSETTINGSSTRUCT_H
#define VS_GRAPHS_CORE_SRC_UTILS_UTILS_OBJECTS_PLANEMATCHSETTINGSSTRUCT_H

#include "mapping/geometry_helpers/objects/PlaneMatchModeEnum.h"
#include "mapping/geometry_helpers/objects/PlaneTypeRuleEnum.h"

#include <Eigen/Core>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

/*!
 * @brief           Settings bundle for one plane-matching call; each caller
 *                  fills it from its own configuration keys. Each field
 *                  names the modes that read it.
 */
struct PlaneMatchSettings
{
  public:
    /*!
     * @brief           Plane relation that counts as a match.
     */
    PlaneMatchMode matchMode{PlaneMatchMode::SAME_FACE};

    /*!
     * @brief           Rule comparing the observed label with the mapped
     *                  plane's type; SAME_FACE only.
     */
    PlaneTypeRule typeRule{PlaneTypeRule::EXPECTED_TYPE_OR_UNDEFINED};

    /*!
     * @brief           Largest angle between the plane normals, in radians;
     *                  both modes. SAME_FACE raises values below 0.01 rad to
     *                  0.01 rad.
     */
    double maximumAngle_rad{0.0};

    /*!
     * @brief           Largest perpendicular distance between the planes, in
     *                  metres; SAME_FACE only. Values below 0.01 m are raised
     *                  to 0.01 m.
     */
    double maximumPlaneDistance_m{0.0};

    /*!
     * @brief           Centroid distance under which two coplanar planes match
     *                  without support-cloud evidence, in metres; SAME_FACE
     *                  only. Values below 0.10 m are raised to 0.10 m.
     */
    double maximumCentroidDistance_m{0.0};

    /*!
     * @brief           Largest nearest-point gap between the support clouds,
     *                  in metres; SAME_FACE only. Values below 0.05 m are
     *                  raised to 0.05 m.
     *                  A wall under the enabled wall-extension rule uses
     *                  sem_seg.reassociate.wall_extension.maximum_in_plane_gap
     *                  instead.
     */
    double maximumCloudGap_m{0.0};

    /*!
     * @brief           Largest face separation, in metres: the mean distance
     *                  of each plane centroid from the other plane;
     *                  OPPOSITE_FACE only.
     */
    double maximumThickness_m{0.0};

    /*!
     * @brief           Smallest footprint overlap as a fraction of the smaller
     *                  footprint, 0 to 1; OPPOSITE_FACE only.
     */
    double minimumOverlapRatio{0.0};

    /*!
     * @brief           Ground normal in the world frame, any length; zero when
     *                  unknown. The footprints are compared on axes tied to
     *                  it; OPPOSITE_FACE only.
     */
    Eigen::Vector3d groundNormal_world{Eigen::Vector3d::Zero()};
};

} /* namespace utils */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_UTILS_UTILS_OBJECTS_PLANEMATCHSETTINGSSTRUCT_H */

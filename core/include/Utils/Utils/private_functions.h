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
 * @file            private_functions.h
 *
 * @brief           Declares the internal plane-observation helpers shared
 *                  by the Utils method translation units
 *                  (CPP_CODING_STANDARD.md Section 5.4). Not exported or
 *                  included by a public header.
 */

#ifndef UTILS_UTILS_PRIVATE_FUNCTIONS_H
#define UTILS_UTILS_PRIVATE_FUNCTIONS_H

#include <limits>
#include <optional>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Geometric/Plane.h"
#include "Semantic/Passage.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{
/*!
 * @brief        Median camera-to-plane side evidence for a mapped plane.
 */
struct ObservationSideEvidence
{
    std::optional<double> medianSignedDistance_m;
    bool                  ambiguous{false};
};

/*!
 * @brief        Estimates which side of a mapped plane observed it.
 *
 *               Plane coefficients have an arbitrary sign, so the caller
 *               supplies an already normalized and consistently oriented
 *               equation. The median camera-to-plane distance rejects
 *               isolated poses produced during relocalization.
 *               Observations too close to the surface do not provide
 *               reliable side evidence.
 *
 * @param[in]    p_plane_in
 *               Plane whose observing keyframes provide the camera
 *               positions.
 * @param[in]    planeEquation_World_in
 *               Normalized plane equation expressed in the active map
 *               frame.
 *
 * @return       Median signed camera distance in metres, or no value
 *               when the available observations do not establish a side.
 */
ObservationSideEvidence getMedianObservationSide_World_m(
    geometric::Plane      *p_plane_in,
    const Eigen::Vector4d &planeEquation_World_in);

/*!
 * @brief Finite ranges of a cloud projected onto two plane-tangent axes.
 */
struct ProjectedPlaneBounds
{
    double minimumU_m = std::numeric_limits<double>::max();
    double maximumU_m = std::numeric_limits<double>::lowest();
    double minimumV_m = std::numeric_limits<double>::max();
    double maximumV_m = std::numeric_limits<double>::lowest();
    bool   valid      = false;
};

/*!
 * @brief        Projects a finite plane cloud onto a shared in-plane
 *               coordinate system.
 *
 * @param[in]    p_planeCloud_in
 *               Plane support cloud expressed in the active map frame.
 * @param[in]    tangentU_World_in
 *               First unit tangent of the common plane.
 * @param[in]    tangentV_World_in
 *               Second unit tangent of the common plane.
 *
 * @return       Finite projected bounds, or invalid bounds for an empty
 *               cloud.
 */
ProjectedPlaneBounds projectPlaneBounds(
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_planeCloud_in,
    const Eigen::Vector3d                              &tangentU_World_in,
    const Eigen::Vector3d                              &tangentV_World_in);

/*!
 * @brief        Tests whether two finite clouds overlap or extend one
 *               another along the same plane.
 *
 * @param[in]    p_firstCloud_in
 *               First finite plane support cloud.
 * @param[in]    p_secondCloud_in
 *               Second finite plane support cloud.
 * @param[in]    commonNormal_World_in
 *               Unit normal shared by the already equation-compatible
 *               planes.
 * @param[in]    maximumInPlaneGap_m_in
 *               Maximum permitted extension gap along either tangent.
 * @param[in]    minimumOrthogonalOverlap_m_in
 *               Required overlap along the other tangent.
 *
 * @return       True when the clouds overlap or form adjacent finite
 *               extensions.
 */
bool finiteWallExtentsAreCompatible(
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_firstCloud_in,
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_secondCloud_in,
    const Eigen::Vector3d                              &commonNormal_World_in,
    const double                                        maximumInPlaneGap_m_in,
    const double minimumOrthogonalOverlap_m_in);

/*!
 * @brief        Tests whether a segment traverses a passable passage
 *               aperture.
 *
 *               Mirrors the far-side wall-routing crossing test used by
 *               the semantic manager so that the merge path can apply
 *               the same rule when imported walls are copied into a
 *               retained room.
 *
 * @param[in]    segmentStart_World_m_in
 *               First endpoint in the active map frame.
 * @param[in]    segmentEnd_World_m_in
 *               Second endpoint in the active map frame.
 * @param[in]    p_passage_in
 *               Passable passage defining the finite aperture.
 * @param[in]    groundNormal_World_in
 *               Unit ground normal in the active map frame.
 * @param[in]    openingMargin_m_in
 *               Aperture expansion used for noisy geometry.
 * @param[in]    minimumSideDistance_m_in
 *               Required endpoint distance from plane.
 *
 * @return       True only when the segment crosses inside the finite
 *               opening.
 */
bool crossesPassablePassageOpening(
    const Eigen::Vector3d              &segmentStart_World_m_in,
    const Eigen::Vector3d              &segmentEnd_World_m_in,
    vs_graphs::core::semantic::Passage *p_passage_in,
    const Eigen::Vector3d              &groundNormal_World_in,
    const double                        openingMargin_m_in,
    const double                        minimumSideDistance_m_in);

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs

#endif // UTILS_UTILS_PRIVATE_FUNCTIONS_H

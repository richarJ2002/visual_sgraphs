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

#ifndef VS_GRAPHS_CORE_SRC_UTILS_UTILS_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_SRC_UTILS_UTILS_PRIVATE_FUNCTIONS_H

#include <limits>
#include <optional>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "map_state/atlas/atlas.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/room/Room.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{
/*!
 * @brief           Median camera-to-plane side evidence for a mapped plane.
 */
struct ObservationSideEvidence
{
    /*!
     * @brief           Median signed camera-to-plane distance in metres, or no
     *                  value when the observations do not establish a side.
     */
    std::optional<double> medianSignedDistance_m;
    /*!
     * @brief           True when the observing cameras do not agree on one side
     *                  of the plane, so no side can be chosen.
     */
    bool                  isAmbiguous{false};
};

/*!
 * @brief           Estimates which side of a mapped plane observed it.
 *
 *                  Plane coefficients have an arbitrary sign, so the caller
 *                  supplies an already normalized and consistently oriented
 *                  equation. The median camera-to-plane distance rejects
 *                  isolated poses produced during relocalization.
 *                  Observations too close to the surface do not provide
 *                  reliable side evidence.
 *
 * @param[in]       p_plane_in
 *                  Plane whose observing keyframes provide the camera
 *                  positions.
 *
 * @param[in]       planeEquation_world_in
 *                  Normalized plane equation expressed in the active map
 *                  frame.
 *
 * @param[out]      medianObservationSide_world_m_out
 *                  Median signed camera distance in metres, or no value when
 *                  the available observations do not establish a side.
 *
 * @return          UTILS_STATUS_SUCCESS.
 */
[[nodiscard]] UtilsStatus getMedianObservationSide_world_m(
    geometric::Plane        *p_plane_in,
    const Eigen::Vector4d   &planeEquation_world_in,
    ObservationSideEvidence &medianObservationSide_world_m_out);

/*!
 * @brief           Finite ranges of a cloud projected onto two plane-tangent
 *                  axes.
 */
struct ProjectedPlaneBounds
{
    /*!
     * @brief           Smallest coordinate along the first tangent, in metres;
     *                  the largest double until a finite point is seen.
     */
    double minimumU_m = std::numeric_limits<double>::max();
    /*!
     * @brief           Largest coordinate along the first tangent, in metres;
     *                  the lowest double until a finite point is seen.
     */
    double maximumU_m = std::numeric_limits<double>::lowest();
    /*!
     * @brief           Smallest coordinate along the second tangent, in metres;
     *                  the largest double until a finite point is seen.
     */
    double minimumV_m = std::numeric_limits<double>::max();
    /*!
     * @brief           Largest coordinate along the second tangent, in metres;
     *                  the lowest double until a finite point is seen.
     */
    double maximumV_m = std::numeric_limits<double>::lowest();
    /*!
     * @brief           True once at least one finite point has set the bounds.
     */
    bool   isValid = false;
};

/*!
 * @brief           Projects a finite plane cloud onto a shared in-plane
 *                  coordinate system.
 *
 * @param[in]       p_planeCloud_in
 *                  Plane support cloud expressed in the active map frame.
 *
 * @param[in]       tangentU_world_in
 *                  First unit tangent of the common plane.
 *
 * @param[in]       tangentV_world_in
 *                  Second unit tangent of the common plane.
 *
 * @param[out]      planeBounds_out
 *                  Finite projected bounds, or invalid bounds for an empty
 *                  cloud.
 *
 * @return          UTILS_STATUS_SUCCESS.
 */
[[nodiscard]] UtilsStatus projectPlaneBounds(
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_planeCloud_in,
    const Eigen::Vector3d                              &tangentU_world_in,
    const Eigen::Vector3d                              &tangentV_world_in,
    ProjectedPlaneBounds                               &planeBounds_out);

/*!
 * @brief           Tests whether two finite clouds overlap or extend one
 *                  another along the same plane.
 *
 *                  Fragments that do not overlap are separated when a mapped
 *                  WALL roughly perpendicular to the face lies between them,
 *                  reaches the face and runs at least 0.50 m into the
 *                  observed side: they are faces of neighbouring rooms meeting
 *                  at a partition, not one surface broken by a door.
 *
 * @param[in]       p_firstCloud_in
 *                  First finite plane support cloud.
 *
 * @param[in]       p_secondCloud_in
 *                  Second finite plane support cloud.
 *
 * @param[in]       commonEquation_world_in
 *                  Equation shared by the already equation-compatible planes,
 *                  in the active map frame.
 *
 * @param[in]       observerSide_m_in
 *                  Signed observer distance from commonEquation_world_in, in
 *                  metres; 0 when unknown, which skips the separation test.
 *
 * @param[in]       mappedPlanes_in
 *                  Mapped planes searched for a separating wall, non-owning;
 *                  null, bad and non-WALL entries are ignored.
 *
 * @param[in]       maximumInPlaneGap_m_in
 *                  Maximum permitted extension gap along either tangent.
 *
 * @param[in]       minimumOrthogonalOverlap_m_in
 *                  Required overlap along the other tangent.
 *
 * @param[in]       touchingTolerance_m_in
 *                  Distance, in metres, within which a fragment end or the
 *                  separating wall still touches.
 *
 * @param[out]      areCompatible_out
 *                  True when the clouds overlap or form adjacent finite
 *                  extensions.
 *
 * @param[out]      isSeparatedByCrossingWall_out
 *                  True when a mapped wall separates the fragments; the
 *                  caller shall not join them by any other test.
 *
 * @return          UTILS_STATUS_SUCCESS.
 */
[[nodiscard]] UtilsStatus finiteWallExtentsAreCompatible(
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_firstCloud_in,
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_secondCloud_in,
    const Eigen::Vector4d                              &commonEquation_world_in,
    const double                                        observerSide_m_in,
    const std::vector<geometric::Plane *>              &mappedPlanes_in,
    const double                                        maximumInPlaneGap_m_in,
    const double minimumOrthogonalOverlap_m_in,
    const double touchingTolerance_m_in,
    bool        &areCompatible_out,
    bool        &isSeparatedByCrossingWall_out);

/*!
 * @brief           Applies the OPPOSITE_FACE test of Utils::findMatchingPlane()
 *                  to one candidate, in this order: candidate valid,
 *                  normals parallel, equations and observed centroid valid,
 *                  face separation within the maximum thickness, both
 *                  stamped observation origins present, each origin at least
 *                  0.10 m from both planes, origins on opposite sides of both
 *                  planes, and footprints on ground-anchored axes overlapping
 *                  by the minimum ratio.
 *
 * @param[in]       observedPlane_in
 *                  Observed plane evidence; its observer side is the stamped
 *                  observation origin.
 *
 * @param[in]       p_mappedPlane_in
 *                  Candidate plane, non-owning; may be null.
 *
 * @param[in]       matchSettings_in
 *                  Angle, maximum thickness, minimum overlap ratio and
 *                  ground normal.
 *
 * @param[out]      candidateVerdict_out
 *                  PLAUSIBLE or the first failed check, with its
 *                  measurement.
 *
 * @return          UTILS_STATUS_SUCCESS.
 */
[[nodiscard]] UtilsStatus
    checkOppositeFacePair(const PlaneEvidence      &observedPlane_in,
                          geometric::Plane         *p_mappedPlane_in,
                          const PlaneMatchSettings &matchSettings_in,
                          PlaneMatchVerdict        &candidateVerdict_out);

/*!
 * @brief           Returns the room itself when it is non-null, not bad and in
 *                  the active room set; otherwise null.
 *
 * @param[in]       activeRoomSet_in
 *                  Rooms currently live in the map, non-owning.
 *
 * @param[in]       p_room_in
 *                  Room to resolve, non-owning; may be null.
 *
 * @param[out]      p_liveRoom_out
 *                  The live room, or null.
 *
 * @return          UTILS_STATUS_SUCCESS.
 */
[[nodiscard]] UtilsStatus resolveLiveRoomHandle(
    const std::unordered_set<semantic::Room *> &activeRoomSet_in,
    semantic::Room                             *p_room_in,
    semantic::Room                            *&p_liveRoom_out);

/*!
 * @brief           Tells whether a room lies on the proven far side of a
 *                  passage: its centroid more than 0.20 m behind the passage
 *                  centroid along the known-side direction.
 *
 * @param[in]       knownSide_in
 *                  Known-side provenance of the passage.
 *
 * @param[in]       passageCentroid_world_m_in
 *                  Passage centroid in the world frame, metres.
 *
 * @param[in]       p_room_in
 *                  Room to test, non-owning; may be null.
 *
 * @param[out]      isFarSide_out
 *                  True when the room is on the proven far side.
 *
 * @return          UTILS_STATUS_SUCCESS.
 */
[[nodiscard]] UtilsStatus isRoomOnProvenFarSide(
    const semantic::Passage::KnownSideProvenance &knownSide_in,
    const Eigen::Vector3d                        &passageCentroid_world_m_in,
    semantic::Room                               *p_room_in,
    bool                                         &isFarSide_out);

/*!
 * @brief           Fuses the attributes of a duplicate passage into the
 *                  retained passage.
 *
 * @param[in,out]   p_retainedPassage_inout
 *                  Passage that is kept, non-owning.
 *
 * @param[in]       p_candidatePassage_in
 *                  Duplicate passage whose attributes are fused, non-owning.
 *
 * @param[in]       retainedEquation_in
 *                  Plane equation of the retained passage.
 *
 * @param[in]       retainedCentroid_world_m_in
 *                  Retained passage centroid in the world frame, metres.
 *
 * @param[in]       candidateCentroid_world_m_in
 *                  Duplicate passage centroid in the world frame, metres.
 *
 * @return          UTILS_STATUS_SUCCESS.
 */
[[nodiscard]] UtilsStatus fuseDuplicatePassageAttributes(
    semantic::Passage     *p_retainedPassage_inout,
    semantic::Passage     *p_candidatePassage_in,
    const Eigen::Vector4d &retainedEquation_in,
    const Eigen::Vector3d &retainedCentroid_world_m_in,
    const Eigen::Vector3d &candidateCentroid_world_m_in);

/*!
 * @brief           Runs the OPPOSITE_FACE pass of Utils::findMatchingPlane()
 *                  over all mapped planes.
 *
 * @param[in]       mappedPlanes_in
 *                  Mapped candidate planes, non-owning.
 *
 * @param[in]       observedPlane_in
 *                  Observed plane evidence.
 *
 * @param[in]       matchSettings_in
 *                  Match thresholds and ground normal.
 *
 * @param[out]      matchedPlaneId_out
 *                  Id of the matched plane, unchanged when none matches.
 *
 * @param[out]      p_candidateVerdicts_out
 *                  Per-candidate verdicts; may be null.
 *
 * @return          UTILS_STATUS_SUCCESS.
 */
[[nodiscard]] UtilsStatus findOppositeFaceMatch(
    const std::vector<geometric::Plane *> &mappedPlanes_in,
    const PlaneEvidence                   &observedPlane_in,
    const PlaneMatchSettings              &matchSettings_in,
    int                                   &matchedPlaneId_out,
    std::vector<PlaneMatchVerdict>        *p_candidateVerdicts_out);

/*!
 * @brief           Checks that the observer and the mapped plane's
 *                  observing cameras are on the same side of the plane.
 *
 * @param[in]       p_mappedPlane_in
 *                  Candidate plane, non-owning.
 *
 * @param[in]       mappedEquation_in
 *                  Candidate plane equation.
 *
 * @param[in]       p_cameraCentre_world_m_in
 *                  Observer camera centre in the world frame, metres.
 *
 * @param[in]       hasCameraCentre_in
 *                  True when the camera centre is valid.
 *
 * @param[in]       p_observingCameras_in
 *                  Observation-side snapshot of the plane; may be null.
 *
 * @param[in]       minimumReliableSideDistance_m_in
 *                  Smallest distance, metres, at which a side is reliable.
 *
 * @param[out]      isOppositeSide_out
 *                  True when the candidate must be rejected.
 *
 * @param[out]      observerSide_m_out
 *                  Signed observer distance to the plane, metres.
 *
 * @return          UTILS_STATUS_SUCCESS.
 */
[[nodiscard]] UtilsStatus checkObserverSideAgreement(
    geometric::Plane                                *p_mappedPlane_in,
    const Eigen::Vector4d                           &mappedEquation_in,
    const Eigen::Vector3d                           *p_cameraCentre_world_m_in,
    bool                                             hasCameraCentre_in,
    const geometric::Plane::ObservationSideSnapshot *p_observingCameras_in,
    double  minimumReliableSideDistance_m_in,
    bool   &isOppositeSide_out,
    double &observerSide_m_out);

/*!
 * @brief           Samples the observed cloud and counts points within the
 *                  maximum distance of the mapped cloud.
 *
 * @param[in]       p_observedCloud_in
 *                  Observed plane cloud.
 *
 * @param[in]       p_mappedCloud_in
 *                  Mapped plane cloud.
 *
 * @param[in]       maximumSampleCount_in
 *                  Largest number of observed points sampled.
 *
 * @param[in]       maximumFiniteCloudDistance_in
 *                  Overlap distance, metres.
 *
 * @param[out]      sampledPointCount_out
 *                  Number of sampled points.
 *
 * @param[out]      overlappingPointCount_out
 *                  Number of sampled points within the overlap distance.
 *
 * @param[out]      minimumCloudDistance_out
 *                  Smallest sampled point distance, metres.
 *
 * @return          UTILS_STATUS_SUCCESS.
 */
[[nodiscard]] UtilsStatus measureFiniteCloudOverlap(
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_observedCloud_in,
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_mappedCloud_in,
    std::size_t                                         maximumSampleCount_in,
    double       maximumFiniteCloudDistance_in,
    std::size_t &sampledPointCount_out,
    std::size_t &overlappingPointCount_out,
    double      &minimumCloudDistance_out);

} /* namespace utils */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* UTILS_UTILS_PRIVATE_FUNCTIONS_H */

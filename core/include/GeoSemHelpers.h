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
 * @file            GeoSemHelpers.h
 *
 * @brief           Declares GeoSemHelpers, helper functions that create and
 *                  update semantic map entities (markers, passages, rooms,
 *                  ground planes) from geometry.
 */

#ifndef GEOSEMHELPERS_H
#define GEOSEMHELPERS_H

#include "Atlas.h"
#include "GeoSemHelpersStatus.h"

#include <Eigen/Core>
#include <iomanip>
#include <optional>
#include <sstream>

namespace vs_graphs
{
namespace core
{
/*!
 * @brief        Static helpers that create and update the planes, markers,
 *               passages, rooms and floors of the semantic map in the atlas.
 */
class GeoSemHelpers
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief       Creates a new plane object to be added to the map
     *
     * @param[in,out] p_atlas_inout
     *                Atlas which owns the active semantic map.
     *
     * @param       p_keyFrame_inout
     *              The address of the current keyframe
     *
     * @param       estimatedPlane_in
     *              The estimated plane
     *
     * @param       p_planeCloud_in
     *              The plane point cloud
     *
     * @param[out]  p_mapPlane_out
     *              The new map plane, or nullptr when none was created
     *
     * @param       semanticType_in
     *              The semantic type of the plane observation
     *
     * @param       confidence_in
     *              The confidence of the plane observation
     */
    [[nodiscard]] static GeoSemHelpersStatus createMapPlane(
        Atlas                                          *p_atlas_inout,
        vs_graphs::core::KeyFrame                      *p_keyFrame_inout,
        const g2o::Plane3D                              estimatedPlane_in,
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr   p_planeCloud_in,
        vs_graphs::core::geometric::Plane             *&p_mapPlane_out,
        vs_graphs::core::geometric::Plane::PlaneVariant semanticType_in =
            vs_graphs::core::geometric::Plane::PlaneVariant::UNDEFINED,
        double confidence_in = 1.0);

    /*!
     * @brief       Updates the map plane
     *
     * @param[in] p_atlas_in
     *              The current map in Atlas
     *
     * @param       p_keyFrame_inout
     *              The current keyframe
     *
     * @param       estimatedPlane_in
     *              The estimated plane
     *
     * @param       p_planeCloud_in
     *              The plane point cloud
     *
     * @param       planeId_in
     *              The plane id
     *
     * @param       semanticType_in
     *              The semantic type of the plane observation
     *
     * @param       confidence_in
     *              The confidence of the plane observation
     */
    [[nodiscard]] static GeoSemHelpersStatus updateMapPlane(
        Atlas                                          *p_atlas_in,
        vs_graphs::core::KeyFrame                      *p_keyFrame_inout,
        const g2o::Plane3D                              estimatedPlane_in,
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr         p_planeCloud_in,
        int                                             planeId_in,
        vs_graphs::core::geometric::Plane::PlaneVariant semanticType_in =
            vs_graphs::core::geometric::Plane::PlaneVariant::UNDEFINED,
        double confidence_in = 1.0);

    /*!
     * @brief       Checks to see if the marker is attached to a doorway or not
     *              (e.g., a window) and returns the name of it if exists (only
     *              valid for doors)
     *
     * @param       markerId_in
     *              The id of the marker
     *
     * @param       envRooms_in
     *              Rooms of the environment; each room's meta-marker id is
     *              compared with the marker
     *
     * @param       doorwayMatch_out
     *              Whether a match was found, and the name of the matched room
     *
     * @return      GEO_SEM_HELPERS_STATUS_SUCCESS
     */
    [[nodiscard]] static GeoSemHelpersStatus checkIfMarkerIsDoorway(
        const int                                     &markerId_in,
        std::vector<vs_graphs::core::semantic::Room *> envRooms_in,
        std::pair<bool, std::string>                  &doorwayMatch_out);

    /*!
     * @brief       Uses the detected markers to detect and map semantic
     *              objects, e.g., planes and doors
     *
     * @param[in]     p_atlas_in
     *              The current map in Atlas
     *
     * @param       p_keyFrame_in
     *              The current keyframe in which the detection took place
     *
     * @param       envRooms_in
     *              The list of rooms in the environment
     */
    [[nodiscard]] static GeoSemHelpersStatus markerSemanticAnalysis(
        Atlas                                         *p_atlas_in,
        vs_graphs::core::KeyFrame                     *p_keyFrame_in,
        std::vector<vs_graphs::core::semantic::Room *> envRooms_in);

    /*!
     * @brief       Creates a new marker object to be added to the map
     *
     * @param[in,out] p_atlas_inout
     *              The current map in Atlas
     *
     * @param       p_keyFrame_inout
     *              The address of the current keyframe
     *
     * @param       p_visitedMarker_in
     *              The address of the visited marker
     *
     * @param[out]  p_mapMarker_out
     *              The new map marker
     */
    [[nodiscard]] static GeoSemHelpersStatus
        createMapMarker(Atlas                  *p_atlas_inout,
                        KeyFrame               *p_keyFrame_inout,
                        const semantic::Marker *p_visitedMarker_in,
                        semantic::Marker      *&p_mapMarker_out);

    /*!
     * @brief       Creates a new passage object to be added to the map
     *
     * @param[in,out] p_atlas_inout
     *              The current map in Atlas
     *
     * @param[in]   p_doorPlane_in
     *              Optional semantic plane representing a closed door.
     *
     * @param[in]   p_wallPlane_in
     *              Plane representing the wall containing the opening.
     *
     * @param[in]   isOpenPassage_in
     *              True when connected free-space evidence confirms that the
     *              opening is traversable.
     *
     * @param[in]   passageCentroid_world_m_in
     *              Open-passage centroid in the active map frame, in metres.
     */
    [[nodiscard]] static GeoSemHelpersStatus createMapPassage(
        vs_graphs::core::Atlas            *p_atlas_inout,
        vs_graphs::core::geometric::Plane *p_doorPlane_in,
        vs_graphs::core::geometric::Plane *p_wallPlane_in,
        bool                               isOpenPassage_in = false,
        Eigen::Vector3d passageCentroid_world_m_in = Eigen::Vector3d::Zero());

    /*!
     * @brief       Creates a blank room object (undefined variant) to be added
     *              to the map
     *
     * @param[in,out] p_atlas_inout
     *              The current map in Atlas
     *
     * @param[out]  p_blankRoomCandidate_out
     *              The new room, or nullptr when the Atlas is null or the
     *              map already holds one room more than its passable
     *              passages
     *
     * @param       centroid_in
     *              The centroid of the room (optional)
     *
     * @param[in]   stableRoomId_in
     *              Identity to restore after a tracking-loss reset (not
     *              limited by the passage count); when absent a new identity
     *              is reserved from the Atlas
     */
    [[nodiscard]] static GeoSemHelpersStatus createBlankRoomCandidate(
        Atlas                            *p_atlas_inout,
        vs_graphs::core::semantic::Room *&p_blankRoomCandidate_out,
        Eigen::Vector3d                   centroid_in = Eigen::Vector3d::Zero(),
        std::optional<int>                stableRoomId_in = std::nullopt);

    /*!
     * @brief       Chooses a ground plane from the Atlas to be associated with
     *              the room
     *
     * @param[in] p_atlas_in
     *              The current map in Atlas
     *
     * @param       p_givenRoom_inout
     *              The address of the detected room
     */
    [[nodiscard]] static GeoSemHelpersStatus associateGroundPlaneToRoom(
        Atlas                           *p_atlas_in,
        vs_graphs::core::semantic::Room *p_givenRoom_inout);

    /*!
     * @brief       Counts the number of points in the ground plane that are
     *              within the walls of the room
     *
     * @param       roomWalls_in
     *              The vector of walls detected in the room
     *
     * @param       p_groundPlane_in
     *              The ground plane associated with the room
     *
     * @param[out]  groundPlanePoints_out
     *              Number of ground-plane points within the walls
     */
    [[nodiscard]] static GeoSemHelpersStatus countGroundPlanePointsWithinWalls(
        std::vector<vs_graphs::core::geometric::Plane *> &roomWalls_in,
        vs_graphs::core::geometric::Plane                *p_groundPlane_in,
        size_t &groundPlanePoints_out);

    /*!
     * @brief       Creates a new floor object to be added to the map
     *
     * @param[in,out] p_atlas_inout
     *              The current map in Atlas
     *
     * @param[in]   stableFloorId_in
     *              Identity to restore after a tracking-loss reset; when
     *              absent a new identity is reserved from the Atlas
     */
    [[nodiscard]] static GeoSemHelpersStatus
        createMapFloor(vs_graphs::core::Atlas *p_atlas_inout,
                       std::optional<int>      stableFloorId_in = std::nullopt);

    /*!
     * @brief       Refits a mapped plane equation from its accumulated global
     *              point cloud.
     *
     * @param[in,out] p_plane_inout
     *              Mapped plane which will be refitted.
     *
     * @param[out]  wasPlaneRefit_out
     *              True when the plane equation was refitted from the cloud.
     */
    [[nodiscard]] static GeoSemHelpersStatus refitMappedPlaneFromCloud(
        vs_graphs::core::geometric::Plane *p_plane_inout,
        bool                              &wasPlaneRefit_out);
};
} // namespace core
} // namespace vs_graphs

#endif // GEOSEMHELPERS_H

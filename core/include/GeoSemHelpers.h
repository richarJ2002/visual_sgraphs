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

#ifndef GEOSEMHELPERS_H
#define GEOSEMHELPERS_H

#include "Atlas.h"
#include "Utils.h"

#include <Eigen/Core>
#include <iomanip>
#include <sstream>

namespace ORB_SLAM3
{
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
     * @param       pKF
     *              The address of the current keyframe
     *
     * @param       estimatedPlane
     *              The estimated plane
     *
     * @param       planeCloud
     *              The plane point cloud
     *
     * @param       semanticType
     *              The semantic type of the plane observation
     *
     * @param       confidence
     *              The confidence of the plane observation
     */
    static ORB_SLAM3::Plane *
        createMapPlane(Atlas               *mpAtlas,
                       ORB_SLAM3::KeyFrame *pKF,
                       const g2o::Plane3D   estimatedPlane,
                       const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud,
                       ORB_SLAM3::Plane::planeVariant semanticType =
                           ORB_SLAM3::Plane::planeVariant::UNDEFINED,
                       double confidence = 1.0);

    /*!
     * @brief       Updates the map plane
     *
     * @param       mpAtlas
     *              The current map in Atlas
     *
     * @param       pKF
     *              The current keyframe
     *
     * @param       estimatedPlane
     *              The estimated plane
     *
     * @param       planeCloud
     *              The plane point cloud
     *
     * @param       planeId
     *              The plane id
     *
     * @param       semanticType
     *              The semantic type of the plane observation
     *
     * @param       confidence
     *              The confidence of the plane observation
     */
    static void
        updateMapPlane(Atlas                                  *mpAtlas,
                       ORB_SLAM3::KeyFrame                    *pKF,
                       const g2o::Plane3D                      estimatedPlane,
                       pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud,
                       int                                     planeId,
                       ORB_SLAM3::Plane::planeVariant          semanticType =
                           ORB_SLAM3::Plane::planeVariant::UNDEFINED,
                       double confidence = 1.0);

    /*!
     * @brief       Checks to see if the marker is attached to a doorway or not
     *              (e.g., a window) and returns the name of it if exists (only
     *              valid for doors)
     *
     * @param       markerId
     *              The id of the marker
     *
     * @param       envDoorways
     *              The list of doorways in the environment
     */
    static std::pair<bool, std::string>
        checkIfMarkerIsDoorway(const int                     &markerId,
                               std::vector<ORB_SLAM3::Room *> envRooms);

    /*!
     * @brief       Uses the detected markers to detect and map semantic
     *              objects, e.g., planes and doors
     *
     * @param       mpAtlas
     *              The current map in Atlas
     *
     * @param       pKF
     *              The current keyframe in which the detection took place
     *
     * @param       envRooms
     *              The list of rooms in the environment
     */
    static void markerSemanticAnalysis(Atlas                         *mpAtlas,
                                       ORB_SLAM3::KeyFrame           *pKF,
                                       std::vector<ORB_SLAM3::Room *> envRooms);

    /*!
     * @brief       Creates a new marker object to be added to the map
     *
     * @param       mpAtlas
     *              The current map in Atlas
     *
     * @param       pKF
     *              The address of the current keyframe
     *
     * @param       visitedMarker
     *              The address of the visited marker
     */
    static Marker *createMapMarker(Atlas        *mpAtlas,
                                   KeyFrame     *pKF,
                                   const Marker *visitedMarker);

    /*!
     * @brief       Creates a new passage object to be added to the map
     *
     * @param       mpAtlas
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
     * @param[in]   passageCentroid_World_m_in
     *              Open-passage centroid in the active map frame, in metres.
     */
    static void createMapPassage(
        ORB_SLAM3::Atlas *p_atlas_inout,
        ORB_SLAM3::Plane *p_doorPlane_in,
        ORB_SLAM3::Plane *p_wallPlane_in,
        bool              isOpenPassage_in           = false,
        Eigen::Vector3d   passageCentroid_World_m_in = Eigen::Vector3d::Zero());

    /*!
     * @brief       Creates a blank room object (undefined variant) to be added
     *              to the map
     *
     * @param       mpAtlas
     *              The current map in Atlas
     *
     * @param       centroid
     *              The centroid of the room (optional)
     */
    static ORB_SLAM3::Room *createBlankRoomCandidate(
        Atlas          *mpAtlas,
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero());

    /*!
     * @brief       Chooses a ground plane from the Atlas to be associated with
     *              the room
     *
     * @param       mpAtlas
     *              The current map in Atlas
     *
     * @param       givenRoom
     *              The address of the detected room
     */
    static void associateGroundPlaneToRoom(Atlas           *mpAtlas,
                                           ORB_SLAM3::Room *givenRoom);

    /*!
     * @brief       Counts the number of points in the ground plane that are
     *              within the walls of the room
     *
     * @param       roomWalls
     *              The vector of walls detected in the room
     *
     * @param       groundPlane
     *              The ground plane associated with the room
     */
    static size_t countGroundPlanePointsWithinWalls(
        std::vector<ORB_SLAM3::Plane *> &roomWalls,
        ORB_SLAM3::Plane                *groundPlane);

    /*!
     * @brief       Creates a new floor object to be added to the map
     *
     * @param       mpAtlas
     *              The current map in Atlas
     */
    static void createMapFloor(ORB_SLAM3::Atlas *mpAtlas);

    /*!
     * @brief       Refits a mapped plane equation from its accumulated global
     *              point cloud.
     *
     * @param[in]   plane
     *              Mapped plane which will be refitted.
     */
    static bool refitMappedPlaneFromCloud(ORB_SLAM3::Plane *plane);
};
} // namespace ORB_SLAM3

#endif // GEOSEMHELPERS_H

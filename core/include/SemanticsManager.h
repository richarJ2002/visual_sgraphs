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

#ifndef SEMANTICSMANAGER_H
#define SEMANTICSMANAGER_H

#include "Atlas.h"
#include "GeoSemHelpers.h"
#include "Utils.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_map>

namespace ORB_SLAM3
{
class Atlas;

class SemanticsManager
{
  private:
    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       This member contains the address of semantic map.
     */
    Atlas *mpAtlas;

    /*!
     * @brief       TODO
     */
    std::mutex mMutexNewRooms;

    /*!
     * @brief       The transformation matrix from ground plane to horizontal.
     */
    Eigen::Matrix4f mPlanePoseMat;

    /*!
     * @brief       The main Run() function runs every runInterval seconds.
     */
    const uint8_t runInterval = 3;

    /*!
     * @brief       Member which contains the system parameters.
     */
    SystemParams *sysParams;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /* ---------------------------------------------------------------------- *
     * PUBLIC METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Constructor which stores the pointer to the map in the
     *              member mpAtlas and gets the systems parameter.
     *
     * @param[in]   pAtlas
     *              Pointer to map.
     */
    explicit SemanticsManager(Atlas *pAtlas);

    /*!
     * @brief       Gets the latest skeleton cluster acquired from voxblox.
     */
    std::vector<std::vector<Eigen::Vector3d>> getLatestSkeletonCluster(void);

    /*!
     * @brief       Detects open passages where a connected Voxblox skeleton
     *              edge crosses a finite mapped wall.
     *
     * @param[in]   wallPlanes
     *              Confirmed wall planes available in the current map.
     */
    void detectOpenPassagesFromSkeletonEdges(
        const std::vector<ORB_SLAM3::Plane *> &wallPlanes);

    /*!
     * @brief       Detects doors and doorways based on the detected planes and
     *              the mapped environment.
     *
     * @param       pAtlas
     *              The Atlas containing the mapped environment
     */
    void detectDoorsAndDoorways(ORB_SLAM3::Atlas *pAtlas);

    /*!
     * @brief       Gets the latest detected room candidates from GNN-based room
     *              detection.
     */
    std::vector<ORB_SLAM3::Room *> getLatestGNNRoomCandidates(void);

    /*!
     * @brief       Filters the wall planes to remove heavily tilted walls. Does
     *              this by comparing the transpose to the ground plane. If the
     *              plane normal is horizontal to the ground then it is left
     *              alone. If the wall plane has a tile greater than the
     *              sem_seg.max_tilt_wall parameter set in System Params, its
     *              semantics are reset.
     */
    void filterWallPlanes(void);

    /*!
     * @brief       Filters the planes which are assoicated with ground semantic
     *              and resets the planes semantics if it is not horizontal
     *              enough or if its height is outside tolerances with the main
     *              ground plane. Removes points that are too far from the
     *              plane.
     *
     * @param       groundPlane
     *              The main ground plane that is the reference
     */
    void filterGroundPlanes(Plane *groundPlane);

    /*!
     * @brief       Updates the passages in the map based on the detected doors
     *              and doorways.
     *
     * @param       pAtlas
     *              The Atlas containing the mapped environment
     */
    void updatePassages(ORB_SLAM3::Atlas *pAtlas);

    /*!
     * @brief       Transforms the plane equation to the ground reference
     *              defined by mPlanePoseMat.
     *
     * @param       planeEq
     *              The plane equation
     *
     * @return      The transformed plane equation
     */
    Eigen::Vector3f
        transformPlaneEqToGroundReference(const Eigen::Vector4d &planeEq);

    /*!
     * @brief       Gets the median height of a ground plane after
     *              transformation to referece by mPlanePoseMat.
     *
     * @param       groundPlane
     *              The ground plane
     *
     * @return      The median height of the ground plane
     */
    float computeGroundPlaneHeight(Plane *groundPlane);

    /*!
     * @brief       Computes the transformation matrix from the ground plane to
     *              the horizontal (y-inverted).
     *
     * @param       plane
     *              The plane
     *
     * @return      the transformation matrix
     */
    Eigen::Matrix4f computePlaneToHorizontal(const Plane *plane);

    /*!
     * @brief       Checks for the existing of a room with particular walls
     *              close to a cluster. It returns the address of the existing
     *              room if found, otherwise returns nullptr.
     *
     * @param       clusterCentroid
     *              The centroid of the cluster
     *
     * @param       wallList
     *              The list of walls to be checked
     */
    ORB_SLAM3::Room *
        associateRooms(const Eigen::Vector3d                  clusterCentroid,
                       const std::vector<ORB_SLAM3::Plane *> &wallList);

    /*!
     * @brief       Ensures every valid WALL plane belongs to at least one  room
     *              or provisional structural element.
     */
    void associateAllWallsToRooms(void);

    /*!
     * @brief       Consolidates redundant single-wall provisional structural
     *              elements into a room supported by a free-space cluster.
     *
     * @param       selectedRoom
     *              The cluster-backed room which has absorbed the walls
     */
    void consolidateProvisionalRooms(ORB_SLAM3::Room *selectedRoom);

    /*!
     * @brief       Associates each passage with the closest room on either
     *              side of its supporting wall.
     */
    void associatePassagesToRooms(void);

    /*!
     * @brief       Re-associates rooms based on fixed time intervals to
     *              avoid duplicates.
     */
    void reAssociateRooms(void);

    /*!
     * @brief       Processes the latest skeleton cluster to detect rooms based
     *              on free space clustering.
     */
    void detectRoom_FreeSpaceCluster(void);

    /*!
     * @brief       Gets the rooms detected by the GNN module.
     */
    void detectRoom_GNN(void);

    /*!
     * @brief       Gets the updated floors containing rooms and corridors.
     */
    void getUpdatedFloors(void);

    /*!
     * @brief       Method which runs the thread of the segmantic manager.
     */
    void Run(void);
};
} // namespace ORB_SLAM3

#endif // SEMANTICSEG_H
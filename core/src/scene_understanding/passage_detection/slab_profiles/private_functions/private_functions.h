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
 * @brief           Declares the helpers of the wall slab profile part of the
 *                  passage detection stage. Package-internal: not included by
 *                  any public header.
 */

#ifndef VS_GRAPHS_CORE_SRC_SLAB_PROFILES_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_SRC_SLAB_PROFILES_PRIVATE_FUNCTIONS_H

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "configuration/types/types.h"
#include "scene_understanding/semantics_manager/objects/SemanticsManagerStatusEnum.h"
#include "scene_understanding/semantics_manager/objects/WallGapStruct.h"
#include "scene_understanding/semantics_manager/objects/WallSlabProfileStruct.h"
#include "scene_understanding/semantics_manager/objects/WallSlabQueryStruct.h"

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Measures the openings through one wall from its Voxblox
 *                  free-space slab: FREE door-band cells seed 4-connected
 *                  components, each end is refined at the TSDF zero crossing,
 *                  and only openings of door size are kept.
 *
 * @param[in]       wallProfile_in
 *                  Slab profile of the wall, world frame, metres.
 *
 * @param[in]       passageParameters_in
 *                  Door band, size limits and bounded-row fraction.
 *
 * @param[out]      wallGaps_out
 *                  Accepted openings; empty when there is none.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus measureWallGap(
    const WallSlabProfile                               &wallProfile_in,
    const types::SystemParams::SemSeg::PassageDetection &passageParameters_in,
    std::vector<WallGap>                                &wallGaps_out);

/*!
 * @brief           Refines a slab gap at the wall's own points: per door-band
 *                  row the point-free run around the coarse gap centre gives
 *                  the two wall ends, the medians give width and centre, and
 *                  the lowest point above the band inside them the lintel.
 *                  The along extent stays the coarse slab aperture; the gap
 *                  is left unchanged when too few rows are bounded.
 *
 * @param[in]       wallQuery_in
 *                  Slab query of the wall, world frame, metres.
 *
 * @param[in]       wallCloud_in
 *                  Support points of the wall, world frame, metres.
 *
 * @param[in]       passageParameters_in
 *                  Door band and bounded-row fraction.
 *
 * @param[in,out]   wallGap_inout
 *                  Coarse gap from measureWallGap(); refined in place.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus refineWallGapFromPoints(
    const WallSlabQuery                                 &wallQuery_in,
    const pcl::PointCloud<pcl::PointXYZRGBA>            &wallCloud_in,
    const types::SystemParams::SemSeg::PassageDetection &passageParameters_in,
    WallGap                                             &wallGap_inout);

/*!
 * @brief           Middle value of a list: the element at index size / 2 once
 *                  the list is partly sorted (the upper middle for an even
 *                  count).
 *
 * @param[in]       values_in
 *                  Values; must not be empty. A copy is reordered, the input
 *                  is left as it is.
 *
 * @return          The middle value.
 */
[[nodiscard]] double computeMedianValue(const std::vector<double> &values_in);

/*!
 * @brief           Zero crossing between a free slab cell and its OCCUPIED
 *                  neighbour, as an offset from the free cell centre towards
 *                  the neighbour; half a cell when the neighbour is unknown,
 *                  missing or the distances are unusable.
 *
 * @param[in]       wallProfile_in
 *                  Slab profile of the wall; its query gives the cell size.
 *
 * @param[in]       freeIndex_in
 *                  Row-major index of the free cell.
 *
 * @param[in]       neighbourIndex_in
 *                  Row-major index of the neighbour; ignored without one.
 *
 * @param[in]       hasNeighbour_in
 *                  False when the free cell lies on the slab border.
 *
 * @return          Offset in metres, between 0 and one cell.
 */
[[nodiscard]] double
    computeSurfaceOffset_m(const WallSlabProfile &wallProfile_in,
                           std::size_t            freeIndex_in,
                           std::size_t            neighbourIndex_in,
                           bool                   hasNeighbour_in);

/*!
 * @brief           One flood-fill step: labels a FREE, not yet labelled slab
 *                  cell with the current component and queues it.
 *
 * @param[in]       wallProfile_in
 *                  Slab profile of the wall.
 *
 * @param[in]       nextIndex_in
 *                  Row-major index of the cell to visit.
 *
 * @param[in]       componentLabel_in
 *                  Label of the component being filled (from 1).
 *
 * @param[in,out]   componentLabels_inout
 *                  Label per cell; 0 = not yet visited.
 *
 * @param[in,out]   openCells_inout
 *                  Cells still to expand.
 */
void visitFreeSlabCell(const WallSlabProfile    &wallProfile_in,
                       std::size_t               nextIndex_in,
                       std::size_t               componentLabel_in,
                       std::vector<std::size_t> &componentLabels_inout,
                       std::vector<std::size_t> &openCells_inout);

/*!
 * @brief           Distance between two points after removing the part along
 *                  the floor normal.
 *
 * @param[in]       first_world_m_in
 *                  First point, world frame, metres.
 *
 * @param[in]       second_world_m_in
 *                  Second point, world frame, metres.
 *
 * @param[in]       groundNormal_world_in
 *                  Unit floor normal, world frame; zero when no floor is
 *                  known (the full distance is returned).
 *
 * @return          Horizontal distance in metres.
 */
[[nodiscard]] double
    computeHorizontalDistance_m(const Eigen::Vector3d &first_world_m_in,
                                const Eigen::Vector3d &second_world_m_in,
                                const Eigen::Vector3d &groundNormal_world_in);

/*!
 * @brief           Counts the wall support points inside an along/height
 *                  window of the slab, only where the slab is not FREE: the
 *                  TSDF is integrated at the poses of the moment, so after a
 *                  SLAM correction current wall points can land in free space
 *                  carved at a stale pose and must not close the door.
 *
 * @param[in]       p_wallCloud_in
 *                  Support points of the wall, world frame; may be null
 *                  (no points).
 *
 * @param[in]       wallProfile_in
 *                  Slab profile of the wall.
 *
 * @param[in]       doorBandMinimum_m_in
 *                  Lowest counted height above the floor, metres.
 *
 * @param[in]       doorBandMaximum_m_in
 *                  Highest counted height above the floor, metres.
 *
 * @param[in]       alongStart_m_in
 *                  Start of the window along the wall, metres.
 *
 * @param[in]       alongEnd_m_in
 *                  End of the window along the wall, metres.
 *
 * @return          Number of counted points.
 */
[[nodiscard]] std::size_t countApertureWallPoints(
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_wallCloud_in,
    const WallSlabProfile                              &wallProfile_in,
    double                                              doorBandMinimum_m_in,
    double                                              doorBandMaximum_m_in,
    double                                              alongStart_m_in,
    double                                              alongEnd_m_in);

/*!
 * @brief           Fraction of the door-band cells inside an along window
 *                  that have any TSDF observation.
 *
 * @param[in]       wallProfile_in
 *                  Slab profile of the wall.
 *
 * @param[in]       doorBandMinimum_m_in
 *                  Lowest cell-centre height above the floor, metres.
 *
 * @param[in]       doorBandMaximum_m_in
 *                  Highest cell-centre height above the floor, metres.
 *
 * @param[in]       alongStart_m_in
 *                  Start of the window along the wall, metres.
 *
 * @param[in]       alongEnd_m_in
 *                  End of the window along the wall, metres.
 *
 * @param[out]      freeCellCount_out
 *                  Number of FREE cells in the window.
 *
 * @param[out]      bandCellCount_out
 *                  Number of cells in the window.
 *
 * @return          Observed fraction; 0 when the window holds no cell.
 */
[[nodiscard]] double
    computeObservedApertureFraction(const WallSlabProfile &wallProfile_in,
                                    double                 doorBandMinimum_m_in,
                                    double                 doorBandMaximum_m_in,
                                    double                 alongStart_m_in,
                                    double                 alongEnd_m_in,
                                    std::size_t           &freeCellCount_out,
                                    std::size_t           &bandCellCount_out);

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SLAB_PROFILES_PRIVATE_FUNCTIONS_H */

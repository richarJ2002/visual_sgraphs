/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            private_functions.h
 *
 * @brief           Declares the helpers of the SemanticsManager graph merging
 *                  methods. Package-internal: not included by any public
 *                  header. Every helper expects the caller to hold both graph
 *                  locks exclusive.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_GRAPH_MERGING_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_GRAPH_MERGING_PRIVATE_FUNCTIONS_H

#include <Eigen/Core>
#include <utility>
#include <vector>

#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"
#include "scene_understanding/semantics_manager/objects/AbsorbedSceneGraphContentsStruct.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Lists the passages, detected rooms, candidate rooms and
 *                  storeys of the absorbed map's scene graph.
 *
 * @param[in]       p_semanticsManager_in
 *                  Manager that lists the graph; borrowed.
 *
 * @param[in]       absorbedMapId_in
 *                  Id of the absorbed map.
 *
 * @param[out]      absorbedContents_out
 *                  Receives the lists; borrowed nodes.
 */
void collectAbsorbedSceneGraph(
    SemanticsManager           *p_semanticsManager_in,
    unsigned long               absorbedMapId_in,
    AbsorbedSceneGraphContents &absorbedContents_out);

/*!
 * @brief           Moves the absorbed map's passages into the surviving map.
 *                  A passage that duplicates one of the surviving map's
 *                  (paired by geometry) is merged into it and erased; the
 *                  rooms that listed the duplicate list the retained passage.
 *
 * @param[in,out]   p_semanticsManager_inout
 *                  Manager that moves and erases the passages; borrowed.
 *
 * @param[in,out]   p_currentMap_inout
 *                  Surviving map.
 *
 * @param[in,out]   p_mergeMap_inout
 *                  Absorbed map.
 *
 * @param[in]       passages_in
 *                  Passages of the absorbed map.
 *
 * @param[in]       detectedRooms_in
 *                  Detected rooms of the absorbed map.
 *
 * @param[in]       markerBasedRooms_in
 *                  Candidate rooms of the absorbed map.
 *
 * @param[in]       configuration_in
 *                  Merge pairing thresholds.
 */
void transferPassages(SemanticsManager *p_semanticsManager_inout,
                      map::Map         *p_currentMap_inout,
                      map::Map         *p_mergeMap_inout,
                      const std::vector<semantic::Passage *> &passages_in,
                      const std::vector<semantic::Room *>    &detectedRooms_in,
                      const std::vector<semantic::Room *>  &markerBasedRooms_in,
                      const semantic::SemanticVerifyConfig &configuration_in);

/*!
 * @brief           Moves the absorbed map's detected and candidate rooms into
 *                  the surviving map.
 *
 * @param[in,out]   p_semanticsManager_inout
 *                  Manager that moves the rooms; borrowed.
 *
 * @param[in,out]   p_currentMap_inout
 *                  Surviving map.
 *
 * @param[in,out]   p_mergeMap_inout
 *                  Absorbed map.
 *
 * @param[in]       detectedRooms_in
 *                  Detected rooms of the absorbed map.
 *
 * @param[in]       markerBasedRooms_in
 *                  Candidate rooms of the absorbed map.
 *
 * @param[in]       skipBadMarkerBasedRooms_in
 *                  True to leave candidate rooms flagged bad behind.
 */
void transferRooms(SemanticsManager *p_semanticsManager_inout,
                   map::Map         *p_currentMap_inout,
                   map::Map         *p_mergeMap_inout,
                   const std::vector<semantic::Room *> &detectedRooms_in,
                   const std::vector<semantic::Room *> &markerBasedRooms_in,
                   bool skipBadMarkerBasedRooms_in);

/*!
 * @brief           Moves the absorbed map's storeys into the surviving map. A
 *                  storey coplanar with one of the surviving map's is merged
 *                  into it and erased. Collapses the surviving map's storeys
 *                  afterwards.
 *
 * @param[in,out]   p_semanticsManager_inout
 *                  Manager that moves and erases the storeys; borrowed.
 *
 * @param[in,out]   p_currentMap_inout
 *                  Surviving map.
 *
 * @param[in,out]   p_mergeMap_inout
 *                  Absorbed map.
 *
 * @param[in]       floors_in
 *                  Storeys of the absorbed map.
 *
 * @param[in]       configuration_in
 *                  Merge pairing thresholds.
 */
void transferFloors(SemanticsManager *p_semanticsManager_inout,
                    map::Map         *p_currentMap_inout,
                    map::Map         *p_mergeMap_inout,
                    const std::vector<semantic::Floor *> &floors_in,
                    const semantic::SemanticVerifyConfig &configuration_in);

/*!
 * @brief           Merges a duplicate storey's plane evidence (when it is the
 *                  better observed one) and rooms into the retained storey.
 *
 * @param[in,out]   p_retainedFloor_inout
 *                  Storey that stays; borrowed.
 *
 * @param[in]       p_duplicateFloor_in
 *                  Storey that is merged into it; borrowed.
 */
void mergeFloorEvidenceAndRooms(semantic::Floor *p_retainedFloor_inout,
                                semantic::Floor *p_duplicateFloor_in);

/*!
 * @brief           Collapses the surviving map's storeys into the best
 *                  observed one: rooms move to it and the others are erased.
 *
 * @param[in,out]   p_semanticsManager_inout
 *                  Manager that lists and erases the storeys; borrowed.
 *
 * @param[in,out]   p_survivingMap_inout
 *                  Surviving map.
 */
void collapseMergedFloors(SemanticsManager *p_semanticsManager_inout,
                          map::Map         *p_survivingMap_inout);

/*!
 * @brief           Indexes the walls of every valid room of the surviving map
 *                  by plane id in the map, so imported rooms' walls resolve
 *                  before fusion.
 *
 * @param[in,out]   p_semanticsManager_inout
 *                  Manager that lists the rooms; borrowed.
 *
 * @param[in,out]   p_currentMap_inout
 *                  Surviving map.
 */
void indexAbsorbedRoomWalls(SemanticsManager *p_semanticsManager_inout,
                            map::Map         *p_currentMap_inout);

/*!
 * @brief           Fuses each duplicate room that a map merge brought in
 *                  into the room it duplicates. The pairs come from the
 *                  geometric room pairing (walls coplanar, same facing and
 *                  overlapping; the anchor rooms by the map link); this
 *                  function pairs nothing itself. The imported room's walls
 *                  and passages move to the retained room, a wall behind a
 *                  passable passage stays with the far-side room, passage
 *                  and floor references are re-pointed, and the imported
 *                  room is retired. A pair whose rooms lie on opposite
 *                  sides of a passable passage opening is left alone.
 *
 * @param[in,out]   p_semanticsManager_inout
 *                  Manager that lists the map's passages, rooms and floors
 *                  and erases the retired rooms; borrowed.
 *
 * @param[in,out]   p_map_inout
 *                  Surviving map whose room graph is reconciled.
 *
 * @param[in]       duplicateRoomPairs_in
 *                  Pairs (imported room, retained room). Borrowed; both
 *                  rooms belong to p_map_inout.
 */
void fuseDuplicateRoomsAfterMerge(
    SemanticsManager *p_semanticsManager_inout,
    map::Map         *p_map_inout,
    const std::vector<std::pair<semantic::Room *, semantic::Room *>>
        &duplicateRoomPairs_in);

/*!
 * @brief           Tests whether a segment traverses a passable passage
 *                  aperture.
 *
 *                  Mirrors the far-side wall-routing crossing test used by
 *                  the semantic manager so that the merge path can apply
 *                  the same rule when imported walls are copied into a
 *                  retained room.
 *
 * @param[in]       segmentStart_world_m_in
 *                  First endpoint in the active map frame.
 *
 * @param[in]       segmentEnd_world_m_in
 *                  Second endpoint in the active map frame.
 *
 * @param[in]       p_passage_in
 *                  Passable passage defining the finite aperture.
 *
 * @param[in]       groundNormal_world_in
 *                  Unit ground normal in the active map frame.
 *
 * @param[in]       openingMargin_m_in
 *                  Aperture expansion used for noisy geometry.
 *
 * @param[in]       minimumSideDistance_m_in
 *                  Required endpoint distance from plane.
 *
 * @param[out]      crossesOpening_out
 *                  True only when the segment crosses inside the finite
 *                  opening.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus crossesPassablePassageOpening(
    const Eigen::Vector3d              &segmentStart_world_m_in,
    const Eigen::Vector3d              &segmentEnd_world_m_in,
    vs_graphs::core::semantic::Passage *p_passage_in,
    const Eigen::Vector3d              &groundNormal_world_in,
    const double                        openingMargin_m_in,
    const double                        minimumSideDistance_m_in,
    bool                               &crossesOpening_out);

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_GRAPH_MERGING_PRIVATE_FUNCTIONS_H \
        */

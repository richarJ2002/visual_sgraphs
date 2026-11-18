/*!
 * @file            private_functions.h
 *
 * @brief           Declares module-internal helpers shared between the
 *                  SemanticsManager translation units.
 *
 * @note            These helpers were file-scope entities inside the
 *                  anonymous namespace of SemanticsManager.cc; external
 *                  linkage here is module-internal only. Names are kept
 *                  verbatim (identifier renaming is a separate step).
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_PRIVATE_FUNCTIONS_H

#include <cmath>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "configuration/types/types.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/room/objects/RoomClass.h"
#include "scene_understanding/semantics_manager/objects/FiniteWallSegment2dStruct.h"
#include "scene_understanding/semantics_manager/objects/RoomWallFootprintStruct.h"
#include "scene_understanding/semantics_manager/objects/SemanticsManagerStatusEnum.h"
#include "scene_understanding/semantics_manager/objects/WallAdmissionEvidenceStruct.h"
#include "scene_understanding/semantics_manager/objects/WallLoopClosureStruct.h"

namespace vs_graphs::core::map
{
class Map;
}

namespace vs_graphs
{
namespace core
{

namespace geometric
{
class Plane;
} /* namespace geometric */

namespace semantic
{
class Passage;
} /* namespace semantic */

/*!
 * @brief           Decides whether a wall has enough evidence to join a room,
 *                  and records the evidence behind that decision.
 *
 * @param[in]       p_wall_in
 *                  Wall to judge; a null or bad wall gets no evidence.
 *
 * @param[in]       p_systemParams_in
 *                  Thresholds for the fit, the extent and the observation
 *                  count; shall be non-null.
 *
 * @param[in]       groundNormal_world_in
 *                  Ground normal in the world frame, used to measure the wall's
 *                  width and height along the ground; a zero vector falls back
 *                  to arbitrary in-plane axes.
 *
 * @param[out]      admissionEvidence_out
 *                  The evidence and the decision (isAdmissible).
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus
    evaluateWallAdmissionEvidence(geometric::Plane          *p_wall_in,
                                  const types::SystemParams *p_systemParams_in,
                                  const Eigen::Vector3d &groundNormal_world_in,
                                  WallAdmissionEvidence &admissionEvidence_out);

/*!
 * @brief           Computes the scalar two-dimensional cross product.
 *
 * @param[in]       firstVector_in
 *                  First vector.
 *
 * @param[in]       secondVector_in
 *                  Second vector.
 *
 * @param[out]      crossProduct_out
 *                  Signed scalar cross product.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus
    crossProduct2d(const Eigen::Vector2d &firstVector_in,
                   const Eigen::Vector2d &secondVector_in,
                   double                &crossProduct_out);

/*!
 * @brief           Builds a robust finite wall segment on the horizontal ground
 *                  plane.
 *
 * @param[in]       p_wall_in
 *                  Wall whose observed cloud defines the finite extent.
 *
 * @param[in]       groundNormal_world_in
 *                  Unit ground normal in the world frame.
 *
 * @param[in]       groundAxisU_world_in
 *                  First horizontal ground axis.
 *
 * @param[in]       groundAxisV_world_in
 *                  Second horizontal ground axis.
 *
 * @param[in]       endpointTrimRatio_in
 *                  Fraction trimmed from both extent tails.
 *
 * @param[in]       minimumWallLength_m_in
 *                  Minimum accepted horizontal length.
 *
 * @param[in,out]   segment_inout
 *                  Resulting finite horizontal segment; written only when the
 *                  wall is usable.
 *
 * @param[out]      isBuilt_out
 *                  True when the wall provides a valid finite segment.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus
    buildFiniteWallSegment2d(geometric::Plane      *p_wall_in,
                             const Eigen::Vector3d &groundNormal_world_in,
                             const Eigen::Vector3d &groundAxisU_world_in,
                             const Eigen::Vector3d &groundAxisV_world_in,
                             const double           endpointTrimRatio_in,
                             const double           minimumWallLength_m_in,
                             FiniteWallSegment2d   &segment_inout,
                             bool                  &isBuilt_out);

/*!
 * @brief           Intersects the infinite lines supporting two finite wall
 *                  segments.
 *
 * @param[in]       firstSegment_in
 *                  First wall segment.
 *
 * @param[in]       secondSegment_in
 *                  Second wall segment.
 *
 * @param[out]      lineIntersection_world_m_out
 *                  Intersection in horizontal world axes.
 *
 * @param[out]      firstParameter_out
 *                  Parametric coordinate on the first segment.
 *
 * @param[out]      secondParameter_out
 *                  Parametric coordinate on the second segment.
 *
 * @param[out]      hasIntersection_out
 *                  False when the supporting lines are parallel.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus
    intersectSupportingLines(const FiniteWallSegment2d &firstSegment_in,
                             const FiniteWallSegment2d &secondSegment_in,
                             Eigen::Vector2d &lineIntersection_world_m_out,
                             double          &firstParameter_out,
                             double          &secondParameter_out,
                             bool            &hasIntersection_out);

/*!
 * @brief           Returns the Euclidean distance from a point to a finite
 *                  segment.
 */
[[nodiscard]] SemanticsManagerStatus
    pointToSegmentDistance_m(const Eigen::Vector2d     &queryPoint_world_m_in,
                             const FiniteWallSegment2d &segment_in,
                             double                    &distance_m_out);

/*!
 * @brief           Core aperture-crossing math behind
 *                  segmentCrossesPassageOpening(), taking the aperture plane,
 *                  centroid and opening size directly.
 *
 * @param[in]       segmentStart_world_m_in
 *                  First endpoint, in metres.
 *
 * @param[in]       segmentEnd_world_m_in
 *                  Second endpoint, in metres.
 *
 * @param[in]       apertureEquation_world_in
 *                  Plane of the aperture.
 *
 * @param[in]       apertureCentroid_world_m_in
 *                  Centre of the aperture, in metres.
 *
 * @param[in]       apertureWidth_m_in
 *                  Width of the opening, in metres.
 *
 * @param[in]       apertureHeight_m_in
 *                  Height of the opening, in metres.
 *
 * @param[in]       groundNormal_world_in
 *                  Unit ground normal.
 *
 * @param[in]       openingMargin_m_in
 *                  Aperture expansion used for noisy
 *                  geometry, in metres.
 *
 * @param[in]       minimumSideDistance_m_in
 *                  Required endpoint distance from the
 *                  plane, in metres.
 *
 * @param[out]      crossesAperture_out
 *                  True when the segment crosses
 *                  inside the opening.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus
    segmentCrossesAperture(const Eigen::Vector3d &segmentStart_world_m_in,
                           const Eigen::Vector3d &segmentEnd_world_m_in,
                           const Eigen::Vector4d &apertureEquation_world_in,
                           const Eigen::Vector3d &apertureCentroid_world_m_in,
                           const double           apertureWidth_m_in,
                           const double           apertureHeight_m_in,
                           const Eigen::Vector3d &groundNormal_world_in,
                           const double           openingMargin_m_in,
                           const double           minimumSideDistance_m_in,
                           bool                  &crossesAperture_out);

/*!
 * @brief           Tests whether a segment crosses a passage aperture.
 *
 * @param[in]       segmentStart_world_m_in
 *                  First endpoint in the active map frame.
 *
 * @param[in]       segmentEnd_world_m_in
 *                  Second endpoint in the active map frame.
 *
 * @param[in]       p_passage_in
 *                  Passage defining the finite aperture.
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
 * @param[out]      crossesPassageOpening_out
 *                  True when the segment crosses inside the finite opening.
 *
 * @param[in]       requirePassable_in
 *                  True when the passage must already be passable before the
 *                  geometric test may fire. Far-side wall routing may pass
 *                  false so the aperture geometry alone drives the decision
 *                  even while the passage is still being confirmed.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus
    segmentCrossesPassageOpening(const Eigen::Vector3d &segmentStart_world_m_in,
                                 const Eigen::Vector3d &segmentEnd_world_m_in,
                                 semantic::Passage     *p_passage_in,
                                 const Eigen::Vector3d &groundNormal_world_in,
                                 const double           openingMargin_m_in,
                                 const double minimumSideDistance_m_in,
                                 bool        &crossesPassageOpening_out,
                                 const bool   requirePassable_in = true);

/*!
 * @brief           Tests whether a straight segment between two points is
 *                  blocked by a wall belonging to a room other than the ones
 *                  the segment is meant to connect.
 *
 *                  Threading one passage's own bounded aperture is necessary
 *                  but not sufficient proof that two points are the direct two
 *                  sides of THAT passage: in a corridor with several rooms and
 *                  doors in a row, a straight line can thread one passage's
 *                  opening while still passing directly through an
 *                  intervening room's own wall. When it does, something else
 *                  -- a wall, and by implication a room -- provably sits
 *                  between the two points, so they are not each other's
 *                  direct neighbour through this passage.
 *
 * @param[in]       segmentStart_world_m_in
 *                  One endpoint of the candidate segment.
 *
 * @param[in]       segmentEnd_world_m_in
 *                  The other endpoint of the candidate segment.
 *
 * @param[in]       excludedRooms_in
 *                  Rooms whose own walls are not "foreign" -- typically the
 *                  rooms/placeholders the segment itself is testing.
 *
 * @param[in]       allRooms_in
 *                  Every currently known room to search for a blocking wall.
 *
 * @param[in]       groundAxisU_world_in
 *                  First horizontal ground axis (matches
 *                  buildFiniteWallSegment2d).
 *
 * @param[in]       groundAxisV_world_in
 *                  Second horizontal ground axis.
 *
 * @param[in]       groundNormal_world_in
 *                  Unit ground normal in the world frame.
 *
 * @param[in]       endpointTrimRatio_in
 *                  Forwarded to buildFiniteWallSegment2d.
 *
 * @param[in]       minimumWallLength_m_in
 *                  Forwarded to buildFiniteWallSegment2d.
 *
 * @param[out]      crossesForeignWall_out
 *                  True when a foreign room's own finite wall extent blocks the
 *                  segment.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus segmentCrossesForeignWall(
    const Eigen::Vector3d &segmentStart_world_m_in,
    const Eigen::Vector3d &segmentEnd_world_m_in,
    const std::vector<vs_graphs::core::semantic::Room *> &excludedRooms_in,
    const std::vector<vs_graphs::core::semantic::Room *> &allRooms_in,
    const Eigen::Vector3d                                &groundAxisU_world_in,
    const Eigen::Vector3d                                &groundAxisV_world_in,
    const Eigen::Vector3d                                &groundNormal_world_in,
    const double                                          endpointTrimRatio_in,
    const double minimumWallLength_m_in,
    bool        &crossesForeignWall_out);

/*!
 * @brief           Fills the OPPOSITE_FACE match evidence of a WALL Plane from
 *                  one geometry snapshot: equation, support cloud and
 *                  centroid, plus the stamped observation origin as the
 *                  observer side (left empty when the plane has none).
 *
 * @param[in]       p_wall_in
 *                  Wall plane, non-owning and non-null.
 *
 * @param[out]      wallEvidence_out
 *                  Evidence for utils::utils::Utils::findMatchingPlane().
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus
    buildWallFaceEvidence(geometric::Plane            *p_wall_in,
                          utils::utils::PlaneEvidence &wallEvidence_out);

/*!
 * @brief           Fills the OPPOSITE_FACE settings bundle of the wall-face
 *                  pairing: room_seg.walls_parallelism_thresh as the angle
 *                  and sem_seg.wall_pairing.maximum_thickness and
 *                  minimum_overlap_ratio.
 *
 * @param[in]       groundNormal_world_in
 *                  Ground normal in the world frame; zero when unknown.
 *
 * @param[out]      matchSettings_out
 *                  Settings for utils::utils::Utils::findMatchingPlane().
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus buildWallFaceMatchSettings(
    const Eigen::Vector3d            &groundNormal_world_in,
    utils::utils::PlaneMatchSettings &matchSettings_out);

/*!
 * @brief           Names a plane variant for log and pipeline output.
 *
 * @param[in]       planeVariant_in
 *                  Variant to name.
 *
 * @param[out]      variantName_out
 *                  Static, null-terminated name: WALL, GROUND, DOOR, WINDOW,
 *                  CEILING or UNDEFINED (also for any unlisted variant); not
 *                  owned by the caller.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus
    getPlaneVariantName(const geometric::Plane::PlaneVariant planeVariant_in,
                        const char                         *&variantName_out);

/*!
 * @brief           Finds the room whose walls include a wall.
 *
 * @param[in]       rooms_in
 *                  Rooms to search; null and bad rooms are skipped.
 *
 * @param[in]       p_wall_in
 *                  Wall to look up; non-owning.
 *
 * @param[out]      p_wallRoom_out
 *                  First room owning the wall; null when none does.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus
    findWallRoom(const std::vector<semantic::Room *> &rooms_in,
                 geometric::Plane                    *p_wall_in,
                 semantic::Room                     *&p_wallRoom_out);

/*!
 * @brief           Checks the topology rules of one passage: one or two walls,
 *                  two walls in different rooms once both are owned, at most
 *                  two linked rooms, and a prospective room other than the
 *                  host wall's room.
 *
 * @param[in]       wallRooms_in
 *                  Owning room of each valid wall, host wall first; null for
 *                  an unowned wall.
 *
 * @param[in]       linkedRoomCount_in
 *                  Number of rooms listing the passage.
 *
 * @param[in]       p_prospectiveRoom_in
 *                  Prospective room of the passage; null when it has none.
 *
 * @param[out]      isPassageTopologyValid_out
 *                  True when every rule holds.
 *
 * @param[out]      violationReason_out
 *                  First broken rule; empty when valid.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus
    isPassageTopologyValid(const std::vector<semantic::Room *> &wallRooms_in,
                           std::size_t     linkedRoomCount_in,
                           semantic::Room *p_prospectiveRoom_in,
                           bool           &isPassageTopologyValid_out,
                           std::string    &violationReason_out);

/*!
 * @brief           Computes the floor-plane box that holds every wall-extent
 *                  corner of a room. Each wall's corners come from its
 *                  Plane::getGeometryMetadataSnapshot() min/max U/V extents,
 *                  the same data the S-Graph archive writes. The box is
 *                  aligned to the horizontal normal of the longest wall.
 *
 * @param[in]       roomWalls_in
 *                  Walls of the room, non-owning; null, bad and extent-less
 *                  walls and planes facing up or down are skipped.
 *
 * @param[in]       upDirection_world_in
 *                  Map up direction (Map::getUpDirection()); the floor plane
 *                  is perpendicular to it. Need not be unit length.
 *
 * @param[out]      footprint_out
 *                  The box; written only when hasFootprint_out is true.
 *
 * @param[out]      hasFootprint_out
 *                  False when the up direction is invalid or the walls do not
 *                  span a box: no usable wall, or no wall more than 45 degrees
 *                  from parallel to the longest wall.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 *
 * @frame           World (the walls' map frame).
 *
 * @units           metres
 */
[[nodiscard]] SemanticsManagerStatus computeRoomWallFootprint(
    const std::vector<geometric::Plane *> &roomWalls_in,
    const Eigen::Vector3d                 &upDirection_world_in,
    RoomWallFootprint                     &footprint_out,
    bool                                  &hasFootprint_out);

/*!
 * @brief           Computes the semantic centre of a room from its walls: the
 *                  centre of computeRoomWallFootprint()'s box on the floor
 *                  plane of the room's map, at the height of the fallback
 *                  below. When the walls do not span a box (a single wall,
 *                  or only parallel walls) the centre is the fallback: the
 *                  mean of the wall centroids, each pushed 0.10 m inward
 *                  along Room::getWallNormalTowardRoom_world(). Undamped;
 *                  callers apply their own damping.
 *
 * @param[in]       p_room_in
 *                  Room the walls belong to, non-owning; its current
 *                  centroid orients the inward normals and its map gives the
 *                  up direction (no map: fallback only).
 *
 * @param[in]       roomWalls_in
 *                  Walls of the room, non-owning; null, bad and non-finite
 *                  walls are skipped.
 *
 * @param[out]      roomCentre_world_m_out
 *                  The centre; empty when the room is null or no wall is
 *                  usable.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 *
 * @frame           World (the room's map frame).
 *
 * @units           metres
 */
[[nodiscard]] SemanticsManagerStatus computeRoomCentreFromWalls(
    semantic::Room                        *p_room_in,
    const std::vector<geometric::Plane *> &roomWalls_in,
    std::optional<Eigen::Vector3d>        &roomCentre_world_m_out);

/*!
 * @brief           Tests whether a point lies on the side of a wall face
 *                  opposite to where the face was first observed from
 *                  (Plane::getObservationOrigin_world()). Only the side of a
 *                  surface turned toward a camera can be seen, so the face
 *                  bounds the space on its observed side, never the point's.
 *
 * @param[in]       p_wall_in
 *                  Wall face, non-owning; null or bad gives false.
 *
 * @param[in]       point_world_m_in
 *                  Point to test.
 *
 * @param[out]      isOnForeignSide_out
 *                  True only when the face has an observation origin and both
 *                  the origin and the point lie at least 0.10 m from the
 *                  plane, on opposite sides; false whenever a side does not
 *                  resolve.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 *
 * @frame           World (the wall's map frame).
 *
 * @units           metres
 */
[[nodiscard]] SemanticsManagerStatus
    isPointOnForeignSideOfWallFace(geometric::Plane      *p_wall_in,
                                   const Eigen::Vector3d &point_world_m_in,
                                   bool                  &isOnForeignSide_out);

/*!
 * @brief           Tests whether a room wall frames a passage opening: it is
 *                  one of the passage's supporting walls, or a wall surface
 *                  aligned with the passage plane, close to it, whose support
 *                  points reach the opening frame.
 *
 * @param[in]       p_roomWall_in
 *                  Room wall, non-owning; null or bad gives false.
 *
 * @param[in]       supportingWalls_in
 *                  Walls the passage is linked to.
 *
 * @param[in]       passageEquation_world_in
 *                  Unit-normal passage plane, world frame.
 *
 * @param[in]       passageCentroid_world_m_in
 *                  Passage centroid, world frame, metres.
 *
 * @param[in]       passageWidth_m_in
 *                  Passage width, metres; a non-positive width gives false.
 *
 * @param[in]       openingFrameReach_m_in
 *                  Largest distance in the wall plane from the passage
 *                  centroid to a wall point that frames the opening, metres.
 *
 * @param[out]      isAtPassageOpening_out
 *                  True when the wall frames the opening.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus isWallAtPassageOpening(
    geometric::Plane                      *p_roomWall_in,
    const std::vector<geometric::Plane *> &supportingWalls_in,
    const Eigen::Vector4d                 &passageEquation_world_in,
    const Eigen::Vector3d                 &passageCentroid_world_m_in,
    double                                 passageWidth_m_in,
    double                                 openingFrameReach_m_in,
    bool                                  &isAtPassageOpening_out);

/*!
 * @brief           Tests whether a room centroid lies on the known side of a
 *                  passage plane.
 *
 * @param[in]       p_room_in
 *                  Room, non-owning; null gives false.
 *
 * @param[in]       passageEquation_world_in
 *                  Unit-normal passage plane, world frame.
 *
 * @param[in]       knownSide_in
 *                  Known-side provenance of the passage; without a direction
 *                  the result is false.
 *
 * @param[in]       knownSideSign_in
 *                  Sign of the known-side direction along the passage
 *                  normal; near 0 gives false.
 *
 * @param[out]      isOnKnownSide_out
 *                  True when the room centroid is on the known side.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus isRoomOnKnownSide(
    semantic::Room                               *p_room_in,
    const Eigen::Vector4d                        &passageEquation_world_in,
    const semantic::Passage::KnownSideProvenance &knownSide_in,
    double                                        knownSideSign_in,
    bool                                         &isOnKnownSide_out);

/*!
 * @brief           Links a passage to a room unless a passage with the same
 *                  id is already linked, and logs links the room did not
 *                  hold before this pass.
 *
 * @param[in,out]   p_room_inout
 *                  Room to link; null does nothing.
 *
 * @param[in]       p_passage_in
 *                  Passage to link; not null.
 *
 * @param[in]       previousPassageIdsByRoom_in
 *                  Passage ids each room held before this pass.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus linkRoomToPassage(
    semantic::Room    *p_room_inout,
    semantic::Passage *p_passage_in,
    const std::unordered_map<semantic::Room *, std::unordered_set<int>>
        &previousPassageIdsByRoom_in);

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_PRIVATE_FUNCTIONS_H */

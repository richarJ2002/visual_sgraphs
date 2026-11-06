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
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "configuration/types/types.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/semantic/room/objects/RoomClass.h"
#include "scene_understanding/semantics_manager/objects/RoomWallFootprintStruct.h"
#include "scene_understanding/semantics_manager/objects/SemanticsManagerStatusEnum.h"
#include "scene_understanding/semantics_manager/objects/WallGapStruct.h"
#include "scene_understanding/semantics_manager/objects/WallSlabProfileStruct.h"

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
 * @brief           Finite horizontal representation of one observed room wall.
 */
struct FiniteWallSegment2d
{
    /*!
     * @brief           Wall the segment was built from; non-owning.
     *
     * @frame           N/A
     *
     * @units           N/A
     */
    geometric::Plane *p_wall = nullptr;

    /*!
     * @brief           First end of the wall on the ground plane.
     *
     * @frame           World, projected onto the two ground axes
     *
     * @units           metres
     */
    Eigen::Vector2d wallStart_world_m = Eigen::Vector2d::Zero();

    /*!
     * @brief           Second end of the wall on the ground plane.
     *
     * @frame           World, projected onto the two ground axes
     *
     * @units           metres
     */
    Eigen::Vector2d wallEnd_world_m = Eigen::Vector2d::Zero();

    /*!
     * @brief           Distance between the two ends.
     *
     * @frame           N/A
     *
     * @units           metres
     */
    double length_m = 0.0;

    /*!
     * @brief           How well supported the wall is: observation count (at
     *                  least
     *                  1) times the square root of the length; larger wins when
     *                     walls compete.
     *
     * @frame           N/A
     *
     * @units           square root of metres
     */
    double supportScore = 0.0;
};

/*!
 * @brief           Facts that decide whether a wall plane may join a room;
 *                  filled by evaluateWallAdmissionEvidence().
 */
struct WallAdmissionEvidence
{
    /*!
     * @brief           True when the wall may join a room: it is a wall by type
     *                  and by expected type, its finite fit is adequate, and it
     *                  was seen often enough or strongly enough at first sight.
     *
     * @frame           N/A
     *
     * @units           N/A
     */
    bool isAdmissible = false;

    /*!
     * @brief           True when at least 20 points fit the plane, enough of
     *                  the points fit, and the fitted extent is large enough
     *                  (length, height and area).
     *
     * @frame           N/A
     *
     * @units           N/A
     */
    bool hasAdequateFiniteFit = false;

    /*!
     * @brief           Points of the support cloud with finite coordinates.
     *
     * @frame           N/A
     *
     * @units           number of points
     */
    std::size_t finitePointCount = 0U;

    /*!
     * @brief           Finite points within the RANSAC distance of the plane.
     *
     * @frame           N/A
     *
     * @units           number of points
     */
    std::size_t fittedPointCount = 0U;
    /*!
     * @brief           Number of times the wall has been observed.
     *
     * @frame           N/A
     *
     * @units           number of observations
     */
    std::size_t observationCount = 0U;
};

/*!
 * @brief           Result of attempting to close a set of wall segments into
 *                  one ordered, non-open loop.
 */
struct WallLoopClosure
{
    /*!
     * @brief           True when the walls do not form a closed loop (the
     *                  default); false when every neighbouring pair of walls
     *                  meets within the allowed corner gap.
     */
    bool hasOpenBoundary = true;

    /*!
     * @brief           Corners of the closed loop in the order of the walls
     *                  around the room, in the world frame projected onto the
     *                  two ground axes, metres; empty when the boundary is
     *                  open.
     */
    std::vector<Eigen::Vector2d> loopCorners_world_m;
};

/*!
 * @brief           Computes Eigen's rotation between two directions in its
 *                  own translation unit.
 *
 * @param[in]       sourceDirection_in
 *                  Borrowed source direction; not modified or retained.
 *
 * @param[in]       targetDirection_in
 *                  Borrowed target direction; not modified or retained.
 *
 * @return          Double-precision quaternion from setFromTwoVectors.
 *
 * @note            Delegates unchanged to Eigen, including its normalization,
 *                  opposite-direction branch and degenerate/nonfinite behavior.
 *                  No additional validation or shared mutable state is added.
 *
 * @frame           Active source-to-target rotation, applied on the left of
 *                  column vectors; the caller supplies the frame conventions.
 *
 * @units           Dimensionless directions and quaternion coefficients.
 */
Eigen::Quaterniond
    computeRotationBetweenDirections(const Eigen::Vector3d &sourceDirection_in,
                                     const Eigen::Vector3d &targetDirection_in);

/*!
 * @brief           Tests whether an observed finite wall separates two
 *                  positions.
 *
 *                  The infinite plane performs the side test, while the mapped
 *                  cloud bounds reject unrelated coplanar wall segments. This
 *                  is used as a veto when connected free-space evidence
 *                  suggests two room hypotheses may describe the same physical
 *                  room.
 *
 * @param[in]       wallList_world_in
 *                  Candidate wall surfaces expressed in the active map frame.
 *
 * @param[in]       firstPoint_world_m_in
 *                  First position expressed in the active map frame, in metres.
 *
 * @param[in]       secondPoint_world_m_in
 *                  Second position expressed in the active map frame, in
 *                  metres.
 *
 * @param[in]       finiteBoundsMargin_m_in
 *                  Margin applied around the observed wall-cloud bounds.
 *
 * @param[in]       isLeadingWallVoteAccepted_in
 *                  When true, a plane whose type is not yet WALL but whose
 *                  leading semantic vote is WALL also counts as a wall;
 *                  when false, only planes of confirmed type WALL count.
 *
 * @param[out]      hasSeparatingFiniteWall_out
 *                  True when the segment crosses an observed finite wall patch.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus hasSeparatingFiniteWall(
    const std::vector<geometric::Plane *> &wallList_world_in,
    const Eigen::Vector3d                 &firstPoint_world_m_in,
    const Eigen::Vector3d                 &secondPoint_world_m_in,
    const double                           finiteBoundsMargin_m_in,
    const bool                             isLeadingWallVoteAccepted_in,
    bool                                  &hasSeparatingFiniteWall_out);

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
 * @brief           Attempts to close the given wall segments (sorted here by
 *                  angle from the room centroid) into one ordered loop, exactly
 *                  as validateRoomBoundaries() always did for a room's full
 *                  wall set. Factored out so the caller can retry on a reduced
 *                  subset when the full set doesn't close (see
 *                  validateRoomBoundaries()'s single-outlier-exclusion retry).
 */
[[nodiscard]] SemanticsManagerStatus tryCloseWallLoop(
    std::vector<FiniteWallSegment2d> wallSegments_in,
    const Eigen::Vector2d           &roomCentroidGround_m_in,
    const types::SystemParams::RoomSeg::BoundaryTopology &topologyParameters_in,
    WallLoopClosure                                      &closure_out);

/*!
 * @brief           Finds the angular sectors (from roomCentroidGround_m_in)
 *                  with no wall evidence -- the "where is this room still
 *                  unobserved" signal (user rule: track and expose
 *                  incomplete-room state, not just a pass/fail boundary
 *                  status).
 *
 *                  Deliberately coarser than the corner-closing algorithm
 *                  above: each wall is reduced to its 2D midpoint angle from
 *                  the centroid, not its true angular extent, trading a small
 *                  amount of precision (a wide wall's own angular span isn't
 *                  subtracted from a neighbouring gap) for a computation that
 *                  stays meaningful at any wall count, including 0 or 1 -- the
 *                  boundary-loop algorithm's own machinery only starts
 *                  producing useful output once minimumWallCount is met.
 *
 * @param[in]       wallSegments_in
 *                  Finite wall segments of the room.
 *
 * @param[in]       roomCentroidGround_m_in
 *                  Room centroid on the ground plane, in
 *                  metres.
 *
 * @param[out]      roomObservationGaps_out
 *                  Sectors wider than the threshold that
 *                  no wall covers.
 *
 * @param[in]       gapThreshold_rad_in
 *                  Smallest angle between neighbouring
 *                  wall midpoints reported as a gap, in
 *                  radians.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus computeRoomObservationGaps(
    const std::vector<FiniteWallSegment2d>      &wallSegments_in,
    const Eigen::Vector2d                       &roomCentroidGround_m_in,
    /* An axis-aligned (or any) rectangle's four wall midpoints sit exactly
     * on its principal axes as seen from the centroid -- always exactly 90
     * deg apart by construction, regardless of aspect ratio. The threshold
     * must clear that deterministic case with margin, or every well-formed
     * rectangular room reports four phantom gaps. */
    std::vector<semantic::Room::ObservationGap> &roomObservationGaps_out,
    double gapThreshold_rad_in = 100.0 * M_PI / 180.0);

/*!
 * @brief           Computes the unsigned area of an ordered horizontal polygon.
 *
 * @param[in]       polygonVertices_world_m_in
 *                  Vertices in order, in metres; fewer
 *                  than three give an area of 0.
 *
 * @param[out]      polygonArea_m2_out
 *                  Area, in square metres.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus computePolygonArea_m2(
    const std::vector<Eigen::Vector2d> &polygonVertices_world_m_in,
    double                             &polygonArea_m2_out);

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

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_PRIVATE_FUNCTIONS_H */

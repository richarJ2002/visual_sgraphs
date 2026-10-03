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

#ifndef VS_GRAPHS_CORE_SEMANTICSMANAGER_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_SEMANTICSMANAGER_PRIVATE_FUNCTIONS_H

#include <cmath>
#include <vector>

#include "Semantic/Room.h"
#include "SemanticsManagerStatus.h"
#include "Types/objects/SystemParams.h"

namespace vs_graphs
{
namespace core
{

class Map;

namespace geometric
{
class Plane;
} // namespace geometric

namespace semantic
{
class Passage;
} // namespace semantic

/*!
 * @brief Finite horizontal representation of one observed room wall.
 */
struct FiniteWallSegment2d
{
    /*!
     * @brief       Wall the segment was built from; non-owning.
     *
     * @frame       N/A
     * @units       N/A
     */
    geometric::Plane *p_wall = nullptr;

    /*!
     * @brief       First end of the wall on the ground plane.
     *
     * @frame       World, projected onto the two ground axes
     * @units       metres
     */
    Eigen::Vector2d wallStart_world_m = Eigen::Vector2d::Zero();

    /*!
     * @brief       Second end of the wall on the ground plane.
     *
     * @frame       World, projected onto the two ground axes
     * @units       metres
     */
    Eigen::Vector2d wallEnd_world_m = Eigen::Vector2d::Zero();

    /*!
     * @brief       Distance between the two ends.
     *
     * @frame       N/A
     * @units       metres
     */
    double length_m = 0.0;

    /*!
     * @brief       How well supported the wall is: observation count (at least
     *              1) times the square root of the length; larger wins when
     *              walls compete.
     *
     * @frame       N/A
     * @units       square root of metres
     */
    double supportScore = 0.0;
};

/*!
 * @brief        Facts that decide whether a wall plane may join a room; filled
 *               by evaluateWallAdmissionEvidence().
 */
struct WallAdmissionEvidence
{
    /*!
     * @brief       True when the wall may join a room: it is a wall by type and
     *              by expected type, its finite fit is adequate, and it was
     *              seen often enough or strongly enough at first sight.
     *
     * @frame       N/A
     * @units       N/A
     */
    bool isAdmissible = false;

    /*!
     * @brief       True when at least 20 points fit the plane, enough of the
     *              points fit, and the fitted extent is large enough (length,
     *              height and area).
     *
     * @frame       N/A
     * @units       N/A
     */
    bool hasAdequateFiniteFit = false;

    /*!
     * @brief       Points of the support cloud with finite coordinates.
     *
     * @frame       N/A
     * @units       number of points
     */
    std::size_t finitePointCount = 0U;

    /*!
     * @brief       Finite points within the RANSAC distance of the plane.
     *
     * @frame       N/A
     * @units       number of points
     */
    std::size_t fittedPointCount = 0U;
    /*!
     * @brief       Number of times the wall has been observed.
     *
     * @frame       N/A
     * @units       number of observations
     */
    std::size_t observationCount = 0U;
};

/*! @brief Result of attempting to close a set of wall segments into one
 *  ordered, non-open loop. */
struct WallLoopClosure
{
    /*!
     * @brief        True when the walls do not form a closed loop (the
     *               default); false when every neighbouring pair of walls meets
     *               within the allowed corner gap.
     */
    bool hasOpenBoundary = true;

    /*!
     * @brief        Corners of the closed loop in the order of the walls around
     *               the room, in the world frame projected onto the two ground
     *               axes, metres; empty when the boundary is open.
     */
    std::vector<Eigen::Vector2d> loopCorners_world_m;
};

/*!
 * @brief       Tests whether an observed finite wall separates two positions.
 *
 *              The infinite plane performs the side test, while the mapped
 *              cloud bounds reject unrelated coplanar wall segments. This is
 *              used as a veto when connected free-space evidence suggests two
 *              room hypotheses may describe the same physical room.
 *
 * @param[in]   wallList_world_in
 *              Candidate wall surfaces expressed in the active map frame.
 * @param[in]   firstPoint_world_m_in
 *              First position expressed in the active map frame, in metres.
 * @param[in]   secondPoint_world_m_in
 *              Second position expressed in the active map frame, in metres.
 * @param[in]   finiteBoundsMargin_m_in
 *              Margin applied around the observed wall-cloud bounds.
 *
 * @param[out]  hasSeparatingFiniteWall_out
 *              True when the segment crosses an observed finite wall patch.
 *
 * @return      SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus hasSeparatingFiniteWall(
    const std::vector<geometric::Plane *> &wallList_world_in,
    const Eigen::Vector3d                 &firstPoint_world_m_in,
    const Eigen::Vector3d                 &secondPoint_world_m_in,
    const double                           finiteBoundsMargin_m_in,
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
 * @brief Computes the scalar two-dimensional cross product.
 *
 * @param[in] firstVector_in First vector.
 * @param[in] secondVector_in Second vector.
 * @param[out] crossProduct_out Signed scalar cross product.
 * @return SEMANTICS_MANAGER_STATUS_SUCCESS.
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
 * @brief Returns the Euclidean distance from a point to a finite segment.
 */
[[nodiscard]] SemanticsManagerStatus
    pointToSegmentDistance_m(const Eigen::Vector2d     &queryPoint_world_m_in,
                             const FiniteWallSegment2d &segment_in,
                             double                    &distance_m_out);

/*!
 * @brief Attempts to close the given wall segments (sorted here by angle
 *        from the room centroid) into one ordered loop, exactly as
 *        validateRoomBoundaries() always did for a room's full wall set.
 *        Factored out so the caller can retry on a reduced subset when the
 *        full set doesn't close (see validateRoomBoundaries()'s single-
 *        outlier-exclusion retry).
 */
[[nodiscard]] SemanticsManagerStatus tryCloseWallLoop(
    std::vector<FiniteWallSegment2d> wallSegments_in,
    const Eigen::Vector2d           &roomCentroidGround_m_in,
    const types::SystemParams::RoomSeg::BoundaryTopology &topologyParameters_in,
    WallLoopClosure                                      &closure_out);

/*!
 * @brief Finds the angular sectors (from roomCentroidGround_m_in) with no
 *        wall evidence -- the "where is this room still unobserved" signal
 *        (user rule: track and expose incomplete-room state, not just a
 *        pass/fail boundary status).
 *
 *        Deliberately coarser than the corner-closing algorithm above: each
 *        wall is reduced to its 2D midpoint angle from the centroid, not its
 *        true angular extent, trading a small amount of precision (a wide
 *        wall's own angular span isn't subtracted from a neighbouring gap)
 *        for a computation that stays meaningful at any wall count,
 *        including 0 or 1 -- the boundary-loop algorithm's own machinery
 *        only starts producing useful output once minimumWallCount is met.
 *
 * @param[in]  wallSegments_in          Finite wall segments of the room.
 * @param[in]  roomCentroidGround_m_in  Room centroid on the ground plane, in
 *                                      metres.
 * @param[out] roomObservationGaps_out  Sectors wider than the threshold that
 *                                      no wall covers.
 * @param[in]  gapThreshold_rad_in      Smallest angle between neighbouring
 *                                      wall midpoints reported as a gap, in
 *                                      radians.
 * @return SEMANTICS_MANAGER_STATUS_SUCCESS.
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
 * @brief Computes the unsigned area of an ordered horizontal polygon.
 *
 * @param[in]  polygonVertices_world_m_in Vertices in order, in metres; fewer
 *                                        than three give an area of 0.
 * @param[out] polygonArea_m2_out         Area, in square metres.
 * @return SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus computePolygonArea_m2(
    const std::vector<Eigen::Vector2d> &polygonVertices_world_m_in,
    double                             &polygonArea_m2_out);

/*!
 * @brief       Core aperture-crossing math shared by both a confirmed
 *              Passage and a still-unconfirmed OpenPassageEvidence
 *              hypothesis (see segmentCrossesOpenPassageEvidence below) --
 *              the two differ only in where the plane equation, centroid,
 *              and opening size come from, never in how the crossing test
 *              itself works.
 *
 * @param[in]   segmentStart_world_m_in     First endpoint, in metres.
 * @param[in]   segmentEnd_world_m_in       Second endpoint, in metres.
 * @param[in]   apertureEquation_world_in   Plane of the aperture.
 * @param[in]   apertureCentroid_world_m_in Centre of the aperture, in metres.
 * @param[in]   apertureWidth_m_in          Width of the opening, in metres.
 * @param[in]   apertureHeight_m_in         Height of the opening, in metres.
 * @param[in]   groundNormal_world_in       Unit ground normal.
 * @param[in]   openingMargin_m_in          Aperture expansion used for noisy
 *                                          geometry, in metres.
 * @param[in]   minimumSideDistance_m_in    Required endpoint distance from the
 *                                          plane, in metres.
 * @param[out]  crossesAperture_out         True when the segment crosses
 *                                          inside the opening.
 * @return      SEMANTICS_MANAGER_STATUS_SUCCESS.
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
 * @brief Tests whether a segment crosses a passage aperture.
 *
 * @param[in] segmentStart_world_m_in First endpoint in the active map frame.
 * @param[in] segmentEnd_world_m_in Second endpoint in the active map frame.
 * @param[in] p_passage_in Passage defining the finite aperture.
 * @param[in] groundNormal_world_in Unit ground normal in the active map frame.
 * @param[in] openingMargin_m_in Aperture expansion used for noisy geometry.
 * @param[in] minimumSideDistance_m_in Required endpoint distance from plane.
 * @param[out] crossesPassageOpening_out True when the segment crosses inside
 *              the finite opening.
 * @param[in] requirePassable_in True when the passage must already be passable
 *              before the geometric test may fire. Far-side wall routing may
 *              pass false so the aperture geometry alone drives the decision
 *              even while the passage is still being confirmed.
 * @return SEMANTICS_MANAGER_STATUS_SUCCESS.
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
 * @brief       Same aperture-crossing test as segmentCrossesPassageOpening,
 *              but against a still-unconfirmed OpenPassageEvidence
 *              hypothesis instead of a confirmed Passage.
 *
 *              Passage confirmation requires several genuinely independent
 *              Voxblox skeleton snapshots (minimumConfirmationSnapshots,
 *              config-gated to guard against double-counting one latched ROS
 *              message -- see the skeleton-fingerprint check in
 *              detectDoorsAndDoorways()) and therefore real elapsed
 *              exploration time. Until that confirmation completes, no
 *              Passage object exists for mpAtlas->GetAllPassages() to
 *              return, so any far-side-routing check that only consults
 *              confirmed passages is blind for that entire window -- a wall
 *              genuinely on the far side of a real, already-evidenced
 *              opening falls through to ordinary admission and gets bound
 *              to the WRONG (near) room, exactly the corruption far-side
 *              routing exists to prevent. Using the same aperture geometry
 *              math against the pending evidence (its supporting wall's
 *              plane stands in for the eventual passage plane, its
 *              openingRadius_m/heightSpan_m for the eventual width/height --
 *              the same derivation createMapPassage() itself uses once
 *              confirmed) closes that window without weakening the
 *              confirmation gate itself: the passage still is not created,
 *              only wall ADMISSION becomes conservative while its identity
 *              is still ambiguous.
 */
[[nodiscard]] SemanticsManagerStatus segmentCrossesOpenPassageEvidence(
    const Eigen::Vector3d &segmentStart_world_m_in,
    const Eigen::Vector3d &segmentEnd_world_m_in,
    geometric::Plane      *p_evidenceSupportingWall_in,
    const Eigen::Vector3d &evidenceCentroid_world_m_in,
    const double           evidenceOpeningRadius_m_in,
    const double           evidenceHeightSpan_m_in,
    const Eigen::Vector3d &groundNormal_world_in,
    const double           openingMargin_m_in,
    const double           minimumSideDistance_m_in,
    bool                  &crossesOpenPassageEvidence_out);

/*!
 * @brief       Tests whether a straight segment between two points is
 *              blocked by a wall belonging to a room other than the ones
 *              the segment is meant to connect.
 *
 *              Threading one passage's own bounded aperture is necessary
 *              but not sufficient proof that two points are the direct two
 *              sides of THAT passage: in a corridor with several rooms and
 *              doors in a row, a straight line can thread one passage's
 *              opening while still passing directly through an
 *              intervening room's own wall. When it does, something else
 *              -- a wall, and by implication a room -- provably sits
 *              between the two points, so they are not each other's
 *              direct neighbour through this passage.
 *
 * @param[in]   segmentStart_world_m_in
 *              One endpoint of the candidate segment.
 * @param[in]   segmentEnd_world_m_in
 *              The other endpoint of the candidate segment.
 * @param[in]   excludedRooms_in
 *              Rooms whose own walls are not "foreign" -- typically the
 *              rooms/placeholders the segment itself is testing.
 * @param[in]   allRooms_in
 *              Every currently known room to search for a blocking wall.
 * @param[in]   groundAxisU_world_in
 *              First horizontal ground axis (matches buildFiniteWallSegment2d).
 * @param[in]   groundAxisV_world_in
 *              Second horizontal ground axis.
 * @param[in]   groundNormal_world_in
 *              Unit ground normal in the world frame.
 * @param[in]   endpointTrimRatio_in
 *              Forwarded to buildFiniteWallSegment2d.
 * @param[in]   minimumWallLength_m_in
 *              Forwarded to buildFiniteWallSegment2d.
 *
 * @param[out]  crossesForeignWall_out
 *              True when a foreign room's own finite wall extent blocks the
 *              segment.
 *
 * @return      SEMANTICS_MANAGER_STATUS_SUCCESS.
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
 * @brief       Tests whether two maps observe a common tagged room name.
 *
 *              A new map is a deterministic merge candidate for the current
 *              map only when at least one non-empty room tag collected from
 *              the other map's detected and marker-based rooms also appears
 *              among the current map's tagged rooms. Room tags originate from
 *              context snapshots and are propagated by matchRoomsToContext,
 *              so they are a stable correspondences key between maps.
 *
 * @param[in]   p_firstMap_in
 *              Map whose detected and marker-based room tags are collected.
 * @param[in]   p_secondMap_in
 *              Map whose tagged rooms are tested against the collected tags.
 *
 * @param[out]  sharesRoomNameTag_out
 *              True when both maps observe at least one shared room tag.
 *
 * @return      SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus
    sharesRoomNameTag(Map  *p_firstMap_in,
                      Map  *p_secondMap_in,
                      bool &sharesRoomNameTag_out);

/*!
 * @brief Projects a WALL Plane's finite support cloud onto a shared in-plane
 *        tangent frame, returning the resulting axis-aligned interval.
 *
 * @return false when the plane has no usable geometry (null/empty cloud, or
 *         a degenerate equation); the caller must treat that as "cannot
 *         claim overlap" rather than as a zero-size interval.
 */
[[nodiscard]] SemanticsManagerStatus
    projectPlaneFootprintOntoSharedAxes(geometric::Plane      *p_plane_in,
                                        const Eigen::Vector3d &axisU_world_in,
                                        const Eigen::Vector3d &axisV_world_in,
                                        double                &minimumU_m_out,
                                        double                &maximumU_m_out,
                                        double                &minimumV_m_out,
                                        double                &maximumV_m_out,
                                        bool                  &isProjected_out);

/*!
 * @brief Decides whether two WALL Planes are plausibly the two opposite
 *        faces of the same physical wall (axiom (e)): parallel, a plausible
 *        wall thickness apart, observed from opposite exterior sides, and
 *        overlapping in-plane footprint.
 */
[[nodiscard]] SemanticsManagerStatus
    arePlausibleTwinWallFaces(geometric::Plane      *p_first_in,
                              geometric::Plane      *p_second_in,
                              double                 minimumThickness_m_in,
                              double                 maximumThickness_m_in,
                              double                 minimumOverlapRatio_in,
                              const Eigen::Vector3d &groundNormal_world_in,
                              bool &arePlausibleTwinWallFaces_out);

} // namespace core
} // namespace vs_graphs

#endif /* VS_GRAPHS_CORE_SEMANTICSMANAGER_PRIVATE_FUNCTIONS_H */

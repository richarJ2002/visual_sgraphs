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
    Eigen::Vector2d start_world_m = Eigen::Vector2d::Zero();

    /*!
     * @brief       Second end of the wall on the ground plane.
     *
     * @frame       World, projected onto the two ground axes
     * @units       metres
     */
    Eigen::Vector2d end_world_m = Eigen::Vector2d::Zero();

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
    bool                         hasOpenBoundary = true;
    std::vector<Eigen::Vector2d> corners_world_m;
};

[[nodiscard]] SemanticsManagerStatus hasSeparatingFiniteWall(
    const std::vector<geometric::Plane *> &wallList_world_in,
    const Eigen::Vector3d                 &firstPoint_world_m_in,
    const Eigen::Vector3d                 &secondPoint_world_m_in,
    const double                           finiteBoundsMargin_m_in,
    bool                                  &hasSeparatingFiniteWall_out);

[[nodiscard]] SemanticsManagerStatus
    evaluateWallAdmissionEvidence(geometric::Plane          *p_wall_in,
                                  const types::SystemParams *p_systemParams_in,
                                  const Eigen::Vector3d &groundNormal_world_in,
                                  WallAdmissionEvidence &admissionEvidence_out);

[[nodiscard]] SemanticsManagerStatus
    crossProduct2d(const Eigen::Vector2d &firstVector_in,
                   const Eigen::Vector2d &secondVector_in,
                   double                &crossProduct_out);

[[nodiscard]] SemanticsManagerStatus
    buildFiniteWallSegment2d(geometric::Plane      *p_wall_in,
                             const Eigen::Vector3d &groundNormal_world_in,
                             const Eigen::Vector3d &groundAxisU_world_in,
                             const Eigen::Vector3d &groundAxisV_world_in,
                             const double           endpointTrimRatio_in,
                             const double           minimumWallLength_m_in,
                             FiniteWallSegment2d   &segment_inout,
                             bool                  &isBuilt_out);

[[nodiscard]] SemanticsManagerStatus
    intersectSupportingLines(const FiniteWallSegment2d &firstSegment_in,
                             const FiniteWallSegment2d &secondSegment_in,
                             Eigen::Vector2d &intersection_world_m_out,
                             double          &firstParameter_out,
                             double          &secondParameter_out,
                             bool            &hasIntersection_out);

[[nodiscard]] SemanticsManagerStatus
    pointToSegmentDistance_m(const Eigen::Vector2d     &point_world_m_in,
                             const FiniteWallSegment2d &segment_in,
                             double                    &distance_m_out);

[[nodiscard]] SemanticsManagerStatus tryCloseWallLoop(
    std::vector<FiniteWallSegment2d> wallSegments_in,
    const Eigen::Vector2d           &roomCentroidGround_m_in,
    const types::SystemParams::RoomSeg::BoundaryTopology &topologyParameters_in,
    WallLoopClosure                                      &closure_out);

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

[[nodiscard]] SemanticsManagerStatus computePolygonArea_m2(
    const std::vector<Eigen::Vector2d> &polygonVertices_world_m_in,
    double                             &polygonArea_m2_out);

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

[[nodiscard]] SemanticsManagerStatus
    segmentCrossesPassageOpening(const Eigen::Vector3d &segmentStart_world_m_in,
                                 const Eigen::Vector3d &segmentEnd_world_m_in,
                                 semantic::Passage     *p_passage_in,
                                 const Eigen::Vector3d &groundNormal_world_in,
                                 const double           openingMargin_m_in,
                                 const double minimumSideDistance_m_in,
                                 bool        &crossesPassageOpening_out,
                                 const bool   requirePassable_in = true);

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

[[nodiscard]] SemanticsManagerStatus
    sharesRoomNameTag(Map  *p_firstMap_in,
                      Map  *p_secondMap_in,
                      bool &sharesRoomNameTag_out);

[[nodiscard]] SemanticsManagerStatus
    projectPlaneFootprintOntoSharedAxes(geometric::Plane      *p_plane_in,
                                        const Eigen::Vector3d &axisU_world_in,
                                        const Eigen::Vector3d &axisV_world_in,
                                        double                &minimumU_m_out,
                                        double                &maximumU_m_out,
                                        double                &minimumV_m_out,
                                        double                &maximumV_m_out,
                                        bool                  &isProjected_out);

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

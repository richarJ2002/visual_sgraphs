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
     * @brief       TODO
     *
     * @frame       N/A
     * @unit        N/A
     */
    geometric::Plane *p_wall = nullptr;

    /*!
     * @brief       TODO
     *
     * @frame       TODO
     * @unit        meters
     */
    Eigen::Vector2d start_World_m = Eigen::Vector2d::Zero();

    /*!
     * @brief       TODO
     *
     * @frame       TODO
     * @unit        meters
     */
    Eigen::Vector2d end_World_m = Eigen::Vector2d::Zero();

    /*!
     * @brief       TODO
     *
     * @frame       N/A
     * @unit        meters
     */
    double length_m = 0.0;

    /*!
     * @brief       TODO
     *
     * @frame       N/A
     * @unit        N/A
     */
    double supportScore = 0.0;
};

struct WallAdmissionEvidence
{
    /*!
     * @brief       TODO
     *
     * @frame       N/A
     * @unit        N/A
     */
    bool admissible = false;

    /*!
     * @brief       TODO
     *
     * @frame       N/A
     * @unit        N/A
     */
    bool adequateFiniteFit = false;

    /*!
     * @brief       TODO
     *
     * @frame       N/A
     * @unit        Number of Points
     */
    std::size_t finitePointCount = 0U;

    /*!
     * @brief       TODO
     *
     * @frame       N/A
     * @unit        Number of Points
     */
    std::size_t fittedPointCount = 0U;
    /*!
     * @brief       TODO
     *
     * @frame       N/A
     * @unit        Number of Obeservations
     */
    std::size_t observationCount = 0U;
};

/*! @brief Result of attempting to close a set of wall segments into one
 *  ordered, non-open loop. */
struct WallLoopClosure
{
    bool                         hasOpenBoundary = true;
    std::vector<Eigen::Vector2d> corners_World_m;
};

bool hasSeparatingFiniteWall(
    const std::vector<geometric::Plane *> &wallList_World_in,
    const Eigen::Vector3d                 &firstPoint_World_m_in,
    const Eigen::Vector3d                 &secondPoint_World_m_in,
    const double                           finiteBoundsMargin_m_in);

WallAdmissionEvidence
    evaluateWallAdmissionEvidence(geometric::Plane          *p_wall_in,
                                  const types::SystemParams *p_systemParams_in,
                                  const Eigen::Vector3d &groundNormal_World_in);

double crossProduct2d(const Eigen::Vector2d &firstVector_in,
                      const Eigen::Vector2d &secondVector_in);

bool buildFiniteWallSegment2d(geometric::Plane      *p_wall_in,
                              const Eigen::Vector3d &groundNormal_World_in,
                              const Eigen::Vector3d &groundAxisU_World_in,
                              const Eigen::Vector3d &groundAxisV_World_in,
                              const double           endpointTrimRatio_in,
                              const double           minimumWallLength_m_in,
                              FiniteWallSegment2d   &segment_out);

bool intersectSupportingLines(const FiniteWallSegment2d &firstSegment_in,
                              const FiniteWallSegment2d &secondSegment_in,
                              Eigen::Vector2d &intersection_World_m_out,
                              double          &firstParameter_out,
                              double          &secondParameter_out);

double pointToSegmentDistance_m(const Eigen::Vector2d     &point_World_m_in,
                                const FiniteWallSegment2d &segment_in);

WallLoopClosure
    tryCloseWallLoop(std::vector<FiniteWallSegment2d> wallSegments_in,
                     const Eigen::Vector2d           &roomCentroid_Ground_m_in,
                     const types::SystemParams::RoomSeg::BoundaryTopology
                         &topologyParameters_in);

std::vector<semantic::Room::ObservationGap> computeRoomObservationGaps(
    const std::vector<FiniteWallSegment2d> &wallSegments_in,
    const Eigen::Vector2d                  &roomCentroid_Ground_m_in,
    /* An axis-aligned (or any) rectangle's four wall midpoints sit exactly
     * on its principal axes as seen from the centroid -- always exactly 90
     * deg apart by construction, regardless of aspect ratio. The threshold
     * must clear that deterministic case with margin, or every well-formed
     * rectangular room reports four phantom gaps. */
    double gapThreshold_rad_in = 100.0 * M_PI / 180.0);

double computePolygonArea_m2(
    const std::vector<Eigen::Vector2d> &polygonVertices_World_m_in);

bool segmentCrossesAperture(const Eigen::Vector3d &segmentStart_World_m_in,
                            const Eigen::Vector3d &segmentEnd_World_m_in,
                            const Eigen::Vector4d &apertureEquation_World_in,
                            const Eigen::Vector3d &apertureCentroid_World_m_in,
                            const double           apertureWidth_m_in,
                            const double           apertureHeight_m_in,
                            const Eigen::Vector3d &groundNormal_World_in,
                            const double           openingMargin_m_in,
                            const double           minimumSideDistance_m_in);

bool segmentCrossesPassageOpening(
    const Eigen::Vector3d &segmentStart_World_m_in,
    const Eigen::Vector3d &segmentEnd_World_m_in,
    semantic::Passage     *p_passage_in,
    const Eigen::Vector3d &groundNormal_World_in,
    const double           openingMargin_m_in,
    const double           minimumSideDistance_m_in,
    const bool             requirePassable_in = true);

bool segmentCrossesOpenPassageEvidence(
    const Eigen::Vector3d &segmentStart_World_m_in,
    const Eigen::Vector3d &segmentEnd_World_m_in,
    geometric::Plane      *p_evidenceSupportingWall_in,
    const Eigen::Vector3d &evidenceCentroid_World_m_in,
    const double           evidenceOpeningRadius_m_in,
    const double           evidenceHeightSpan_m_in,
    const Eigen::Vector3d &groundNormal_World_in,
    const double           openingMargin_m_in,
    const double           minimumSideDistance_m_in);

bool segmentCrossesForeignWall(
    const Eigen::Vector3d &segmentStart_World_m_in,
    const Eigen::Vector3d &segmentEnd_World_m_in,
    const std::vector<vs_graphs::core::semantic::Room *> &excludedRooms_in,
    const std::vector<vs_graphs::core::semantic::Room *> &allRooms_in,
    const Eigen::Vector3d                                &groundAxisU_World_in,
    const Eigen::Vector3d                                &groundAxisV_World_in,
    const Eigen::Vector3d                                &groundNormal_World_in,
    const double                                          endpointTrimRatio_in,
    const double minimumWallLength_m_in);

bool sharesRoomNameTag(Map *p_firstMap_in, Map *p_secondMap_in);

bool projectPlaneFootprintOntoSharedAxes(geometric::Plane      *p_plane_in,
                                         const Eigen::Vector3d &axisU_World_in,
                                         const Eigen::Vector3d &axisV_World_in,
                                         double                &minU_m_out,
                                         double                &maxU_m_out,
                                         double                &minV_m_out,
                                         double                &maxV_m_out);

bool arePlausibleTwinWallFaces(geometric::Plane      *p_first_in,
                               geometric::Plane      *p_second_in,
                               double                 minimumThickness_m_in,
                               double                 maximumThickness_m_in,
                               double                 minimumOverlapRatio_in,
                               const Eigen::Vector3d &groundNormal_World_in);

} // namespace core
} // namespace vs_graphs

#endif /* VS_GRAPHS_CORE_SEMANTICSMANAGER_PRIVATE_FUNCTIONS_H */

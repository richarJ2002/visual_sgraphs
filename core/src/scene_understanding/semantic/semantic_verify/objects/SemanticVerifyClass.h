/*!
 * @file            SemanticVerifyClass.h
 *
 * @brief           Declares semantic verification: the checks and evidence used
 *                  to accept or reject room merges and wall observations.
 */

/*!
 * @brief           Declares the deterministic, plane-gated geometric
 *                  verifier.
 *
 *                  Consumes a SemanticCandidate's two rooms (already
 *                  resolved by the caller to per-wall observations via
 *                  verified Room/Plane getters) and either produces a
 *                  verified inter-map SE(3) transform with an inlier
 *                  wall set, or rejects. This is verification only: it
 *                  mutates nothing, assigns no room tag, and triggers
 *                  no map merge.
 */
#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_SEMANTIC_VERIFY_CLASS_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_SEMANTIC_VERIFY_CLASS_H

#include "scene_understanding/semantic/room/objects/RoomContextSnapshotStruct.h"
#include "scene_understanding/semantic/semantic_verify/objects/SemanticMergeDecisionEnum.h"
#include "scene_understanding/semantic/semantic_verify/objects/SemanticMergeGateResultStruct.h"
#include "scene_understanding/semantic/semantic_verify/objects/SemanticMergeReasonEnum.h"
#include "scene_understanding/semantic/semantic_verify/objects/SemanticMergeRoomEvidenceStruct.h"
#include "scene_understanding/semantic/semantic_verify/objects/SemanticVerifyConfigStruct.h"
#include "scene_understanding/semantic/semantic_verify/objects/SemanticVerifyResultStatusEnum.h"
#include "scene_understanding/semantic/semantic_verify/objects/SemanticVerifyResultStruct.h"
#include "scene_understanding/semantic/semantic_verify/objects/SemanticVerifyStatusEnum.h"
#include "scene_understanding/semantic/semantic_verify/objects/VerifyRejectReasonEnum.h"
#include "scene_understanding/semantic/semantic_verify/objects/VerifyWallObservationStruct.h"
#include "scene_understanding/semantic/semantic_verify/objects/WallInlierPairStruct.h"
#include <vs_graphs_g2o/types/types_seven_dof_expmap.h>

#include <Eigen/Geometry>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace vs_graphs::core::map
{
class Map;
}

namespace vs_graphs
{
namespace core
{

namespace semantic
{
class Room;

/*!
 * @brief           Static functions that decide whether semantic evidence
 *                  (walls, passages, floors) shows two rooms or two maps are
 *                  the same place. Holds no state.
 */
class SemanticVerify
{
  public:
    /*!
     * @brief           Builds a SemanticVerifyConfig from the loaded
     *                  SystemParams::Verification / Factor YAML fields
     *                  (SystemParams::getParams() must already have been
     *                  populated via SystemParams::setParams()). This is the
     *                  only place those fields are read into a
     *                  SemanticVerifyConfig: SemanticVerifyConfig's own
     *                  default-member-initialisers are literal fallbacks for
     *                  callers that construct one directly (as every current
     *                  test does), not a live link to the YAML file, so calling
     *                  this explicitly is required to make an operator's
     *                  config-file edits take effect.
     */
    [[nodiscard]] static SemanticVerifyStatus
        configFromSystemParams(SemanticVerifyConfig &configuration_out);

    /*!
     * @brief           Collects wall observations for one room via verified
     *                  getters only; bounded by config.maxWallsPerRoom /
     *                  maxSupportSamplePerWall.
     */
    [[nodiscard]] static SemanticVerifyStatus collectWallObservations(
        const Room                         *p_room_in,
        const SemanticVerifyConfig         &configuration_in,
        std::vector<VerifyWallObservation> &observations_out);

    /*!
     * @brief           Reads the floor plane of a room's floor, oriented so the
     *                  room centroid lies on its positive side.
     *
     * @param[in]       p_room_in
     *                  Room whose floor is read; may be nullptr.
     *
     * @param[out]      floorPlane_world_out
     *                  Unit normal and offset (n^T x + d = 0) in the room's
     *                  map world frame, or empty when the room has no floor
     *                  with a plane identity.
     *
     * @return          SEMANTIC_VERIFY_STATUS_SUCCESS always.
     */
    [[nodiscard]] static SemanticVerifyStatus collectFloorObservation(
        const Room                     *p_room_in,
        std::optional<Eigen::Vector4d> &floorPlane_world_out);

    /*!
     * @brief           Core verifier. Rooms are already resolved to wall
     *                  observations by the caller; no Atlas/Map lookups happen
     *                  here. Deterministic: no locks held, no randomness,
     *                  stable hypothesis enumeration order. With both floor
     *                  planes given, a hypothesis needs two wall pairs plus the
     *                  floor pair, which fixes the vertical offset that
     *                  vertical walls cannot; without it, three wall pairs.
     *
     * @param[in]       wallsA_in
     *                  Walls of room A in room A's map world frame.
     *
     * @param[in]       wallsB_in
     *                  Walls of room B in room B's map world frame.
     *
     * @param[out]      result_out
     *                  Verdict, transform (A into B) and diagnostics.
     *
     * @param[in]       configuration_in
     *                  Thresholds.
     *
     * @param[in]       floorPlaneA_in
     *                  Floor plane of room A in its map world frame, or empty.
     *
     * @param[in]       floorPlaneB_in
     *                  Floor plane of room B in its map world frame, or empty.
     *
     * @return          SEMANTIC_VERIFY_STATUS_SUCCESS always.
     */
    [[nodiscard]] static SemanticVerifyStatus
        verify(const std::vector<VerifyWallObservation> &wallsA_in,
               const std::vector<VerifyWallObservation> &wallsB_in,
               SemanticVerifyResult                     &result_out,
               const SemanticVerifyConfig               &configuration_in,
               const std::optional<Eigen::Vector4d>     &floorPlaneA_in,
               const std::optional<Eigen::Vector4d>     &floorPlaneB_in);

    /*!
     * @brief           Floor gate wrapper: caller supplies the live Maps
     *                  (SemanticVerify itself never looks up Atlas/Map state).
     *                  Thin adapter over the shared verifyLoopMergeFloors
     *                  (LoopClosing.h) so both the legacy merge path and this
     *                  phase use the exact same floor-identity check. Writes
     *                  its outcome onto result_inout's
     *                  hasFloorGateRun/hasFloorGatePassed/floorGateResult
     *                  fields -- these are exactly what toVerificationVerdict()
     *                  combines with the geometric pass/fail, so calling this
     *                  (instead of the standalone verifyLoopMergeFloors) is
     *                  what makes that combination correct: without it,
     *                  hasFloorGatePassed stays at its default false and
     *                  toVerificationVerdict() reports a false negative even
     *                  when both gates genuinely passed.
     */
    [[nodiscard]] static SemanticVerifyStatus runFloorGate(
        SemanticVerifyResult    &result_inout,
        map::Map                *p_survivingMap_in,
        map::Map                *p_absorbedMap_in,
        const Eigen::Isometry3d &mergeTransform_absorbedToSurviving_in,
        bool                    &hasPassed_out);

    /*!
     * @brief           Evaluates copied room evidence under a proposed
     *                  absorbed-to-surviving map transform. Only the anchor
     *                  rooms (same room tag on both sides) are matched by
     *                  identity; walls, floors, rooms and passages are paired
     *                  by geometry in the surviving frame. A contradiction
     *                  between paired elements (an anchor floor or wall
     *                  offset, one element paired twice, paired passages that
     *                  disagree on being passable) rejects. Too little
     *                  evidence (no anchor pair, no anchor floor, too few
     *                  anchor wall pairs or directions) defers. Elements seen
     *                  in one map only neither accept nor reject. The pairs
     *                  and problems behind the decision are written to
     *                  result_out.pairDetail.
     */
    [[nodiscard]] static SemanticVerifyStatus evaluateMergeAlignment(
        const std::vector<SemanticMergeRoomEvidence> &survivingRooms_in,
        const std::vector<SemanticMergeRoomEvidence> &absorbedRooms_in,
        const g2o::Sim3            &mergeTransform_absorbedToSurviving_in,
        SemanticMergeGateResult    &result_out,
        const SemanticVerifyConfig &configuration_in = SemanticVerifyConfig());

    /*!
     * @brief           Runs the complete floor and semantic gate against two
     *                  live maps. Caller must prevent concurrent semantic
     *                  mutation for both maps.
     */
    [[nodiscard]] static SemanticVerifyStatus evaluateMapMergeGate(
        map::Map                   *p_survivingMap_in,
        map::Map                   *p_absorbedMap_in,
        const g2o::Sim3            &mergeTransform_absorbedToSurviving_in,
        SemanticMergeGateResult    &result_out,
        const SemanticVerifyConfig &configuration_in = SemanticVerifyConfig());

    /*!
     * @brief           Returns a stable parseable name for a merge decision.
     */
    [[nodiscard]] static SemanticVerifyStatus
        mergeDecisionName(SemanticMergeDecision decision_in,
                          const char          *&p_name_out);

    /*!
     * @brief           Returns a stable parseable name for a merge reason.
     */
    [[nodiscard]] static SemanticVerifyStatus
        mergeReasonName(SemanticMergeReason reason_in, const char *&p_name_out);

    /*!
     * @brief           Returns a stable parseable name for a verify() reject
     *                  reason.
     *
     * @param[in]       reason_in
     *                  Reject reason.
     *
     * @param[out]      p_name_out
     *                  Static name, never nullptr.
     *
     * @return          SEMANTIC_VERIFY_STATUS_SUCCESS always.
     */
    [[nodiscard]] static SemanticVerifyStatus
        verifyRejectReasonName(VerifyRejectReason reason_in,
                               const char       *&p_name_out);
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* SEMANTIC_VERIFY_H */

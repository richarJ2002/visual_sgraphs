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

#include <vs_graphs_g2o/types/types_seven_dof_expmap.h>
#include "semantic/room/objects/RoomContextSnapshotStruct.h"
#include "semantic/semantic_verify/objects/SemanticMergeDecisionEnum.h"
#include "semantic/semantic_verify/objects/SemanticMergeGateResultStruct.h"
#include "semantic/semantic_verify/objects/SemanticMergeReasonEnum.h"
#include "semantic/semantic_verify/objects/SemanticMergeRoomEvidenceStruct.h"
#include "semantic/semantic_verify/objects/SemanticVerifyConfigStruct.h"
#include "semantic/semantic_verify/objects/SemanticVerifyResultStatusEnum.h"
#include "semantic/semantic_verify/objects/SemanticVerifyResultStruct.h"
#include "semantic/semantic_verify/objects/SemanticVerifyStatusEnum.h"
#include "semantic/semantic_verify/objects/VerifyRejectReasonEnum.h"
#include "semantic/semantic_verify/objects/VerifyWallObservationStruct.h"
#include "semantic/semantic_verify/objects/WallInlierPairStruct.h"

#include <Eigen/Geometry>

#include <cstddef>
#include <string>
#include <vector>

namespace vs_graphs::core::map { class Map; }

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
     * @brief           Core verifier. Rooms are already resolved to wall
     *                  observations by the caller; no Atlas/Map lookups happen
     *                  here. Deterministic: no locks held, no randomness,
     *                  stable hypothesis enumeration order.
     */
    [[nodiscard]] static SemanticVerifyStatus verify(
        const std::vector<VerifyWallObservation> &wallsA_in,
        const std::vector<VerifyWallObservation> &wallsB_in,
        SemanticVerifyResult                     &result_out,
        const SemanticVerifyConfig &configuration_in = SemanticVerifyConfig());

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
        map::Map                     *p_survivingMap_in,
        map::Map                     *p_absorbedMap_in,
        const Eigen::Isometry3d &mergeTransform_absorbedToSurviving_in,
        bool                    &hasPassed_out);

    /*!
     * @brief           Evaluates copied room evidence under a proposed
     *                  absorbed-to-surviving map transform. Contradictory
     *                  stable topology rejects, incomplete evidence defers, and
     *                  only mutually consistent walls and passage topology
     *                  accept.
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
        map::Map                        *p_survivingMap_in,
        map::Map                        *p_absorbedMap_in,
        const g2o::Sim3            &mergeTransform_absorbedToSurviving_in,
        SemanticMergeGateResult    &result_out,
        const SemanticVerifyConfig &configuration_in = SemanticVerifyConfig());

    /*!
     * @brief           Tolerances for consecutive-map (post-reset) merge
     *                  validation. Sourced from the mapMerge SystemParams
     *                  section via mapMergeConfigFromSystemParams(); defaults
     *                  mirror the YAML.
     *
     *                  Companion checks intentionally reuse shared constants so
     *                  the consecutive path cannot drift from loop closure:
     *                  wall offset prefilter (0.35 m), passage direction
     *                  agreement (0.85) and the wall core inlier rules come
     *                  from the verification defaults, and the floor normal
     *                  gate is shared. Rooms pair by non-empty tag only;
     *                  untagged rooms contribute no passage constraints (defer,
     *                  never guess).
     */
    struct MapMergeConfig
    {
        /*!
         * @brief           Largest distance between a mapped absorbed passage
         *                  centroid and its surviving partner, metres.
         */
        double passage_match_tolerance_m{0.20};
        /*!
         * @brief           Largest angle between a mapped absorbed wall normal
         *                  and a surviving wall normal for the walls to be
         *                  coplanar, degrees.
         */
        double wall_coplanar_angle_deg{5.0};
        /*!
         * @brief           Smallest overlap along the wall direction that
         *                  paired walls must have, metres.
         */
        double wall_edge_overlap_m{0.50};
        /*!
         * @brief           Largest floor plane offset difference after the
         *                  transform, metres.
         */
        double floor_match_tolerance_m{0.10};
        /*!
         * @brief           Largest distance between a mapped absorbed anchor
         *                  room centroid and its tag-matched surviving partner,
         *                  metres.
         */
        double room_centroid_tolerance_m{0.50};
    };

    /*!
     * @brief           Builds a MapMergeConfig from the live SystemParams
     *                  mapMerge section. Falls back to the struct defaults when
     *                  SystemParams is unavailable.
     */
    [[nodiscard]] static SemanticVerifyStatus mapMergeConfigFromSystemParams(
        SemanticVerify::MapMergeConfig &configuration_out);

    /*!
     * @brief           Consecutive-map variant of evaluateMapMergeGate: same
     *                  decision vocabulary, but wall/passage/floor/anchor-room
     *                  tolerances come from MapMergeConfig (mapMerge params)
     *                  instead of SemanticVerifyConfig, rooms pair by non-empty
     *                  room tag only (never by map-local ID), and wall pairs
     *                  must additionally overlap along the wall direction.
     *                  Verification only: mutates nothing.
     */
    [[nodiscard]] static SemanticVerifyStatus evaluateConsecutiveMergeGate(
        map::Map                     *p_survivingMap_in,
        map::Map                     *p_absorbedMap_in,
        const g2o::Sim3         &mergeTransform_absorbedToSurviving_in,
        SemanticMergeGateResult &result_out,
        const MapMergeConfig    &configuration_in);

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
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* SEMANTIC_VERIFY_H */

/*!
 * @file            SemanticVerify.h
 *
 * @brief           Declares semantic verification: the checks and evidence used
 *                  to accept or reject room merges and wall observations.
 */

/*!
 * @brief        Declares the deterministic, plane-gated geometric
 *               verifier.
 *
 *               Consumes a SemanticCandidate's two rooms (already
 *               resolved by the caller to per-wall observations via
 *               verified Room/Plane getters) and either produces a
 *               verified inter-map SE(3) transform with an inlier
 *               wall set, or rejects. This is verification only: it
 *               mutates nothing, assigns no room tag, and triggers
 *               no map merge.
 */
#ifndef SEMANTIC_VERIFY_H
#define SEMANTIC_VERIFY_H

#include "Semantic/RoomContextSnapshot.h"
#include "Semantic/RoomTracker.h"
#include "Semantic/SemanticVerifyResultStatus.h"
#include "Semantic/SemanticVerifyStatus.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include <Eigen/Geometry>

#include <cstddef>
#include <string>
#include <vector>

namespace vs_graphs
{
namespace core
{
class Map;
namespace semantic
{
class Room;

/*! Pre-mutation decision for a proposed physical map merge. */
enum class SemanticMergeDecision
{
    ACCEPT,
    DEFER,
    REJECT
};

/*! Typed reason for a semantic merge-gate decision. */
enum class SemanticMergeReason
{
    ALIGNED,
    INVALID_INPUT,
    FLOOR_EVIDENCE_MISSING,
    FLOOR_CONTRADICTION,
    SHARED_ROOM_IDENTITY_MISSING,
    WALL_EVIDENCE_MISSING,
    WALL_ALIGNMENT_CONTRADICTION,
    PASSAGE_EVIDENCE_MISSING,
    PASSAGE_IDENTITY_CONTRADICTION,
    PASSAGE_ENDPOINT_CONTRADICTION,
    PASSAGE_DIRECTION_CONTRADICTION
};

/*! One wall's plane equation and centroid, already resolved through verified
 * getters (Room::getWalls(), Room::getWallNormalTowardRoom_world(),
 * Plane::getGlobalEquation(), Plane::getCentroid(),
 * Plane::getGeometrySnapshot()) and expressed in the SOURCE room's own map
 * frame. Never compared raw against another map's frame before a hypothesis
 * transform exists. */
struct VerifyWallObservation
{
    /*!
     * @brief        Identifier of the wall plane this observation was taken
     *               from (geometric::Plane::getId()). WallInlierPair refers
     *               back to observations through it.
     */
    int wallId{0};

    /*! Canonically oriented via getWallNormalTowardRoom_world
     * (n.c_room + d > 0). */
    Eigen::Vector3d wallNormal_world{Eigen::Vector3d::Zero()};

    /*! Plane equation offset paired with wallNormal_world: getGlobalEquation()
     * .coeffs()(3), i.e. n^T x + d = 0 -- NOT g2o::Plane3D::distance(),
     * which returns -d (Plane::transformPlaneEquation, Plane.cc:441-442). */
    double d{0.0};

    /*!
     * @brief        Centroid of the wall plane in the source room's map world
     *               frame, metres (Plane::getCentroid()).
     */
    Eigen::Vector3d wallCentroid_world{Eigen::Vector3d::Zero()};

    /*! Bounded, deterministic stride-sampled points from
     * Plane::getGeometrySnapshot().supportCloud. */
    std::vector<Eigen::Vector3d> supportSample_world;
};

/*!
 * @brief        Thresholds and weights that decide when two rooms' wall sets
 *               are the same place. Defaults are fallbacks;
 *               configFromSystemParams() reads the operator's values.
 */
struct SemanticVerifyConfig
{
    /*!
     * @brief        Largest angle between a transformed wall normal and its
     *               partner's normal for the pair to count as an inlier,
     *               degrees.
     */
    double       maxNormalAngle_deg{10.0};
    /*!
     * @brief        Largest difference between a transformed wall plane offset
     *               and its partner's offset for an inlier, metres.
     */
    double       maxOffset_m{0.35};
    /*!
     * @brief        Largest symmetric distance between the support samples of
     *               two paired walls for an inlier, metres.
     */
    double       maxSupportDist_m{0.25};
    /*!
     * @brief        Smallest fraction of walls that must be inliers for a
     *               hypothesis to pass, dimensionless.
     */
    double       minInlierRatio{0.6};
    /*!
     * @brief        Largest condition number of the translation system a
     *               hypothesis may have; also scales the normalised condition
     *               number.
     */
    double       maxConditionNumber{100.0};
    /*!
     * @brief        Number of inliers the best hypothesis must exceed the
     *               runner-up by, otherwise the result is rejected as
     *               ambiguous.
     */
    unsigned int ambiguityMarginInliers{1U};
    /*!
     * @brief        Largest number of walls collected from one room; further
     *               walls are ignored.
     */
    std::size_t  maxWallsPerRoom{16U};
    /*!
     * @brief        Largest number of hypotheses verify() evaluates before it
     *               stops enumerating.
     */
    std::size_t  maxHypotheses{2000U};
    /*!
     * @brief        Largest number of support points kept per wall.
     */
    std::size_t  maxSupportSamplePerWall{64U};
    /*!
     * @brief        Explicit |cos(theta)| gate, distinct from
     *               maxNormalAngle_deg.
     */
    double       minAbsCosNormalAngle{0.85};
    /*!
     * @brief        Standard deviation of the normal angle residual that
     *               weights the refinement, radians.
     */
    double       sigmaTheta_rad{0.05};
    /*!
     * @brief        Standard deviation of the plane offset residual that
     *               weights the refinement, metres.
     */
    double       sigmaOffset_m{0.05};
    /*!
     * @brief        Huber kernel threshold of the refinement, in weighted
     *               residual units.
     */
    double       huberDelta{1.345};
    /*!
     * @brief        Number of refinement iterations the optimizer runs.
     */
    unsigned int optimizerIterations{20U};
};

/*!
 * @brief        One pair of walls, one from each room, that agrees under the
 *               transform found by verify().
 */
struct WallInlierPair
{
    /*!
     * @brief        Identifier of the paired wall in room A
     *               (VerifyWallObservation::wallId).
     */
    int    wallIdA{0};
    /*!
     * @brief        Identifier of the paired wall in room B
     *               (VerifyWallObservation::wallId).
     */
    int    wallIdB{0};
    /*!
     * @brief        Angle between room A's wall normal, rotated into room B's
     *               frame, and room B's wall normal, radians.
     */
    double normalAngleResidual_rad{0.0};
    /*!
     * @brief        Absolute difference between room A's wall offset, moved
     *               into room B's frame, and room B's wall offset, metres.
     */
    double offsetResidual_m{0.0};
    /*!
     * @brief        Symmetric distance between the support samples of the two
     *               walls under the transform, metres.
     */
    double supportDistResidual_m{0.0};
};

/*! Copied evidence for one room. No live map pointer escapes the merge lock. */
struct SemanticMergeRoomEvidence
{
    /*!
     * @brief        Snapshot of the room's identity, floor, centroid and
     *               passages.
     */
    RoomContextSnapshot                context;
    /*!
     * @brief        Wall observations of the room, in its own map's world
     *               frame.
     */
    std::vector<VerifyWallObservation> walls;
};

/*! Complete, reason-coded result of the physical map-merge semantic gate. */
struct SemanticMergeGateResult
{
    /*!
     * @brief        Outcome of the gate: accept, defer or reject the merge.
     */
    SemanticMergeDecision decision{SemanticMergeDecision::DEFER};
    /*!
     * @brief        Why the gate chose its decision.
     */
    SemanticMergeReason   reason{
        SemanticMergeReason::SHARED_ROOM_IDENTITY_MISSING};
    /*!
     * @brief        Floor check outcome text: "ACCEPTED", "REJECTED",
     *               "DEFERRED" or empty when the floor check did not run.
     */
    std::string floorDecision;
    /*!
     * @brief        Number of rooms found on both sides of the merge.
     */
    std::size_t sharedRoomCount{0U};
    /*!
     * @brief        Number of shared rooms whose walls and passages agree under
     *               the proposed transform.
     */
    std::size_t alignedRoomCount{0U};
    /*!
     * @brief        Total number of wall pairs matched across the shared rooms.
     */
    std::size_t matchedWallCount{0U};
    /*!
     * @brief        Total number of passage pairs matched across the shared
     *               rooms.
     */
    std::size_t matchedPassageCount{0U};
};

/*! Identifies which of verify()'s distinct early-return gates produced a
 *  REJECTED result. Diagnostic-only, added because every rejection path
 *  used to leave SemanticVerifyResult's numeric fields (inlierRatio, rank,
 *  conditionNumber, ...) at their zero defaults -- indistinguishable in a
 *  log line from "genuinely computed as zero". See the fields' own comments
 *  below for which reason populates which fields. */
enum class VerifyRejectReason
{
    NONE, // PASS, or verify() has not returned yet.
    TOO_FEW_WALLS,
    NO_VALID_HYPOTHESIS,
    AMBIGUOUS_TOP_HYPOTHESES,
    BELOW_MIN_INLIER_RATIO,
    REFINED_FIT_NOT_OBSERVABLE
};

/*!
 * @brief        Outcome of verify(): whether two rooms' wall sets are the same
 *               place, the transform found and diagnostics of the search.
 */
struct SemanticVerifyResult
{
    /*!
     * @brief        Verdict: PASS, REJECTED, or UNAVAILABLE when verify() had
     *               no verdict.
     */
    VerificationStatus status{VerificationStatus::UNAVAILABLE};
    /*!
     * @brief        True only when status is PASS.
     */
    bool               hasPassed{false};

    /*! Maps room-A-frame points into room B's frame: x_B = R x_A + t. */
    Eigen::Isometry3d roomTransform_roomAToRoomB{Eigen::Isometry3d::Identity()};

    /*!
     * @brief        Wall pairs that agree under the best hypothesis; empty
     *               unless the status is PASS.
     */
    std::vector<WallInlierPair> inliers;
    /*!
     * @brief        Number of wall pairs that could be formed: wall count of
     *               room A times wall count of room B.
     */
    std::size_t                 candidateWallPairCount{0U};
    /*!
     * @brief        Rank of the translation system of the refined fit (3 =
     *               fully determined); 0 when it was not computed.
     */
    std::size_t                 rank{0U};
    /*!
     * @brief        Condition number of the translation system of the refined
     *               fit; 0 when it was not computed.
     */
    double                      conditionNumber{0.0};
    /*!
     * @brief        conditionNumber divided by maxConditionNumber, capped at 1
     *               (1 when the condition number is not finite); set only on
     *               PASS.
     */
    double                      normalisedConditionNumber{0.0};
    /*!
     * @brief        Inliers of the best hypothesis divided by the wall count of
     *               the room with fewer walls; 0 before hypotheses were ranked.
     */
    double                      inlierRatio{0.0};
    /*!
     * @brief        Median normal angle residual over the inliers, radians; set
     *               only on PASS.
     */
    double                      angularResidual_rad{0.0};
    /*!
     * @brief        Score in [0, 1]: inlierRatio reduced by up to half through
     *               normalisedConditionNumber; set only on PASS.
     */
    double                      confidence{0.0};

    /*! Diagnostic-only fields, populated on every exit path (see
     *  VerifyRejectReason). NONE on PASS. */
    VerifyRejectReason rejectReason{VerifyRejectReason::NONE};
    /*! Best distinct hypothesis's inlier count, set from
     *  AMBIGUOUS_TOP_HYPOTHESES onward (i.e. once at least one valid
     *  hypothesis existed to rank). 0 for TOO_FEW_WALLS/NO_VALID_HYPOTHESIS,
     *  which never reach hypothesis ranking. */
    std::size_t        topInlierCount{0U};
    /*! Second-best distinct hypothesis's inlier count, same availability as
     *  topInlierCount. */
    std::size_t        runnerUpInlierCount{0U};

    /*!
     * @brief        True once runFloorGate() has been called on this result.
     */
    bool        hasFloorGateRun{false};
    /*!
     * @brief        True when the floor gate accepted the floors; meaningful
     *               only when hasFloorGateRun is true.
     */
    bool        hasFloorGatePassed{false};
    /*!
     * @brief        Floor gate outcome text: "ACCEPTED", "REJECTED", "DEFERRED"
     *               or empty when the gate has not run.
     */
    std::string floorGateResult;

    /*!
     * @brief        Converts to the RoomTracker verdict type this
     *               verifier exists to eventually feed
     *               (SemanticsManager::submitVerificationVerdict()'s
     *               consumer). Wiring that call site is out of scope
     *               here.
     */
    [[nodiscard]] SemanticVerifyResultStatus toVerificationVerdict(
        VerificationVerdict &verificationVerdict_out) const;
};

/*!
 * @brief        Static functions that decide whether semantic evidence (walls,
 *               passages, floors) shows two rooms or two maps are the same
 *               place. Holds no state.
 */
class SemanticVerify
{
  public:
    /*! Builds a SemanticVerifyConfig from the loaded SystemParams::Verification
     * / Factor YAML fields (SystemParams::getParams() must already have been
     * populated via SystemParams::setParams()). This is the only place those
     * fields are read into a SemanticVerifyConfig: SemanticVerifyConfig's
     * own default-member-initialisers are literal fallbacks for callers that
     * construct one directly (as every current test does), not a live link
     * to the YAML file, so calling this explicitly is required to make an
     * operator's config-file edits take effect. */
    [[nodiscard]] static SemanticVerifyStatus
        configFromSystemParams(SemanticVerifyConfig &configuration_out);

    /*! Collects wall observations for one room via verified getters only;
     * bounded by config.maxWallsPerRoom / maxSupportSamplePerWall. */
    [[nodiscard]] static SemanticVerifyStatus collectWallObservations(
        const Room                         *p_room_in,
        const SemanticVerifyConfig         &configuration_in,
        std::vector<VerifyWallObservation> &observations_out);

    /*! Core verifier. Rooms are already resolved to wall observations by the
     * caller; no Atlas/Map lookups happen here. Deterministic: no locks
     * held, no randomness, stable hypothesis enumeration order. */
    [[nodiscard]] static SemanticVerifyStatus verify(
        const std::vector<VerifyWallObservation> &wallsA_in,
        const std::vector<VerifyWallObservation> &wallsB_in,
        SemanticVerifyResult                     &result_out,
        const SemanticVerifyConfig &configuration_in = SemanticVerifyConfig());

    /*! Floor gate wrapper: caller supplies the live Maps (SemanticVerify
     * itself never looks up Atlas/Map state). Thin adapter over the shared
     * verifyLoopMergeFloors (LoopClosing.h) so both the legacy merge path and
     * this phase use the exact same floor-identity check. Writes its outcome
     * onto result_inout's hasFloorGateRun/hasFloorGatePassed/floorGateResult
     * fields -- these are exactly what toVerificationVerdict() combines with
     * the geometric pass/fail, so calling this (instead of the standalone
     * verifyLoopMergeFloors) is what makes that combination correct: without
     * it, hasFloorGatePassed stays at its default false and
     * toVerificationVerdict() reports a false negative even when both gates
     * genuinely passed. */
    [[nodiscard]] static SemanticVerifyStatus runFloorGate(
        SemanticVerifyResult    &result_inout,
        Map                     *p_survivingMap_in,
        Map                     *p_absorbedMap_in,
        const Eigen::Isometry3d &mergeTransform_absorbedToSurviving_in,
        bool                    &hasPassed_out);

    /*! Evaluates copied room evidence under a proposed absorbed-to-surviving
     * map transform. Contradictory stable topology rejects, incomplete
     * evidence defers, and only mutually consistent walls and passage
     * topology accept. */
    [[nodiscard]] static SemanticVerifyStatus evaluateMergeAlignment(
        const std::vector<SemanticMergeRoomEvidence> &survivingRooms_in,
        const std::vector<SemanticMergeRoomEvidence> &absorbedRooms_in,
        const g2o::Sim3            &mergeTransform_absorbedToSurviving_in,
        SemanticMergeGateResult    &result_out,
        const SemanticVerifyConfig &configuration_in = SemanticVerifyConfig());

    /*! Runs the complete floor and semantic gate against two live maps.
     * Caller must prevent concurrent semantic mutation for both maps. */
    [[nodiscard]] static SemanticVerifyStatus evaluateMapMergeGate(
        Map                        *p_survivingMap_in,
        Map                        *p_absorbedMap_in,
        const g2o::Sim3            &mergeTransform_absorbedToSurviving_in,
        SemanticMergeGateResult    &result_out,
        const SemanticVerifyConfig &configuration_in = SemanticVerifyConfig());

    /*! Tolerances for consecutive-map (post-reset) merge validation. Sourced
     * from the mapMerge SystemParams section via
     * mapMergeConfigFromSystemParams(); defaults mirror the YAML.
     *
     * Companion checks intentionally reuse shared constants so the
     * consecutive path cannot drift from loop closure: wall offset
     * prefilter (0.35 m), passage direction agreement (0.85) and the wall
     * core inlier rules come from the verification defaults, and the floor
     * normal gate is shared. Rooms pair by non-empty tag only; untagged
     * rooms contribute no passage constraints (defer, never guess). */
    struct MapMergeConfig
    {
        /*!
         * @brief        Largest distance between a mapped absorbed passage (or
         *               anchor room) centroid and its surviving partner,
         *               metres.
         */
        double passage_match_tolerance_m{0.20};
        /*!
         * @brief        Largest angle between a mapped absorbed wall normal and
         *               a surviving wall normal for the walls to be coplanar,
         *               degrees.
         */
        double wall_coplanar_angle_deg{5.0};
        /*!
         * @brief        Smallest overlap along the wall direction that paired
         *               walls must have, metres.
         */
        double wall_edge_overlap_m{0.50};
        /*!
         * @brief        Largest floor plane offset difference after the
         *               transform, metres.
         */
        double floor_match_tolerance_m{0.10};
        /*!
         * @brief        Loaded from the mapMerge parameters; no check reads it
         *               yet.
         */
        double room_centroid_tolerance_m{0.50};
    };

    /*! Builds a MapMergeConfig from the live SystemParams mapMerge section.
     * Falls back to the struct defaults when SystemParams is unavailable. */
    [[nodiscard]] static SemanticVerifyStatus mapMergeConfigFromSystemParams(
        SemanticVerify::MapMergeConfig &configuration_out);

    /*! Consecutive-map variant of evaluateMapMergeGate: same decision
     * vocabulary, but wall/passage/floor tolerances come from MapMergeConfig
     * (mapMerge params) instead of SemanticVerifyConfig, rooms pair by
     * non-empty room tag only (never by map-local ID), and wall pairs must
     * additionally overlap along the wall direction. Verification only:
     * mutates nothing. */
    [[nodiscard]] static SemanticVerifyStatus evaluateConsecutiveMergeGate(
        Map                     *p_survivingMap_in,
        Map                     *p_absorbedMap_in,
        const g2o::Sim3         &mergeTransform_absorbedToSurviving_in,
        SemanticMergeGateResult &result_out,
        const MapMergeConfig    &configuration_in);

    /*! Returns a stable parseable name for a merge decision. */
    [[nodiscard]] static SemanticVerifyStatus
        mergeDecisionName(SemanticMergeDecision decision_in,
                          const char          *&p_name_out);

    /*! Returns a stable parseable name for a merge reason. */
    [[nodiscard]] static SemanticVerifyStatus
        mergeReasonName(SemanticMergeReason reason_in, const char *&p_name_out);
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_VERIFY_H

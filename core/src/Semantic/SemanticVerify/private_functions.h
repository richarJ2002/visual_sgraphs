/*!
 * @file            private_functions.h
 *
 * @brief           Declares module-internal helpers shared between the
 *                  SemanticVerify translation units.
 *
 * @note            These entities were file-scope members of the
 *                  anonymous namespace of SemanticVerify.cc;
 *                  external linkage here is module-internal only.
 */

#ifndef VS_GRAPHS_CORE_SEMANTIC_SEMANTICVERIFY_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_SEMANTIC_SEMANTICVERIFY_PRIVATE_FUNCTIONS_H

#include "Semantic/SemanticVerify.h"
#include "Semantic/SemanticVerifyStatus.h"

#include "Geometric/Plane.h"
#include "LoopClosing.h"
#include "Map.h"
#include "OptimizableTypes.h"
#include "Semantic/Room.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"
#include "Types/objects/SystemParams.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*! One admitted (indexA, indexB) candidate correspondence -- not yet a
 * hypothesis, just a pairing considered for 3-subset enumeration. */
struct CandidatePair
{
    /*!
     * @brief        Index of the wall in room A's observation list.
     */
    std::size_t indexA{0U};
    /*!
     * @brief        Index of the wall in room B's observation list.
     */
    std::size_t indexB{0U};
};

/*!
 * @brief        Rotation estimated from paired wall normals.
 */
struct RotationFit
{
    /*!
     * @brief        True when the fit succeeded and rotation may be used.
     */
    bool            valid{false};
    /*!
     * @brief        Rotation taking room A's wall normals onto room B's.
     */
    Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
};

/*!
 * @brief        Translation estimated from paired wall plane offsets.
 */
struct TranslationFit
{
    /*!
     * @brief        True when the fit succeeded and the other fields may be
     *               used.
     */
    bool            valid{false};
    /*!
     * @brief        Translation of the room A to room B transform, metres.
     */
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
    /*!
     * @brief        Rank of the normal matrix; 3 means the translation is fully
     *               determined.
     */
    std::size_t     rank{0U};
    /*!
     * @brief        Condition number of the normal matrix; infinite until
     *               computed.
     */
    double          conditionNumber{std::numeric_limits<double>::infinity()};
};

/*!
 * @brief        Result of one consistency check between two maps' evidence:
 *               evidence agrees, evidence is missing, or evidence contradicts.
 */
enum class AlignmentCheck
{
    ALIGNED,
    MISSING,
    CONTRADICTION
};

/*! @brief Consecutive-map anchor: one tag-matched room pair. Tags are the
 * only correspondence key; map-local IDs are never compared across maps. */
struct ConsecutiveAnchorPair
{
    /*!
     * @brief        Room of the surviving map in the pair; borrowed, never
     *               null.
     */
    const SemanticMergeRoomEvidence *p_surviving{nullptr};
    /*!
     * @brief        Room of the absorbed map with the same tag; borrowed, never
     *               null.
     */
    const SemanticMergeRoomEvidence *p_absorbed{nullptr};
};

/*! @brief Minimum known-side direction agreement for paired passages. Mirrors
 * the verification default (SemanticVerifyConfig::minAbsCosNormalAngle). */
constexpr double kConsecutiveMinDirectionAgreement = 0.85;

/*! @brief Maximum plane offset for wall pairing. Mirrors the verification
 * default (SemanticVerifyConfig::maxOffset_m); the mapMerge section tunes
 * the angle and the edge overlap, not the offset. */
constexpr double kConsecutiveMaxPlaneOffset_m = 0.35;

/*!
 * @brief        Tells whether a vector contains only finite numbers.
 *
 * @param[in]    value_in
 *               Vector to test.
 *
 * @param[out]   isFiniteVector_out
 *               True when all three components are finite.
 *
 * @return       Always SEMANTIC_VERIFY_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticVerifyStatus
    isFiniteVector(const Eigen::Vector3d &value_in, bool &isFiniteVector_out);

[[nodiscard]] SemanticVerifyStatus
    fitRotationFromNormals(const std::vector<Eigen::Vector3d> &normalsA_in,
                           const std::vector<Eigen::Vector3d> &normalsB_in,
                           RotationFit                        &rotation_out);

[[nodiscard]] SemanticVerifyStatus
    fitTranslation(const Eigen::Matrix3d              &rotation_in,
                   const std::vector<Eigen::Vector3d> &normalsA_in,
                   const std::vector<double>          &offsetsA_in,
                   const std::vector<Eigen::Vector3d> &normalsB_in,
                   const std::vector<double>          &offsetsB_in,
                   TranslationFit                     &translation_out);

/*!
 * @brief        Returns the angle between two vectors.
 *
 * @param[in]    first_in
 *               First vector; need not be unit length.
 *
 * @param[in]    second_in
 *               Second vector; need not be unit length.
 *
 * @param[out]   angle_rad_out
 *               Angle between the vectors, radians in [0, pi].
 *
 * @return       Always SEMANTIC_VERIFY_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticVerifyStatus
    angleBetween_rad(const Eigen::Vector3d &first_in,
                     const Eigen::Vector3d &second_in,
                     double                &angle_rad_out);

/*!
 * @brief        Finds the observation of a wall by its identifier.
 *
 * @param[in]    walls_in
 *               Wall observations to search.
 *
 * @param[in]    wallId_in
 *               Wall identifier to look for.
 *
 * @param[out]   p_byWallId_out
 *               First observation with that identifier, or nullptr when there
 *               is none. Points into walls_in.
 *
 * @return       Always SEMANTIC_VERIFY_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticVerifyStatus
    findByWallId(const std::vector<VerifyWallObservation> &walls_in,
                 const int                                 wallId_in,
                 const VerifyWallObservation             *&p_byWallId_out);

[[nodiscard]] SemanticVerifyStatus
    symmetricSupportDistance(const VerifyWallObservation &wallA_in,
                             const VerifyWallObservation &wallB_in,
                             const Eigen::Matrix3d       &rotation_in,
                             const Eigen::Vector3d       &translation_in,
                             double                      &distance_out);

/*!
 * @brief        Returns the name a room is known by across maps.
 *
 * @param[in]    context_in
 *               Room snapshot.
 *
 * @param[out]   identity_out
 *               The room tag, or "room_<id>" when the tag is empty.
 *
 * @return       Always SEMANTIC_VERIFY_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticVerifyStatus
    stableRoomIdentity(const RoomContextSnapshot &context_in,
                       std::string               &identity_out);

/*!
 * @brief        Counts the absorbed walls that coincide with a surviving wall
 *               under a given transform, without searching for a better one.
 *
 * @param[in]    survivingWalls_in
 *               Wall observations of the surviving room.
 *
 * @param[in]    absorbedWalls_in
 *               Wall observations of the absorbed room.
 *
 * @param[in]    transform_in
 *               Maps absorbed-map world points into the surviving map's world
 *               frame.
 *
 * @param[in]    configuration_in
 *               Angle, offset and inlier ratio thresholds.
 *
 * @param[out]   matchedCount_out
 *               Number of absorbed walls matched one-to-one to a surviving
 *               wall.
 *
 * @param[out]   alignmentCheck_out
 *               MISSING when either room has fewer than three walls; ALIGNED
 *               when at least three walls match and the matched fraction of the
 *               larger wall set reaches minInlierRatio; CONTRADICTION
 *               otherwise, including an unusable transform.
 *
 * @return       Always SEMANTIC_VERIFY_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticVerifyStatus checkFixedTransformWalls(
    const std::vector<VerifyWallObservation> &survivingWalls_in,
    const std::vector<VerifyWallObservation> &absorbedWalls_in,
    const g2o::Sim3                          &transform_in,
    const SemanticVerifyConfig               &configuration_in,
    std::size_t                              &matchedCount_out,
    AlignmentCheck                           &alignmentCheck_out);

/*!
 * @brief        Checks that the two rooms' passages, paired by passage id,
 *               agree on whether they are passable, which rooms they join and
 *               which way they face.
 *
 * @param[in]    survivingContext_in
 *               Snapshot of the surviving room.
 *
 * @param[in]    absorbedContext_in
 *               Snapshot of the absorbed room.
 *
 * @param[in]    transform_in
 *               Maps absorbed-map world points into the surviving map's world
 *               frame; only its rotation is used.
 *
 * @param[in]    configuration_in
 *               Supplies minAbsCosNormalAngle, the smallest direction
 *               agreement.
 *
 * @param[out]   matchedCount_out
 *               Number of absorbed passages whose id exists in the surviving
 *               room.
 *
 * @param[out]   contradictionReason_out
 *               Reason for a CONTRADICTION; untouched otherwise.
 *
 * @param[out]   alignmentCheck_out
 *               MISSING when either room has no passages or some evidence is
 *               incomplete; CONTRADICTION when passable flag, endpoint rooms or
 *               direction disagree, or no id matches; ALIGNED otherwise.
 *
 * @return       Always SEMANTIC_VERIFY_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticVerifyStatus
    checkPassageTopology(const RoomContextSnapshot  &survivingContext_in,
                         const RoomContextSnapshot  &absorbedContext_in,
                         const g2o::Sim3            &transform_in,
                         const SemanticVerifyConfig &configuration_in,
                         std::size_t                &matchedCount_out,
                         SemanticMergeReason        &contradictionReason_out,
                         AlignmentCheck             &alignmentCheck_out);

/*!
 * @brief        Copies one room's identity, walls and passages into plain data
 *               so no live map pointer leaves the merge lock.
 *
 * @param[in]    p_room_in
 *               Room to copy; must not be null. Borrowed. The caller holds the
 *               merge lock.
 *
 * @param[in]    configuration_in
 *               Limits for the wall observations.
 *
 * @param[out]   evidence_out
 *               Copy of the room's id, tag, centroid, floor id (-1 when it has
 *               no floor), walls and non-bad passages.
 *
 * @return       Always SEMANTIC_VERIFY_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticVerifyStatus
    copyMergeRoomEvidence(const Room                 *p_room_in,
                          const SemanticVerifyConfig &configuration_in,
                          SemanticMergeRoomEvidence  &evidence_out);

/*!
 * @brief        Pairs rooms of two maps by room tag; map-local ids are never
 *               compared.
 *
 * @param[in]    survivingRooms_in
 *               Evidence of the surviving map's rooms.
 *
 * @param[in]    absorbedRooms_in
 *               Evidence of the absorbed map's rooms.
 *
 * @param[out]   consecutiveAnchors_out
 *               One pair per absorbed room whose non-empty tag also names a
 *               surviving room. The pointers borrow from the input vectors and
 *               are valid only while they live.
 *
 * @return       Always SEMANTIC_VERIFY_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticVerifyStatus collectConsecutiveAnchors(
    const std::vector<SemanticMergeRoomEvidence> &survivingRooms_in,
    const std::vector<SemanticMergeRoomEvidence> &absorbedRooms_in,
    std::vector<ConsecutiveAnchorPair>           &consecutiveAnchors_out);

/*!
 * @brief        Maps a point from the absorbed map into the surviving map.
 *
 * @param[in]    transform_in
 *               Similarity transform from the absorbed map's world frame to the
 *               surviving map's world frame.
 *
 * @param[in]    point_in
 *               Point in the absorbed map's world frame, metres.
 *
 * @param[out]   mapped_out
 *               Point in the surviving map's world frame, metres; written only
 *               on success.
 *
 * @return       SEMANTIC_VERIFY_STATUS_SUCCESS on success;
 *               SEMANTIC_VERIFY_STATUS_INVALID_ARGUMENT when the scale is not
 *               finite and positive or the transform or point is not finite;
 *               SEMANTIC_VERIFY_STATUS_NUMERICAL_FAILURE when the result is not
 *               finite.
 */
[[nodiscard]] SemanticVerifyStatus
    transformAbsorbedPoint(const g2o::Sim3       &transform_in,
                           const Eigen::Vector3d &point_in,
                           Eigen::Vector3d       &mapped_out);

/*!
 * @brief        Compares the best observed floor of each map under the proposed
 *               transform.
 *
 * @param[in]    p_survivingMap_in
 *               Surviving map; must not be null. Borrowed.
 *
 * @param[in]    p_absorbedMap_in
 *               Absorbed map; must not be null. Borrowed.
 *
 * @param[in]    transform_in
 *               Maps absorbed-map world points into the surviving map's world
 *               frame.
 *
 * @param[in]    maximumOffset_m_in
 *               Largest allowed floor plane offset difference, metres.
 *
 * @param[out]   decision_out
 *               "DEFERRED" when either map has no floor plane, otherwise
 *               "ACCEPTED" or "REJECTED".
 *
 * @param[out]   floorsMatch_out
 *               True only when the decision is "ACCEPTED".
 *
 * @return       Always SEMANTIC_VERIFY_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticVerifyStatus
    checkConsecutiveFloors(core::Map       *p_survivingMap_in,
                           core::Map       *p_absorbedMap_in,
                           const g2o::Sim3 &transform_in,
                           double           maximumOffset_m_in,
                           std::string     &decision_out,
                           bool            &floorsMatch_out);

/*!
 * @brief        Checks that each absorbed anchor room lands on its surviving
 *               partner after the proposed transform.
 *
 * @param[in]    pairs_in
 *               Tag-matched room pairs.
 *
 * @param[in]    transform_in
 *               Maps absorbed-map world points into the surviving map's world
 *               frame.
 *
 * @param[in]    maximumDistance_m_in
 *               Largest allowed centroid distance, metres.
 *
 * @param[out]   alignmentCheck_out
 *               ALIGNED when every mapped absorbed centroid lies within the
 *               distance of its surviving partner; MISSING when there are no
 *               pairs or a centroid is not usable; CONTRADICTION when one is
 *               too far.
 *
 * @return       Always SEMANTIC_VERIFY_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticVerifyStatus
    checkAnchorRoomCentroids(const std::vector<ConsecutiveAnchorPair> &pairs_in,
                             const g2o::Sim3 &transform_in,
                             double           maximumDistance_m_in,
                             AlignmentCheck  &alignmentCheck_out);

/*!
 * @brief        Tells whether a passage snapshot has geometry good enough to
 *               compare.
 *
 * @param[in]    context_in
 *               Passage snapshot to test.
 *
 * @param[out]   isUsable_out
 *               True when the centroid is finite and width and height are
 *               finite and positive.
 *
 * @return       Always SEMANTIC_VERIFY_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticVerifyStatus
    passageGeometryIsUsable(const PassageContext &context_in,
                            bool                 &isUsable_out);

/*!
 * @brief        Pairs passages across two maps by room tag, passage lineage or
 *               endpoints, and checks that paired passages agree under the
 *               transform.
 *
 * @param[in]    survivingRooms_in
 *               Evidence of the surviving map's rooms.
 *
 * @param[in]    absorbedRooms_in
 *               Evidence of the absorbed map's rooms.
 *
 * @param[in]    transform_in
 *               Maps absorbed-map world points into the surviving map's world
 *               frame.
 *
 * @param[in]    maximumCentroidDistance_m_in
 *               Largest allowed passage centroid distance, metres.
 *
 * @param[out]   matchedCount_out
 *               Number of absorbed passages paired with a surviving one.
 *
 * @param[out]   contradictionReason_out
 *               Reason for a CONTRADICTION, or PASSAGE_EVIDENCE_MISSING when
 *               the absorbed side has no usable passage; untouched otherwise.
 *
 * @param[out]   alignmentCheck_out
 *               ALIGNED when every passage pairs and agrees; MISSING when
 *               evidence is absent or incomplete; CONTRADICTION when a pair
 *               disagrees or none pairs.
 *
 * @return       Always SEMANTIC_VERIFY_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticVerifyStatus checkConsecutivePassageTopology(
    const std::vector<SemanticMergeRoomEvidence> &survivingRooms_in,
    const std::vector<SemanticMergeRoomEvidence> &absorbedRooms_in,
    const g2o::Sim3                              &transform_in,
    double                                        maximumCentroidDistance_m_in,
    std::size_t                                  &matchedCount_out,
    SemanticMergeReason                          &contradictionReason_out,
    AlignmentCheck                               &alignmentCheck_out);

/*!
 * @brief        Finds the stretch of an axis that a wall's support samples
 *               cover.
 *
 * @param[in]    wall_in
 *               Wall whose support samples are projected.
 *
 * @param[in]    axis_in
 *               Axis to project onto; metres when unit length.
 *
 * @param[in]    origin_in
 *               Point in the same frame as the samples that sits at coordinate
 *               zero.
 *
 * @param[out]   minimum_out
 *               Smallest coordinate of the finite samples; written only when
 *               hasFiniteSample_out is true.
 *
 * @param[out]   maximum_out
 *               Largest coordinate of the finite samples; written only when
 *               hasFiniteSample_out is true.
 *
 * @param[out]   hasFiniteSample_out
 *               True when the wall has at least one finite sample.
 *
 * @return       Always SEMANTIC_VERIFY_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticVerifyStatus
    wallSamplesSpanInterval(const VerifyWallObservation &wall_in,
                            const Eigen::Vector3d       &axis_in,
                            const Eigen::Vector3d       &origin_in,
                            double                      &minimum_out,
                            double                      &maximum_out,
                            bool                        &hasFiniteSample_out);

[[nodiscard]] SemanticVerifyStatus checkConsecutiveWallEdgeOverlap(
    const std::vector<ConsecutiveAnchorPair> &pairs_in,
    const g2o::Sim3                          &transform_in,
    double                                    maximumNormalAngle_deg_in,
    double                                    minimumOverlap_m_in,
    AlignmentCheck                           &alignmentCheck_out);

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // VS_GRAPHS_CORE_SEMANTIC_SEMANTICVERIFY_PRIVATE_FUNCTIONS_H

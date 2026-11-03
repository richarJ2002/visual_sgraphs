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

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_PRIVATE_FUNCTIONS_H

#include "scene_understanding/semantic/semantic_verify/objects/SemanticVerifyStatusEnum.h"
#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"

#include "mapping/geometric/Plane/Plane.h"
#include "place_recognition_and_loop_closure/loop_closing/loop_closing.h"
#include "map_state/map/map.h"

#include <vs_graphs_g2o/core/block_solver.h>
#include <vs_graphs_g2o/core/optimization_algorithm_levenberg.h>
#include <vs_graphs_g2o/core/robust_kernel_impl.h>
#include <vs_graphs_g2o/core/sparse_optimizer.h>
#include <vs_graphs_g2o/solvers/linear_solver_eigen.h>
#include <vs_graphs_g2o/types/sim3.h>
#include "configuration/types/types.h"
#include "scene_understanding/semantic/room/objects/RoomClass.h"

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

/*!
 * @brief           One admitted (indexA, indexB) candidate correspondence --
 *                  not yet a hypothesis, just a pairing considered for 3-subset
 *                  enumeration.
 */
struct CandidatePair
{
    /*!
     * @brief           Index of the wall in room A's observation list.
     */
    std::size_t indexA{0U};
    /*!
     * @brief           Index of the wall in room B's observation list.
     */
    std::size_t indexB{0U};
};

/*!
 * @brief           Rotation estimated from paired wall normals.
 */
struct RotationFit
{
    /*!
     * @brief           True when the fit succeeded and rotation may be used.
     */
    bool            valid{false};
    /*!
     * @brief           Rotation taking room A's wall normals onto room B's.
     */
    Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
};

/*!
 * @brief           Translation estimated from paired wall plane offsets.
 */
struct TranslationFit
{
    /*!
     * @brief           True when the fit succeeded and the other fields may be
     *                  used.
     */
    bool            valid{false};
    /*!
     * @brief           Translation of the room A to room B transform, metres.
     */
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
    /*!
     * @brief           Rank of the normal matrix; 3 means the translation is
     *                  fully determined.
     */
    std::size_t     rank{0U};
    /*!
     * @brief           Condition number of the normal matrix; infinite until
     *                  computed.
     */
    double          conditionNumber{std::numeric_limits<double>::infinity()};
};

/*!
 * @brief           Result of one consistency check between two maps' evidence:
 *                  evidence agrees, evidence is missing, or evidence
 *                  contradicts.
 */
enum class AlignmentCheck
{
    ALIGNED,
    MISSING,
    CONTRADICTION
};

/*!
 * @brief           Tells whether a vector contains only finite numbers.
 *
 * @param[in]       value_in
 *                  Vector to test.
 *
 * @param[out]      isFiniteVector_out
 *                  True when all three components are finite.
 *
 * @return          Always SEMANTIC_VERIFY_STATUS_SUCCESS.
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
 * @brief           Returns the angle between two vectors.
 *
 * @param[in]       first_in
 *                  First vector; need not be unit length.
 *
 * @param[in]       second_in
 *                  Second vector; need not be unit length.
 *
 * @param[out]      angle_rad_out
 *                  Angle between the vectors, radians in [0, pi].
 *
 * @return          Always SEMANTIC_VERIFY_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticVerifyStatus
    angleBetween_rad(const Eigen::Vector3d &first_in,
                     const Eigen::Vector3d &second_in,
                     double                &angle_rad_out);

/*!
 * @brief           Finds the observation of a wall by its identifier.
 *
 * @param[in]       walls_in
 *                  Wall observations to search.
 *
 * @param[in]       wallId_in
 *                  Wall identifier to look for.
 *
 * @param[out]      p_byWallId_out
 *                  First observation with that identifier, or nullptr when
 *                  there is none. Points into walls_in.
 *
 * @return          Always SEMANTIC_VERIFY_STATUS_SUCCESS.
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
 * @brief           Returns the name a room is known by across maps.
 *
 * @param[in]       context_in
 *                  Room snapshot.
 *
 * @param[out]      identity_out
 *                  The room tag, or "room_<id>" when the tag is empty.
 *
 * @return          Always SEMANTIC_VERIFY_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticVerifyStatus
    stableRoomIdentity(const RoomContextSnapshot &context_in,
                       std::string               &identity_out);

/*!
 * @brief           Counts the absorbed walls that coincide with a surviving
 *                  wall under a given transform, without searching for a better
 *                  one.
 *
 * @param[in]       survivingWalls_in
 *                  Wall observations of the surviving room.
 *
 * @param[in]       absorbedWalls_in
 *                  Wall observations of the absorbed room.
 *
 * @param[in]       transform_in
 *                  Maps absorbed-map world points into the surviving map's
 *                  world frame.
 *
 * @param[in]       configuration_in
 *                  Angle, offset and inlier ratio thresholds.
 *
 * @param[out]      matchedCount_out
 *                  Number of absorbed walls matched one-to-one to a surviving
 *                  wall.
 *
 * @param[out]      alignmentCheck_out
 *                  MISSING when either room has fewer than three walls; ALIGNED
 *                  when at least three walls match and the matched fraction of
 *                  the larger wall set reaches minInlierRatio; CONTRADICTION
 *                  otherwise, including an unusable transform.
 *
 * @return          Always SEMANTIC_VERIFY_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticVerifyStatus checkFixedTransformWalls(
    const std::vector<VerifyWallObservation> &survivingWalls_in,
    const std::vector<VerifyWallObservation> &absorbedWalls_in,
    const g2o::Sim3                          &transform_in,
    const SemanticVerifyConfig               &configuration_in,
    std::size_t                              &matchedCount_out,
    AlignmentCheck                           &alignmentCheck_out);

/*!
 * @brief           Checks that the two rooms' passages, paired by passage id,
 *                  agree on whether they are passable, which rooms they join
 *                  and which way they face.
 *
 * @param[in]       survivingContext_in
 *                  Snapshot of the surviving room.
 *
 * @param[in]       absorbedContext_in
 *                  Snapshot of the absorbed room.
 *
 * @param[in]       transform_in
 *                  Maps absorbed-map world points into the surviving map's
 *                  world frame; only its rotation is used.
 *
 * @param[in]       configuration_in
 *                  Supplies minAbsCosNormalAngle, the smallest direction
 *                  agreement.
 *
 * @param[out]      matchedCount_out
 *                  Number of absorbed passages whose id exists in the surviving
 *                  room.
 *
 * @param[out]      contradictionReason_out
 *                  Reason for a CONTRADICTION; untouched otherwise.
 *
 * @param[out]      alignmentCheck_out
 *                  MISSING when either room has no passages or some evidence is
 *                  incomplete; CONTRADICTION when passable flag, endpoint rooms
 *                  or direction disagree, or no id matches; ALIGNED otherwise.
 *
 * @return          Always SEMANTIC_VERIFY_STATUS_SUCCESS.
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
 * @brief           Copies one room's identity, walls and passages into plain
 *                  data so no live map pointer leaves the merge lock.
 *
 * @param[in]       p_room_in
 *                  Room to copy; must not be null. Borrowed. The caller holds
 *                  the merge lock.
 *
 * @param[in]       configuration_in
 *                  Limits for the wall observations.
 *
 * @param[out]      evidence_out
 *                  Copy of the room's id, tag, centroid, floor id (-1 when it
 *                  has no floor), walls and non-bad passages.
 *
 * @return          Always SEMANTIC_VERIFY_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticVerifyStatus
    copyMergeRoomEvidence(const Room                 *p_room_in,
                          const SemanticVerifyConfig &configuration_in,
                          SemanticMergeRoomEvidence  &evidence_out);

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_PRIVATE_FUNCTIONS_H */

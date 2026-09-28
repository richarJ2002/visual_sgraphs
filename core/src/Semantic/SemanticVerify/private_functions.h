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
    std::size_t indexA{0U};
    std::size_t indexB{0U};
};

struct RotationFit
{
    bool            valid{false};
    Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
};

struct TranslationFit
{
    bool            valid{false};
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
    std::size_t     rank{0U};
    double          conditionNumber{std::numeric_limits<double>::infinity()};
};

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
    const SemanticMergeRoomEvidence *p_surviving{nullptr};
    const SemanticMergeRoomEvidence *p_absorbed{nullptr};
};

/*! @brief Minimum known-side direction agreement for paired passages. Mirrors
 * the verification default (SemanticVerifyConfig::minAbsCosNormalAngle). */
constexpr double kConsecutiveMinDirectionAgreement = 0.85;

/*! @brief Maximum plane offset for wall pairing. Mirrors the verification
 * default (SemanticVerifyConfig::maxOffset_m); the mapMerge section tunes
 * the angle and the edge overlap, not the offset. */
constexpr double kConsecutiveMaxPlaneOffset_m = 0.35;

bool isFiniteVector(const Eigen::Vector3d &value_in);

RotationFit
    fitRotationFromNormals(const std::vector<Eigen::Vector3d> &normalsA_in,
                           const std::vector<Eigen::Vector3d> &normalsB_in);

TranslationFit fitTranslation(const Eigen::Matrix3d              &rotation_in,
                              const std::vector<Eigen::Vector3d> &normalsA_in,
                              const std::vector<double>          &offsetsA_in,
                              const std::vector<Eigen::Vector3d> &normalsB_in,
                              const std::vector<double>          &offsetsB_in);

double angleBetween_rad(const Eigen::Vector3d &first_in,
                        const Eigen::Vector3d &second_in);

const VerifyWallObservation *
    findByWallId(const std::vector<VerifyWallObservation> &walls_in,
                 const int                                 wallId_in);

double symmetricSupportDistance(const VerifyWallObservation &wallA_in,
                                const VerifyWallObservation &wallB_in,
                                const Eigen::Matrix3d       &rotation_in,
                                const Eigen::Vector3d       &translation_in);

std::string stableRoomIdentity(const RoomContextSnapshot &context_in);

AlignmentCheck checkFixedTransformWalls(
    const std::vector<VerifyWallObservation> &survivingWalls_in,
    const std::vector<VerifyWallObservation> &absorbedWalls_in,
    const g2o::Sim3                          &transform_in,
    const SemanticVerifyConfig               &configuration_in,
    std::size_t                              &matchedCount_out);

AlignmentCheck
    checkPassageTopology(const RoomContextSnapshot  &survivingContext_in,
                         const RoomContextSnapshot  &absorbedContext_in,
                         const g2o::Sim3            &transform_in,
                         const SemanticVerifyConfig &configuration_in,
                         std::size_t                &matchedCount_out,
                         SemanticMergeReason        &contradictionReason_out);

SemanticMergeRoomEvidence
    copyMergeRoomEvidence(const Room                 *p_room_in,
                          const SemanticVerifyConfig &configuration_in);

std::vector<ConsecutiveAnchorPair> collectConsecutiveAnchors(
    const std::vector<SemanticMergeRoomEvidence> &survivingRooms_in,
    const std::vector<SemanticMergeRoomEvidence> &absorbedRooms_in);

bool transformAbsorbedPoint(const g2o::Sim3       &transform_in,
                            const Eigen::Vector3d &point_in,
                            Eigen::Vector3d       &mapped_out);

bool checkConsecutiveFloors(core::Map       *p_survivingMap_in,
                            core::Map       *p_absorbedMap_in,
                            const g2o::Sim3 &transform_in,
                            double           maximumOffset_m_in,
                            std::string     &decision_out);

AlignmentCheck
    checkAnchorRoomCentroids(const std::vector<ConsecutiveAnchorPair> &pairs_in,
                             const g2o::Sim3 &transform_in,
                             double           maximumDistance_m_in);

bool passageGeometryIsUsable(const PassageContext &context_in);

AlignmentCheck checkConsecutivePassageTopology(
    const std::vector<SemanticMergeRoomEvidence> &survivingRooms_in,
    const std::vector<SemanticMergeRoomEvidence> &absorbedRooms_in,
    const g2o::Sim3                              &transform_in,
    double                                        maximumCentroidDistance_m_in,
    std::size_t                                  &matchedCount_out,
    SemanticMergeReason                          &contradictionReason_out);

bool wallSamplesSpanInterval(const VerifyWallObservation &wall_in,
                             const Eigen::Vector3d       &axis_in,
                             const Eigen::Vector3d       &origin_in,
                             double                      &minimum_out,
                             double                      &maximum_out);

AlignmentCheck checkConsecutiveWallEdgeOverlap(
    const std::vector<ConsecutiveAnchorPair> &pairs_in,
    const g2o::Sim3                          &transform_in,
    double                                    maximumNormalAngle_deg_in,
    double                                    minimumOverlap_m_in);

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // VS_GRAPHS_CORE_SEMANTIC_SEMANTICVERIFY_PRIVATE_FUNCTIONS_H

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

#include "scene_understanding/semantic/semantic_verify/objects/MergePairStruct.h"
#include "scene_understanding/semantic/semantic_verify/objects/MergePairingStruct.h"
#include "scene_understanding/semantic/semantic_verify/objects/PlaneGapStruct.h"
#include "scene_understanding/semantic/semantic_verify/objects/RoomPairStruct.h"
#include "scene_understanding/semantic/semantic_verify/objects/SemanticVerifyStatusEnum.h"
#include "scene_understanding/semantic/semantic_verify/objects/WallRelationEnum.h"
#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"

#include "map_state/map/map.h"
#include "mapping/geometric/Plane/Plane.h"
#include "place_recognition_and_loop_closure/loop_closing/loop_closing.h"

#include "configuration/types/types.h"
#include "scene_understanding/semantic/room/objects/RoomClass.h"
#include <vs_graphs_g2o/core/block_solver.h>
#include <vs_graphs_g2o/core/optimization_algorithm_levenberg.h>
#include <vs_graphs_g2o/core/robust_kernel_impl.h>
#include <vs_graphs_g2o/core/sparse_optimizer.h>
#include <vs_graphs_g2o/solvers/linear_solver_eigen.h>
#include <vs_graphs_g2o/types/sim3.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <string>

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
 * @brief           Degrees in one radian; shared by the pairing helpers.
 */
inline constexpr double DEGREES_PER_RADIAN = 57.29577951308232;

/*!
 * @brief           Copies one room's identity, floor plane, walls and passages
 *                  into plain data so no live map pointer leaves the merge
 *                  lock.
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
 *                  has no floor), floor plane, walls and non-bad passages with
 *                  their planes.
 *
 * @return          Always SEMANTIC_VERIFY_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticVerifyStatus
    copyMergeRoomEvidence(const Room                 *p_room_in,
                          const SemanticVerifyConfig &configuration_in,
                          SemanticMergeRoomEvidence  &evidence_out);

/*!
 * @brief           Copies one passage's measured geometry and sides into plain
 *                  data.
 *
 * @param[in]       p_passage_in
 *                  Passage to copy; must not be null. Borrowed. The caller
 *                  holds the merge lock.
 *
 * @return          The passage's id, passable flag, centre, aperture size,
 *                  plane (unit normal; absent when the plane is degenerate),
 *                  recovery-proxy flag and known-side and far-side
 *                  information.
 */
[[nodiscard]] PassageContext copyPassageContext(const Passage *p_passage_in);

/*!
 * @brief           Copies the evidence of every confirmed room in a list.
 *
 * @param[in]       rooms_in
 *                  Rooms of one map; null, bad and unconfirmed (prospective)
 *                  rooms are skipped.
 *
 * @param[in]       configuration_in
 *                  Limits for the wall observations.
 *
 * @param[out]      admittedRooms_out
 *                  The rooms that were copied, in the order of evidence_out.
 *                  Borrowed.
 *
 * @param[out]      evidence_out
 *                  One evidence copy per admitted room.
 */
void collectRoomEvidence(const std::vector<Room *>  &rooms_in,
                         const SemanticVerifyConfig &configuration_in,
                         std::vector<Room *>        &admittedRooms_out,
                         std::vector<SemanticMergeRoomEvidence> &evidence_out);

/*!
 * @brief           Copies every live passage in a list into plain data.
 *
 * @param[in]       passages_in
 *                  Passages of one map; null and bad passages are skipped.
 *
 * @param[out]      admittedPassages_out
 *                  The passages that were copied, in the order of
 *                  contexts_out. Borrowed.
 *
 * @param[out]      contexts_out
 *                  One copy per admitted passage.
 */
void collectPassageContexts(const std::vector<Passage *> &passages_in,
                            std::vector<Passage *>       &admittedPassages_out,
                            std::vector<PassageContext>  &contexts_out);

/*!
 * @brief           Moves a plane from the absorbed map's world frame into the
 *                  surviving map's world frame.
 *
 * @param[in]       plane_absorbedWorld_in
 *                  Plane (n^T x + d = 0) with a unit normal, in the absorbed
 *                  world frame.
 *
 * @param[in]       mergeTransform_absorbedToSurviving_in
 *                  Similarity mapping a point as p_surviving = scale * R *
 *                  p_absorbed + t.
 *
 * @return          The same plane in the surviving world frame: normal R * n,
 *                  offset scale * d - (R * n)^T t. The normal stays a unit
 *                  vector.
 */
[[nodiscard]] Eigen::Vector4d transformPlaneBetweenMaps(
    const Eigen::Vector4d &plane_absorbedWorld_in,
    const g2o::Sim3       &mergeTransform_absorbedToSurviving_in);

/*!
 * @brief           Moves one wall observation (plane, centroid and support
 *                  sample) into the surviving map's world frame.
 *
 * @param[in]       wall_absorbedWorld_in
 *                  Wall observation in the absorbed world frame.
 *
 * @param[in]       mergeTransform_absorbedToSurviving_in
 *                  Similarity from the absorbed to the surviving world frame.
 *
 * @return          The wall observation in the surviving world frame; the wall
 *                  id is kept for logging only.
 */
[[nodiscard]] VerifyWallObservation transformWallBetweenMaps(
    const VerifyWallObservation &wall_absorbedWorld_in,
    const g2o::Sim3             &mergeTransform_absorbedToSurviving_in);

/*!
 * @brief           Moves one passage (centre, plane, direction and size) into
 *                  the surviving map's world frame.
 *
 * @param[in]       passage_absorbedWorld_in
 *                  Passage description in the absorbed world frame.
 *
 * @param[in]       mergeTransform_absorbedToSurviving_in
 *                  Similarity from the absorbed to the surviving world frame.
 *
 * @return          The passage in the surviving world frame; width and height
 *                  are multiplied by the transform scale.
 */
[[nodiscard]] PassageContext transformPassageBetweenMaps(
    const PassageContext &passage_absorbedWorld_in,
    const g2o::Sim3      &mergeTransform_absorbedToSurviving_in);

/*!
 * @brief           Moves one room's evidence (centroid, floor plane, walls and
 *                  passages) into the surviving map's world frame.
 *
 * @param[in]       room_absorbedWorld_in
 *                  Room evidence in the absorbed world frame.
 *
 * @param[in]       mergeTransform_absorbedToSurviving_in
 *                  Similarity from the absorbed to the surviving world frame.
 *
 * @return          The room evidence in the surviving world frame; ids are
 *                  kept.
 */
[[nodiscard]] SemanticMergeRoomEvidence transformRoomEvidenceBetweenMaps(
    const SemanticMergeRoomEvidence &room_absorbedWorld_in,
    const g2o::Sim3                 &mergeTransform_absorbedToSurviving_in);

/*!
 * @brief           Measures how far two planes in one frame lie apart.
 *
 * @param[in]       firstPlane_world_in
 *                  First plane (n^T x + d = 0); the normal need not be unit.
 *
 * @param[in]       secondPlane_world_in
 *                  Second plane in the same frame.
 *
 * @param[in]       referencePoint_world_in
 *                  Point on the first plane, metres, at which the offset is
 *                  measured; measuring locally keeps a small rotation error
 *                  from growing with the distance to the frame origin.
 *
 * @param[in]       isOrientationSignificant_in
 *                  True when opposite normals describe different faces (walls,
 *                  floors); false when the second plane may be flipped to face
 *                  the first (passage planes).
 *
 * @return          The normal angle in degrees and the offset in metres; both
 *                  infinite when a plane or the point is not finite or a normal
 *                  is degenerate.
 */
[[nodiscard]] PlaneGap
    measurePlaneGap(const Eigen::Vector4d &firstPlane_world_in,
                    const Eigen::Vector4d &secondPlane_world_in,
                    const Eigen::Vector3d &referencePoint_world_in,
                    bool                   isOrientationSignificant_in);

/*!
 * @brief           Measures the in-plane gap between two wall patches that lie
 *                  in one frame.
 *
 * @param[in]       firstWall_world_in
 *                  First wall; its support sample gives its extent.
 *
 * @param[in]       secondWall_world_in
 *                  Second wall in the same frame; its normal defines the
 *                  in-plane axes.
 *
 * @return          Largest gap, metres, between the two patches' extents along
 *                  the two in-plane axes; 0 when they overlap on both. Infinite
 *                  when either wall has no support sample.
 */
[[nodiscard]] double
    measureWallExtentGap(const VerifyWallObservation &firstWall_world_in,
                         const VerifyWallObservation &secondWall_world_in);

/*!
 * @brief           Relates one absorbed wall to one surviving wall.
 *
 * @param[in]       absorbedWall_survivingWorld_in
 *                  Absorbed wall already moved into the surviving frame.
 *
 * @param[in]       survivingWall_in
 *                  Surviving wall.
 *
 * @param[in]       configuration_in
 *                  Supplies the coplanar angle, wall offset and edge overlap.
 *
 * @return          PAIRED when the normals agree (orientation included), the
 *                  offset at the absorbed centroid is small and the patches
 *                  overlap or touch; OFFSET_MISMATCH when only the offset is
 *                  too large; UNRELATED otherwise.
 */
[[nodiscard]] WallRelation classifyWallRelation(
    const VerifyWallObservation &absorbedWall_survivingWorld_in,
    const VerifyWallObservation &survivingWall_in,
    const SemanticVerifyConfig  &configuration_in);

/*!
 * @brief           Pairs walls of the two maps by geometry, never by id.
 *
 * @param[in]       absorbedWalls_survivingWorld_in
 *                  Absorbed walls already moved into the surviving frame.
 *
 * @param[in]       survivingWalls_in
 *                  Surviving walls.
 *
 * @param[in]       configuration_in
 *                  Pairing thresholds.
 *
 * @return          Every PAIRED combination as a pair. A contradiction is an
 *                  OFFSET_MISMATCH combination whose two walls both have no
 *                  PAIRED partner. A wall without any relation appears in
 *                  neither list: it gives no answer.
 */
[[nodiscard]] MergePairing pairWalls(
    const std::vector<VerifyWallObservation> &absorbedWalls_survivingWorld_in,
    const std::vector<VerifyWallObservation> &survivingWalls_in,
    const SemanticVerifyConfig               &configuration_in);

/*!
 * @brief           Pairs rooms whose floors are coplanar.
 *
 * @param[in]       absorbedRooms_survivingWorld_in
 *                  Absorbed rooms already moved into the surviving frame.
 *
 * @param[in]       survivingRooms_in
 *                  Surviving rooms.
 *
 * @param[in]       configuration_in
 *                  Supplies the coplanar angle and the floor tolerance.
 *
 * @return          One pair per absorbed and surviving room whose floor planes
 *                  agree in orientation and, at the absorbed room's centroid,
 *                  in height. Rooms without a floor plane are not paired.
 */
[[nodiscard]] std::vector<MergePair>
    pairFloors(const std::vector<SemanticMergeRoomEvidence>
                   &absorbedRooms_survivingWorld_in,
               const std::vector<SemanticMergeRoomEvidence> &survivingRooms_in,
               const SemanticVerifyConfig                   &configuration_in);

/*!
 * @brief           Pairs rooms of the two maps through their walls, never by
 *                  tag or id.
 *
 * @param[in]       absorbedRooms_survivingWorld_in
 *                  Absorbed rooms already moved into the surviving frame.
 *
 * @param[in]       survivingRooms_in
 *                  Surviving rooms.
 *
 * @param[in]       configuration_in
 *                  Supplies the wall pairing thresholds, the room pair wall
 *                  fraction and the minimum wall pairs.
 *
 * @return          One entry per room combination in which at least the
 *                  minimum number of walls pair and the paired fraction of the
 *                  smaller room's walls reaches the room pair wall fraction.
 */
[[nodiscard]] std::vector<RoomPair>
    pairRooms(const std::vector<SemanticMergeRoomEvidence>
                  &absorbedRooms_survivingWorld_in,
              const std::vector<SemanticMergeRoomEvidence> &survivingRooms_in,
              const SemanticVerifyConfig                   &configuration_in);

/*!
 * @brief           Pairs passages of the two maps by geometry, never by id.
 *
 * @param[in]       absorbedPassages_survivingWorld_in
 *                  Absorbed passages already moved into the surviving frame.
 *
 * @param[in]       survivingPassages_in
 *                  Surviving passages.
 *
 * @param[in]       configuration_in
 *                  Supplies the coplanar angle and the wall plane offset.
 *
 * @return          Two passages pair when both have a plane and a valid
 *                  aperture and are not recovery proxies, lie on the same plane
 *                  (normal angle and offset, orientation ignored) and their
 *                  centres are no further apart, measured in that plane, than
 *                  half the narrower passage's width. Passages never
 *                  contradict: a difference in the passable flag is only
 *                  a partial view, so it gives no answer.
 */
[[nodiscard]] MergePairing pairPassages(
    const std::vector<PassageContext> &absorbedPassages_survivingWorld_in,
    const std::vector<PassageContext> &survivingPassages_in,
    const SemanticVerifyConfig        &configuration_in);

/*!
 * @brief           Finds the anchor room pairs, the only elements matched by
 *                  identity.
 *
 * @param[in]       survivingRooms_in
 *                  Surviving rooms, in evidence order; borrowed.
 *
 * @param[in]       absorbedRooms_in
 *                  Absorbed rooms, in evidence order; borrowed.
 *
 * @param[in]       p_survivingAnchorRoom_in
 *                  Surviving map's room of the map chain link
 *                  (Map::getLinkedRooms); may be null.
 *
 * @param[in]       p_absorbedAnchorRoom_in
 *                  Absorbed map's room of that link; may be null.
 *
 * @return          The one pair of list indices of the two linked rooms, or
 *                  no pair when either room is null or not in its list. No
 *                  other room is matched this way.
 */
[[nodiscard]] std::vector<MergePair>
    findAnchorRoomPairs(const std::vector<Room *> &survivingRooms_in,
                        const std::vector<Room *> &absorbedRooms_in,
                        const Room                *p_survivingAnchorRoom_in,
                        const Room                *p_absorbedAnchorRoom_in);

/*!
 * @brief           Tells whether any element appears in more than one pair on
 *                  the same side.
 *
 * @param[in]       pairs_in
 *                  Pairs to inspect.
 *
 * @return          True when an absorbed or a surviving index occurs twice:
 *                  one element then pairs with two counterparts.
 */
[[nodiscard]] bool hasRepeatedElement(const std::vector<MergePair> &pairs_in);

/*!
 * @brief           Lists each passage of a map once.
 *
 * @param[in]       rooms_in
 *                  Rooms of one map; a passage between two rooms appears in
 *                  both.
 *
 * @return          The passages with distinct ids within this map. Ids are
 *                  compared inside one map only, never across maps.
 */
[[nodiscard]] std::vector<PassageContext> collectUniquePassages(
    const std::vector<SemanticMergeRoomEvidence> &rooms_in);

/*!
 * @brief           Formats pairs for the pairs log line.
 *
 * @param[in]       pairs_in
 *                  Pairs to format.
 *
 * @param[in]       absorbedIds_in
 *                  Printable id of each absorbed element.
 *
 * @param[in]       survivingIds_in
 *                  Printable id of each surviving element.
 *
 * @return          "a<id>~s<id>" entries separated by commas, or "-" when
 *                  there are no pairs. The ids are for reading logs only.
 */
[[nodiscard]] std::string
    formatMergePairs(const std::vector<MergePair> &pairs_in,
                     const std::vector<int>       &absorbedIds_in,
                     const std::vector<int>       &survivingIds_in);

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_PRIVATE_FUNCTIONS_H */

/*!
 * @file            RoomContextSnapshot.h
 *
 * @brief           Declares RoomContextSnapshot, a saved description of a room
 *                  (wall bounds and passages) used to recognise the room again.
 */

#ifndef ROOM_CONTEXT_SNAPSHOT_H
#define ROOM_CONTEXT_SNAPSHOT_H

#include <Eigen/Core>

#include <cstddef>
#include <string>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           Extent of one wall plane in its own in-plane (U, V)
 *                  coordinates, kept so rooms can be compared by wall size.
 */
struct WallBounds
{
    /*!
     * @brief           True when the four bounds below were finite and
     *                  span a non-empty rectangle at snapshot time.
     */
    bool   isValid{false};
    /*!
     * @brief           Smallest in-plane U coordinate of the wall points,
     *                  metres.
     */
    double minU_m{0.0};
    /*!
     * @brief           Largest in-plane U coordinate of the wall points,
     *                  metres.
     */
    double maxU_m{0.0};
    /*!
     * @brief           Smallest in-plane V coordinate of the wall points,
     *                  metres.
     */
    double minV_m{0.0};
    /*!
     * @brief           Largest in-plane V coordinate of the wall points,
     *                  metres.
     */
    double maxV_m{0.0};
};

/*!
 * @brief           Saved description of one passage (door or opening) of a
 *                  room: its aperture, which rooms it joins and how often it
 *                  was walked through.
 */
struct PassageContext
{
    /*!
     * @brief           Passage identifier; stays the same for the same
     *                  doorway across maps.
     */
    int             id{0};
    /*!
     * @brief           True when the passage was marked passable (the UAV can
     *                  fly through it).
     */
    bool            isPassable{false};
    /*!
     * @brief           True when the passage had a far-side room recorded;
     *                  secondaryRoomId is meaningful only then.
     */
    bool            hasFarSideRoom{false};
    /*!
     * @brief           Id of the room on the far side of the passage, valid
     *                  when hasFarSideRoom is true.
     */
    int             secondaryRoomId{0};
    /*!
     * @brief           True when width_m and height_m are both finite and
     *                  positive.
     */
    bool            isApertureValid{false};
    /*!
     * @brief           Passage opening width, metres.
     */
    double          width_m{0.0};
    /*!
     * @brief           Passage opening height, metres.
     */
    double          height_m{0.0};
    /*!
     * @brief           Centre of the passage in the world frame, metres.
     */
    Eigen::Vector3d passageCentroid_world{Eigen::Vector3d::Zero()};
    /*!
     * @brief           True when knownSideDirection_world holds a finite unit
     *                  direction.
     */
    bool            hasKnownSideDirection{false};
    /*!
     * @brief           Unit direction in the world frame from the passage
     *                  toward the side the UAV observed it from; valid when
     *                  hasKnownSideDirection is true.
     */
    Eigen::Vector3d knownSideDirection_world{Eigen::Vector3d::Zero()};
    /*!
     * @brief           True when the observing-side room was recorded;
     *                  knownSideRoomId is meaningful only then.
     */
    bool            hasKnownSideRoom{false};
    /*!
     * @brief           Id of the room on the side the passage was observed
     *                  from; -1 when not recorded.
     */
    int             knownSideRoomId{-1};
    /*!
     * @brief           Number of recorded crossings from the known side to the
     *                  far side.
     */
    std::size_t     traversalKnownToFarCount{0U};
    /*!
     * @brief           Number of recorded crossings from the far side back to
     *                  the known side.
     */
    std::size_t     traversalFarToKnownCount{0U};
    /*!
     * @brief           Number of recorded crossings whose direction could not
     *                  be determined.
     */
    std::size_t     traversalUnknownCount{0U};
    /*!
     * @brief           Number of walls associated with this passage.
     */
    std::size_t     associatedWallCount{0U};
    /*!
     * @brief           True when the passage was crossed in both directions.
     */
    bool            hasBidirectionalTraversalEvidence{false};
    /*!
     * @brief           True when the passage is historical recovery topology
     *                  only, not a freshly observed one.
     */
    bool            isRecoveryProxy{false};
};

/*!
 * @brief           Saved description of one room (walls, passages, tag) taken
 *                  from a map, used to recognise the same room again later.
 *                  The wall vectors share one index per wall and the passage
 *                  vectors one index per passage.
 */
struct RoomContextSnapshot
{
    /*!
     * @brief           Id of the room this snapshot was taken from.
     */
    int                          roomId{0};
    /*!
     * @brief           The room's floor at snapshot time, -1 when the room had
     *                  no floor identity yet. A room is always floor-scoped
     *                  (Room::getFloor()), so this is captured alongside roomId
     *                  rather than re-derived later -- by the time this
     *                  snapshot is consumed (e.g. reacquisition after a
     *                  tracking loss), the original Room object may no longer
     *                  be the cheapest way to answer "what floor was the UAV
     *                  on". A future building-node level would extend this the
     *                  same way, once that concept exists.
     */
    int                          floorId{-1};
    /*!
     * @brief           Room centroid in the world frame, metres.
     */
    Eigen::Vector3d              centroid{Eigen::Vector3d::Zero()};
    /*!
     * @brief           Per wall, unit normal in the world frame pointing toward
     *                  the room; NaN when the wall was bad or had no normal.
     */
    std::vector<Eigen::Vector3d> wallNormals;
    /*!
     * @brief           Per wall, wall centroid in the world frame, metres; NaN
     *                  when the wall was bad.
     */
    std::vector<Eigen::Vector3d> wallCentroids;
    /*!
     * @brief           Per wall, distance of the wall plane from the world
     *                  origin, metres; NaN when the wall was bad.
     */
    std::vector<double>          wallDistances;
    /*!
     * @brief           Per wall, in-plane extent of the wall; default (invalid)
     *                  when the wall was bad.
     */
    std::vector<WallBounds>      wallBounds;
    /*!
     * @brief           Per passage, passage centre in the world frame, metres;
     *                  NaN when the passage pointer was null.
     */
    std::vector<Eigen::Vector3d> passageCentroids;
    /*!
     * @brief           Per passage, saved passage description, in the same
     *                  order as passageCentroids.
     */
    std::vector<PassageContext>  passageContexts;
    /*!
     * @brief           Time the snapshot was taken, seconds since the epoch of
     *                  the clock named in timestampProvenance.
     */
    double                       timestamp{0.0};
    /*!
     * @brief           Name of the clock that produced timestamp.
     */
    std::string                  timestampProvenance{"steady_clock"};
    /*!
     * @brief           Persistent room tag, "room_<roomId>", used to match
     *                  rooms across maps.
     */
    std::string                  roomTag;
    /*!
     * @brief           True when the room had been promoted to a confirmed
     *                  room at snapshot time, false while it was still a
     *                  candidate.
     */
    bool                         wasConfirmedRoom{false};
    /*!
     * @brief           Whether the UAV had entered the room at snapshot time.
     *                  Mission truth restored alongside identity; never
     *                  consulted by creation, promotion, retirement, or merge
     *                  paths.
     */
    bool                         wasPreviouslyVisited{false};
    /*!
     * @brief           Room::BoundaryStatus at snapshot time, stored as its
     *                  integer value (0 unobserved, 1 incomplete, 2 complete,
     *                  3 conflicting).
     */
    int                          boundaryStatus{0};
};
} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // ROOM_CONTEXT_SNAPSHOT_H

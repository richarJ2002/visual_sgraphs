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
struct WallBounds
{
    bool   isValid{false};
    double minU_m{0.0};
    double maxU_m{0.0};
    double minV_m{0.0};
    double maxV_m{0.0};
};

struct PassageContext
{
    int             id{0};
    bool            isPassable{false};
    bool            hasFarSideRoom{false};
    int             secondaryRoomId{0};
    bool            isApertureValid{false};
    double          width_m{0.0};
    double          height_m{0.0};
    Eigen::Vector3d centroid_World{Eigen::Vector3d::Zero()};
    bool            hasKnownSideDirection{false};
    Eigen::Vector3d knownSideDirection_World{Eigen::Vector3d::Zero()};
    bool            hasKnownSideRoom{false};
    int             knownSideRoomId{-1};
    std::size_t     traversalKnownToFarCount{0U};
    std::size_t     traversalFarToKnownCount{0U};
    std::size_t     traversalUnknownCount{0U};
    std::size_t     associatedWallCount{0U};
    bool            hasBidirectionalTraversalEvidence{false};
    bool            isRecoveryProxy{false};
};

struct RoomContextSnapshot
{
    int                          roomId{0};
    /*! The room's floor at snapshot time, -1 when the room had no floor
     *  identity yet. A room is always floor-scoped (Room::getFloor()), so
     *  this is captured alongside roomId rather than re-derived later --
     *  by the time this snapshot is consumed (e.g. reacquisition after a
     *  tracking loss), the original Room object may no longer be the
     *  cheapest way to answer "what floor was the UAV on". A future
     *  building-node level would extend this the same way, once that
     *  concept exists. */
    int                          floorId{-1};
    Eigen::Vector3d              centroid{Eigen::Vector3d::Zero()};
    std::vector<Eigen::Vector3d> wallNormals;
    std::vector<Eigen::Vector3d> wallCentroids;
    std::vector<double>          wallDistances;
    std::vector<WallBounds>      wallBounds;
    std::vector<Eigen::Vector3d> passageCentroids;
    std::vector<PassageContext>  passageContexts;
    double                       timestamp{0.0};
    std::string                  timestampProvenance{"steady_clock"};
    std::string                  roomTag;
    bool                         wasConfirmedRoom{false};
    /*! Whether the UAV had entered the room at snapshot time. Mission truth
     *  restored alongside identity; never consulted by creation, promotion,
     *  retirement, or merge paths. */
    bool                         wasPreviouslyVisited{false};
    int                          boundaryStatus{0};
};
} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // ROOM_CONTEXT_SNAPSHOT_H

/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            Floor.h
 *
 * @brief           Declares Floor, one storey of the building and the rooms on
 *                  it.
 */

#ifndef FLOOR_H
#define FLOOR_H

#include "Semantic/FloorStatus.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"

#include <Eigen/Core>
#include <cstddef>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace vs_graphs
{
namespace core
{
class Map;
} // namespace core
} // namespace vs_graphs

namespace vs_graphs
{
namespace core
{
namespace semantic
{
class Room;

class Floor
{
  public:
    /*! Comparable, normalized ground-plane identity and its observation
     * quality. */
    struct PlaneIdentity
    {
        Eigen::Vector4d equation_World{Eigen::Vector4d::Zero()};
        std::size_t     finiteSupportCount{0U};
        std::size_t     observationCount{0U};
    };

    static constexpr double kMergeMaxPlaneNormalAngle_deg = 10.0;
    static constexpr double kMergeMaxPlaneOffset_m        = 0.35;

  private:
    int             id;       // Floor's ID
    int             opId;     // Floor's ID in the local optimizer
    int             opIdG;    // Floor's ID in the global optimizer
    std::string     name;     // The name devoted for each room (optional)
    Eigen::Vector3d centroid; // Floor's centroid in the global reference
    std::vector<vs_graphs::core::semantic::Room *>
                                 rooms; // Floor's rooms and corridors
    std::optional<PlaneIdentity> planeIdentity;

    [[nodiscard]] FloorStatus
        detachRoom(vs_graphs::core::semantic::Room *p_room_inout);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Floor() :
        id(-1),
        opId(-1),
        opIdG(-1),
        name(""),
        centroid(Eigen::Vector3d::Zero()),
        p_map(nullptr)
    {}

    /*!
     * @brief       Apply a rigid/similarity transform to the plane geometry.
     *
     *              Updates the centroid, point cloud and plane equations so
     *              that the plane remains consistent with the merged map frame.
     *
     * @param[in]   transform_oldWorldToNewWorld_in
     *              Transform from the current plane frame to the new map frame.
     */
    [[nodiscard]] FloorStatus
        applyTransform(const g2o::Sim3 &transform_oldWorldToNewWorld_in);

    [[nodiscard]] FloorStatus getId(int &id_out) const;
    [[nodiscard]] FloorStatus setId(int value_in);

    [[nodiscard]] FloorStatus getOpId(int &opId_out) const;
    [[nodiscard]] FloorStatus setOpId(int value_in);

    [[nodiscard]] FloorStatus getOpIdG(int &opIdG_out) const;
    [[nodiscard]] FloorStatus setOpIdG(int value_in);

    [[nodiscard]] FloorStatus getName(std::string &name_out) const;
    [[nodiscard]] FloorStatus setName(std::string value_in);

    [[nodiscard]] FloorStatus getCentroid(Eigen::Vector3d &centroid_out) const;
    [[nodiscard]] FloorStatus setCentroid(Eigen::Vector3d value_in);

    /*! Returns true when a finite normalized ground-plane identity is stored.
     */
    [[nodiscard]] FloorStatus
        hasPlaneIdentity(bool &hasPlaneIdentity_out) const;

    /*! Returns one atomic snapshot of the equation and its evidence quality. */
    [[nodiscard]] FloorStatus getPlaneIdentity(
        std::optional<Floor::PlaneIdentity> &planeIdentity_out) const;

    /*!
     * Stores a normalized equation and evidence. Invalid equations are ignored
     * so a previously valid identity is never overwritten by missing data.
     */
    [[nodiscard]] FloorStatus
        setPlaneIdentity(const Eigen::Vector4d &equation_World_in,
                         std::size_t            finiteSupportCount_in,
                         std::size_t            observationCount_in);

    /*! Marks the current ground-plane identity unavailable. */
    [[nodiscard]] FloorStatus clearPlaneIdentity(void);

    /*! Transforms an identity under the active old-world to new-world Sim3. */
    [[nodiscard]] static FloorStatus transformPlaneIdentity(
        const PlaneIdentity                 &identity_OldWorld_in,
        const g2o::Sim3                     &transform_oldWorldToNewWorld_in,
        std::optional<Floor::PlaneIdentity> &transformedIdentity_out);

    /*! Compares normalized planes with sign-invariant angle and offset tests.
     */
    [[nodiscard]] static FloorStatus
        planeIdentitiesMatch(const PlaneIdentity &firstIdentity_in,
                             const PlaneIdentity &secondIdentity_in,
                             double               maximumNormalAngle_deg_in,
                             double               maximumOffset_m_in,
                             double              &normalAngle_deg_inout,
                             double              &offset_m_inout,
                             bool                &isMatch_out);

    /*! Selects the valid identity with most finite support, then observations.
     */
    [[nodiscard]] static FloorStatus
        selectBestObservedFloor(const std::vector<Floor *> &floors_in,
                                Floor                     *&p_bestFloor_out);

    [[nodiscard]] FloorStatus
        addRoom(vs_graphs::core::semantic::Room *p_value_inout);
    [[nodiscard]] FloorStatus getRooms(
        std::vector<vs_graphs::core::semantic::Room *> &rooms_out) const;
    [[nodiscard]] FloorStatus setRooms(
        const std::vector<vs_graphs::core::semantic::Room *> &value_in);

    /*!
     * @brief       Replaces a retired room association after consolidation.
     *
     * @param[in,out] p_retiredRoom_inout
     *              Structural element which is being retired.
     *
     * @param[in,out] p_retainedRoom_inout
     *              Structural element which absorbs the relationship.
     *
     * @param[out] wasRoomReplaced_out True when the retired room was present.
     * @return FLOOR_STATUS_SUCCESS, or FLOOR_STATUS_INVALID_ARGUMENT when an
     * input is rejected.
     */
    [[nodiscard]] FloorStatus
        replaceRoom(vs_graphs::core::semantic::Room *p_retiredRoom_inout,
                    vs_graphs::core::semantic::Room *p_retainedRoom_inout,
                    bool                            &wasRoomReplaced_out);

    [[nodiscard]] FloorStatus getMap(vs_graphs::core::Map *&p_map_out);
    [[nodiscard]] FloorStatus setMap(vs_graphs::core::Map *p_map_in);

  protected:
    vs_graphs::core::Map *p_map{nullptr};
    std::mutex            mapMutex;
    mutable std::mutex    roomsMutex;
    mutable std::mutex    geometryMutex;
};
} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif

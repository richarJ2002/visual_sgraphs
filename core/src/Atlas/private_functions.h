/*!
 * @file            private_functions.h
 *
 * @brief           Declares module-internal helpers shared between the
 *                  Atlas translation units.
 *
 * @note            These helpers were file-scope entities inside the
 *                  anonymous namespace of Atlas.cc; external linkage here
 *                  is module-internal only. Names are kept verbatim
 *                  (identifier renaming is a separate step).
 */

#ifndef VS_GRAPHS_CORE_ATLAS_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_ATLAS_PRIVATE_FUNCTIONS_H

#include "AtlasStatus.h"
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <iostream>
#include <rclcpp/logging.hpp>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{

class Map;

namespace geometric
{
class Plane;
} // namespace geometric

namespace semantic
{
class Passage;
} // namespace semantic

/*!
 * @brief           Plans the ids that entities imported from a merged map
 *                  receive in the destination map. An imported entity keeps its
 *                  id when that id is not negative and the destination does not
 *                  use it; otherwise it gets the lowest unused id above the
 *                  highest reserved one.
 *
 * @param[in]       existingEntities_in
 *                  Entities already in the destination map; null entries are
 *                  skipped.
 *
 * @param[in]       importedEntities_in
 *                  Entities coming from the source map; null entries are
 *                  skipped.
 *
 * @param[in]       entityName_in
 *                  Entity kind, only used in the abort message.
 *
 * @param[out]      assignments_out
 *                  Imported entity and its planned id, in ascending order of
 *                  the original id; cleared and refilled only when planning
 *                  succeeds.
 *
 * @param[out]      isPlanned_out
 *                  False, with a message on stderr, when an imported entity is
 *                  already in the destination or two imported entities share an
 *                  id; true otherwise.
 *
 * @return          ATLAS_STATUS_SUCCESS always.
 */
template <typename Entity>
[[nodiscard]] AtlasStatus
    planImportedIds(const std::vector<Entity *>           &existingEntities_in,
                    const std::vector<Entity *>           &importedEntities_in,
                    const char                            *entityName_in,
                    std::vector<std::pair<Entity *, int>> &assignments_out,
                    bool                                  &isPlanned_out)
{
    // Identity of an imported or existing entity. Every semantic status
    // enumeration uses 0 for SUCCESS; getId cannot fail.
    const auto idOf = [](const Entity *p_entity_in)
    {
        int entityId = -1;
        if (p_entity_in->getId(entityId) !=
            decltype(p_entity_in->getId(entityId)){})
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        return entityId;
    };
    const std::set<Entity *> importedEntities(importedEntities_in.begin(),
                                              importedEntities_in.end());
    std::set<int>            destinationIds;

    for (Entity *p_entity : existingEntities_in)
    {
        if (p_entity == nullptr)
        {
            continue;
        }

        if (importedEntities.count(p_entity) > 0U)
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source "
                      << entityName_in
                      << " already belongs to the destination container."
                      << std::endl;
            isPlanned_out = false;
            return AtlasStatus::ATLAS_STATUS_SUCCESS;
        }

        if (idOf(p_entity) >= 0)
        {
            destinationIds.insert(idOf(p_entity));
        }
    }

    std::vector<Entity *> orderedImportedEntities;
    orderedImportedEntities.reserve(importedEntities_in.size());
    std::set<int> importedOriginalIds;

    for (Entity *p_entity : importedEntities_in)
    {
        if (p_entity == nullptr)
        {
            continue;
        }

        if (!importedOriginalIds.insert(idOf(p_entity)).second)
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: duplicate "
                      << entityName_in << " ID " << idOf(p_entity)
                      << " in source map." << std::endl;
            isPlanned_out = false;
            return AtlasStatus::ATLAS_STATUS_SUCCESS;
        }

        orderedImportedEntities.push_back(p_entity);
    }

    std::sort(orderedImportedEntities.begin(),
              orderedImportedEntities.end(),
              [&idOf](const Entity *p_first, const Entity *p_second)
              { return idOf(p_first) < idOf(p_second); });

    std::set<int> reservedIds = destinationIds;
    for (Entity *p_entity : orderedImportedEntities)
    {
        if (idOf(p_entity) >= 0 && destinationIds.count(idOf(p_entity)) == 0U)
        {
            reservedIds.insert(idOf(p_entity));
        }
    }

    int nextAvailableId = reservedIds.empty() ? 0 : *reservedIds.rbegin() + 1;
    assignments_out.clear();
    assignments_out.reserve(orderedImportedEntities.size());

    for (Entity *p_entity : orderedImportedEntities)
    {
        int assignedId = idOf(p_entity);
        if (assignedId < 0 || destinationIds.count(assignedId) > 0U)
        {
            while (reservedIds.count(nextAvailableId) > 0U)
            {
                ++nextAvailableId;
            }
            assignedId = nextAvailableId++;
        }

        reservedIds.insert(assignedId);
        assignments_out.emplace_back(p_entity, assignedId);
    }

    isPlanned_out = true;
    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

/*!
 * @brief           Moves an identity counter past an identity that already
 *                  exists, so the next reservation cannot repeat it. The
 *                  counter never goes down; a negative identity is ignored.
 *
 * @param[in,out]   nextIdentity_inout
 *                  Counter holding the next identity to hand out.
 *
 * @param[in]       observedIdentity_in
 *                  Identity already in use.
 *
 * @return          ATLAS_STATUS_SUCCESS always.
 */
[[nodiscard]] AtlasStatus
    advanceIdentityAllocator(std::atomic<int> &nextIdentity_inout,
                             const int         observedIdentity_in);

/*!
 * @brief           Counts the rooms that are not marked bad in a map. A null
 *                  map counts as 0.
 *
 * @param[in]       p_map_in
 *                  Map to inspect; borrowed.
 *
 * @param[out]      liveRooms_out
 *                  Number of live rooms.
 *
 * @return          ATLAS_STATUS_SUCCESS always.
 */
[[nodiscard]] AtlasStatus countLiveRooms(Map         *p_map_in,
                                         std::size_t &liveRooms_out);

/*!
 * @brief           Counts the planes that are not marked bad and are wall
 *                  planes in a map. A null map counts as 0.
 *
 * @param[in]       p_map_in
 *                  Map to inspect; borrowed.
 *
 * @param[out]      liveWallPlanes_out
 *                  Number of live wall planes.
 *
 * @return          ATLAS_STATUS_SUCCESS always.
 */
[[nodiscard]] AtlasStatus countLiveWallPlanes(Map         *p_map_in,
                                              std::size_t &liveWallPlanes_out);

/*!
 * @brief           Counts the passages that are not marked bad in a map. A null
 *                  map counts as 0.
 *
 * @param[in]       p_map_in
 *                  Map to inspect; borrowed.
 *
 * @param[out]      livePassages_out
 *                  Number of live passages.
 *
 * @return          ATLAS_STATUS_SUCCESS always.
 */
[[nodiscard]] AtlasStatus countLivePassages(Map         *p_map_in,
                                            std::size_t &livePassages_out);

/*!
 * @brief           Counts the floors that have a plane identity in a map. A
 *                  null map counts as 0.
 *
 * @param[in]       p_map_in
 *                  Map to inspect; borrowed.
 *
 * @param[out]      liveFloors_out
 *                  Number of live floors.
 *
 * @return          ATLAS_STATUS_SUCCESS always.
 */
[[nodiscard]] AtlasStatus countLiveFloors(Map         *p_map_in,
                                          std::size_t &liveFloors_out);

/*!
 * @brief           Condenses the live room, wall plane, passage and floor
 *                  counts of the old map and then of the current map into one
 *                  number. Two equal numbers mean neither map gained or lost
 *                  such content, so a merge attempt can be skipped. A null map
 *                  counts as empty.
 *
 * @param[in]       p_oldMap_in
 *                  Older map of the pair; borrowed.
 *
 * @param[in]       p_currentMap_in
 *                  Current map of the pair; borrowed.
 *
 * @param[out]      contentHash_out
 *                  Combined count value (counts folded with a factor of 31).
 *
 * @return          ATLAS_STATUS_SUCCESS always.
 */
[[nodiscard]] AtlasStatus consecutiveContentHash(Map         *p_oldMap_in,
                                                 Map         *p_currentMap_in,
                                                 std::size_t &contentHash_out);

[[nodiscard]] AtlasStatus consecutiveSeedTagsMatch(Map  *p_oldMap_in,
                                                   Map  *p_currentMap_in,
                                                   bool &isMatch_out);

/*!
 * @brief           Collects the room tags that appear on a live room in both
 *                  maps; such rooms anchor the old map to the current one.
 *                  Rooms that are bad, untagged or have an empty tag are
 *                  ignored. The set is empty when either map is null.
 *
 * @param[in]       p_oldMap_in
 *                  Older map of the pair; borrowed.
 *
 * @param[in]       p_currentMap_in
 *                  Current map of the pair; borrowed.
 *
 * @param[out]      anchorTags_out
 *                  Tags present in both maps.
 *
 * @return          ATLAS_STATUS_SUCCESS always.
 */
[[nodiscard]] AtlasStatus
    collectAnchorTags(Map                   *p_oldMap_in,
                      Map                   *p_currentMap_in,
                      std::set<std::string> &anchorTags_out);

[[nodiscard]] AtlasStatus
    resurfaceProxyFromTransferred(semantic::Passage *p_proxy_inout,
                                  semantic::Passage *p_transferred_in,
                                  bool              &wasResurfaced_out);

} // namespace core
} // namespace vs_graphs

#endif /* VS_GRAPHS_CORE_ATLAS_PRIVATE_FUNCTIONS_H */

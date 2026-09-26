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

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <iostream>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{

class Map;

namespace semantic
{
class Passage;
} // namespace semantic

template <typename Entity>
bool planImportedIds(const std::vector<Entity *>           &existingEntities_in,
                     const std::vector<Entity *>           &importedEntities_in,
                     const char                            *entityName_in,
                     std::vector<std::pair<Entity *, int>> &assignments_out)
{
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
            return false;
        }

        if (p_entity->getId() >= 0)
        {
            destinationIds.insert(p_entity->getId());
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

        if (!importedOriginalIds.insert(p_entity->getId()).second)
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: duplicate "
                      << entityName_in << " ID " << p_entity->getId()
                      << " in source map." << std::endl;
            return false;
        }

        orderedImportedEntities.push_back(p_entity);
    }

    std::sort(orderedImportedEntities.begin(),
              orderedImportedEntities.end(),
              [](const Entity *p_first, const Entity *p_second)
              { return p_first->getId() < p_second->getId(); });

    std::set<int> reservedIds = destinationIds;
    for (Entity *p_entity : orderedImportedEntities)
    {
        if (p_entity->getId() >= 0 &&
            destinationIds.count(p_entity->getId()) == 0U)
        {
            reservedIds.insert(p_entity->getId());
        }
    }

    int nextAvailableId = reservedIds.empty() ? 0 : *reservedIds.rbegin() + 1;
    assignments_out.clear();
    assignments_out.reserve(orderedImportedEntities.size());

    for (Entity *p_entity : orderedImportedEntities)
    {
        int assignedId = p_entity->getId();
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

    return true;
}

void advanceIdentityAllocator(std::atomic<int> &nextIdentity_inout,
                              const int         observedIdentity_in);

std::size_t countLiveRooms(Map *p_map_in);

std::size_t countLiveWallPlanes(Map *p_map_in);

std::size_t countLivePassages(Map *p_map_in);

std::size_t countLiveFloors(Map *p_map_in);

std::size_t consecutiveContentHash(Map *p_oldMap_in, Map *p_currentMap_in);

bool consecutiveSeedTagsMatch(Map *p_oldMap_in, Map *p_currentMap_in);

std::set<std::string> collectAnchorTags(Map *p_oldMap_in, Map *p_currentMap_in);

bool resurfaceProxyFromTransferred(semantic::Passage *p_proxy_inout,
                                   semantic::Passage *p_transferred_in);

} // namespace core
} // namespace vs_graphs

#endif /* VS_GRAPHS_CORE_ATLAS_PRIVATE_FUNCTIONS_H */

/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 */

#include "Atlas.h"
#include "Map.h"
#include "Semantic/Room.h"

#include <gtest/gtest.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace vs_graphs
{
namespace core
{
namespace
{

struct SnapshotHookBarrier
{
    std::mutex              mutex;
    std::condition_variable condition;
    bool                    reached  = false;
    bool                    released = false;
};

SnapshotHookBarrier *pSnapshotHookBarrier = nullptr;

} // namespace

extern "C" void vsGraphsAtlasLockOrderBeforeMapSnapshot()
{
    SnapshotHookBarrier         &barrier = *pSnapshotHookBarrier;
    std::unique_lock<std::mutex> lock(barrier.mutex);
    barrier.reached = true;
    barrier.condition.notify_one();
    barrier.condition.wait(lock, [&barrier]() { return barrier.released; });
}

namespace
{

class TestAtlas : public Atlas
{
  public:
    explicit TestAtlas(int initialKeyFrameId_in) :
        Atlas(initialKeyFrameId_in)
    {}

    std::unique_lock<std::mutex> lockAtlas()
    {
        return std::unique_lock<std::mutex>(atlasMutex);
    }

    bool tryLockRoomContext()
    {
        if (!roomContextMutex.try_lock())
        {
            return false;
        }

        roomContextMutex.unlock();
        return true;
    }
};

TEST(AtlasLockOrder, MatchingDoesNotHoldRoomContextWhileWaitingForAtlas)
{
    TestAtlas atlas(0);

    semantic::Room priorRoom;
    priorRoom.setId(42);
    priorRoom.setCentroid(Eigen::Vector3d::Zero());
    atlas.getCurrentMap()->addDetectedMapRoom(&priorRoom);

    atlas.createNewMap();
    Map       *pNewMap = atlas.getCurrentMap();
    const auto history = atlas.copyRoomContextHistory();
    ASSERT_EQ(history.count(0U), 1U);
    ASSERT_EQ(history.at(0U).size(), 1U);
    EXPECT_EQ(history.at(0U).front().wallNormals.size(), 0U);
    EXPECT_EQ(history.at(0U).front().wallBounds.size(), 0U);

    semantic::Room newRoom;
    newRoom.setId(84);
    newRoom.setCentroid(Eigen::Vector3d::Zero());
    pNewMap->addDetectedMapRoom(&newRoom);

    atlas.matchRoomsToContext(pNewMap);
    EXPECT_TRUE(atlas.tryLockRoomContext());
    EXPECT_TRUE(newRoom.getRoomTag().empty());
}

TEST(AtlasLockOrder, NewMapEventIsConsumedExactlyOnce)
{
    Atlas atlas(0);
    EXPECT_TRUE(atlas.consumeNewMapCreatedEvent());
    EXPECT_FALSE(atlas.consumeNewMapCreatedEvent());
    atlas.createNewMap();
    EXPECT_TRUE(atlas.consumeNewMapCreatedEvent());
    EXPECT_FALSE(atlas.consumeNewMapCreatedEvent());
}

TEST(AtlasLockOrder, EventAndHistoryCopiesAreSafeWithoutBorrowedEntities)
{
    Atlas             atlas(0);
    Map              *pStableMap = atlas.getCurrentMap();
    std::atomic<bool> complete{false};
    std::thread       reader(
        [&atlas, &complete, pStableMap]()
        {
            for (unsigned int iteration = 0U; iteration < 100U; ++iteration)
            {
                const auto history = atlas.copyRoomContextHistory();
                (void)history;
                const auto liveSnapshot =
                    atlas.copyRoomContextForMap(pStableMap);
                (void)liveSnapshot;
                (void)atlas.consumeNewMapCreatedEvent();
            }
            complete.store(true, std::memory_order_release);
        });
    for (unsigned int iteration = 0U; iteration < 100U; ++iteration)
    {
        atlas.createNewMap();
    }
    reader.join();
    EXPECT_TRUE(complete.load(std::memory_order_acquire));
}

} // namespace
} // namespace core
} // namespace vs_graphs

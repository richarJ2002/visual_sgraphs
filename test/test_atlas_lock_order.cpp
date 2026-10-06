/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 */

/*!
 * @file            test_atlas_lock_order.cpp
 *
 * @brief           Tests the order in which Atlas takes its locks
 *                  (AtlasLockOrder), through the test hook in
 *                  Atlas::matchRoomsToContext(), and that the camera registry
 *                  takes atlasMutex.
 */

#include "Atlas.h"
#include "Map.h"
#include "Semantic/Room.h"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <thread>
#include <vector>

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

/*!
 * @brief           Test hook that Atlas::matchRoomsToContext() calls, when
 *                  built with the hook enabled, just before it snapshots the
 *                  maps. It tells the test the matcher has reached that point,
 *                  then blocks until the test releases it.
 *
 * @pre             pSnapshotHookBarrier points to a live barrier; it is
 *                  dereferenced without a null check.
 */
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

/*!
 * @brief           Runs a call on another thread while the test holds
 *                  atlasMutex and checks that the call waits for the mutex,
 *                  then finishes once the mutex is released.
 *
 * @param[in,out]   testAtlas_inout
 *                  Atlas whose mutex is held during the wait.
 *
 * @param[in]       atlasCall_in
 *                  Call that must take atlasMutex.
 */
void expectCallWaitsForAtlasMutex(TestAtlas                   &testAtlas_inout,
                                  const std::function<void()> &atlasCall_in)
{
    std::atomic<bool>            hasFinished{false};
    std::unique_lock<std::mutex> atlasLock = testAtlas_inout.lockAtlas();
    std::thread                  callerThread(
        [&atlasCall_in, &hasFinished]()
        {
            atlasCall_in();
            hasFinished.store(true);
        });
    /* Long enough for a call that skips the mutex to finish. */
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_FALSE(hasFinished.load());
    atlasLock.unlock();
    callerThread.join();
    EXPECT_TRUE(hasFinished.load());
}

TEST(AtlasLockOrder, MatchingDoesNotHoldRoomContextWhileWaitingForAtlas)
{
    TestAtlas atlas(0);

    semantic::Room priorRoom;
    ASSERT_EQ((priorRoom.setId(42)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((priorRoom.setCentroid(Eigen::Vector3d::Zero())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    Map *p_currentMap = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_currentMap)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((p_currentMap->addDetectedMapRoom(&priorRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *pNewMap = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(pNewMap)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    std::map<unsigned long, std::vector<semantic::RoomContextSnapshot>>
        history{};
    ASSERT_EQ((atlas.copyRoomContextHistory(history)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ(history.count(0U), 1U);
    ASSERT_EQ(history.at(0U).size(), 1U);
    EXPECT_EQ(history.at(0U).front().wallNormals.size(), 0U);
    EXPECT_EQ(history.at(0U).front().wallBounds.size(), 0U);

    semantic::Room newRoom;
    ASSERT_EQ((newRoom.setId(84)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((newRoom.setCentroid(Eigen::Vector3d::Zero())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((pNewMap->addDetectedMapRoom(&newRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    ASSERT_EQ((atlas.matchRoomsToContext(pNewMap)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    EXPECT_TRUE(atlas.tryLockRoomContext());
    std::string roomTag{};
    ASSERT_EQ((newRoom.getRoomTag(roomTag)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_TRUE(roomTag.empty());
}

TEST(AtlasLockOrder, NewMapEventIsConsumedExactlyOnce)
{
    Atlas atlas(0);
    bool  wasEventPending{};
    ASSERT_EQ((atlas.consumeNewMapCreatedEvent(wasEventPending)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    EXPECT_TRUE(wasEventPending);
    bool wasEventPending2{};
    ASSERT_EQ((atlas.consumeNewMapCreatedEvent(wasEventPending2)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    EXPECT_FALSE(wasEventPending2);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    bool wasEventPending3{};
    ASSERT_EQ((atlas.consumeNewMapCreatedEvent(wasEventPending3)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    EXPECT_TRUE(wasEventPending3);
    bool wasEventPending4{};
    ASSERT_EQ((atlas.consumeNewMapCreatedEvent(wasEventPending4)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    EXPECT_FALSE(wasEventPending4);
}

TEST(AtlasLockOrder, EventAndHistoryCopiesAreSafeWithoutBorrowedEntities)
{
    Atlas atlas(0);
    Map  *pStableMap = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(pStableMap)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    std::atomic<bool> complete{false};
    std::thread       reader(
        [&atlas, &complete, pStableMap]()
        {
            for (unsigned int iteration = 0U; iteration < 100U; ++iteration)
            {
                std::map<unsigned long,
                               std::vector<semantic::RoomContextSnapshot>>
                    history{};
                if (atlas.copyRoomContextHistory(history) !=
                    AtlasStatus::ATLAS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: copyRoomContextHistory returned a failure status "
                              "although it cannot fail; continuing as before.",
                        __func__);
                }
                (void)history;
                std::vector<semantic::RoomContextSnapshot> liveSnapshot{};
                if (atlas.copyRoomContextForMap(pStableMap, liveSnapshot) !=
                    AtlasStatus::ATLAS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: copyRoomContextForMap returned a failure status "
                              "although it cannot fail; continuing as before.",
                        __func__);
                }
                (void)liveSnapshot;
                bool atlasWasEventPending{};
                if (atlas.consumeNewMapCreatedEvent(atlasWasEventPending) !=
                    AtlasStatus::ATLAS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: consumeNewMapCreatedEvent returned a failure "
                              "status although it cannot fail; continuing as before.",
                        __func__);
                }
                (void)atlasWasEventPending;
            }
            complete.store(true, std::memory_order_release);
        });
    for (unsigned int iteration = 0U; iteration < 100U; ++iteration)
    {
        ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    }
    reader.join();
    EXPECT_TRUE(complete.load(std::memory_order_acquire));
}

/*!
 * @brief           Checks that addCamera() and getAllCameras() wait for
 *                  atlasMutex, and that the camera they register and return is
 *                  the borrowed one the caller passed in.
 */
TEST(AtlasLockOrder, CameraRegistryWaitsForAtlasMutex)
{
    /* Declared first: the camera outlives the Atlas that borrows it. */
    camera_models::pinhole::Pinhole pinholeCamera(
        std::vector<float>{500.0F, 500.0F, 320.0F, 240.0F});
    TestAtlas testAtlas(0);

    camera_models::geometriccamera::GeometricCamera *p_registeredCamera =
        nullptr;
    expectCallWaitsForAtlasMutex(
        testAtlas,
        [&testAtlas, &pinholeCamera, &p_registeredCamera]()
        {
            EXPECT_EQ((testAtlas.addCamera(&pinholeCamera, p_registeredCamera)),
                      AtlasStatus::ATLAS_STATUS_SUCCESS);
        });
    EXPECT_EQ(p_registeredCamera, &pinholeCamera);

    std::vector<camera_models::geometriccamera::GeometricCamera *> allCameras{};
    expectCallWaitsForAtlasMutex(testAtlas,
                                 [&testAtlas, &allCameras]()
                                 {
                                     EXPECT_EQ(
                                         (testAtlas.getAllCameras(allCameras)),
                                         AtlasStatus::ATLAS_STATUS_SUCCESS);
                                 });
    ASSERT_EQ(allCameras.size(), 1U);
    EXPECT_EQ(allCameras[0], &pinholeCamera);
}

} // namespace
} // namespace core
} // namespace vs_graphs

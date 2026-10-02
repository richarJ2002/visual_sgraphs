/*!
 * @file            test_room_tracker_integration.cpp
 *
 * @brief           Integration tests for the room tracker inside the production
 *                  pipeline (RoomTrackerProductionIntegration).
 */

/*
 * Focused production seam tests for event delivery.
 */

#include "Atlas.h"
#include "GeoSemHelpers.h"
#include "KeyFrame.h"
#include "Semantic/Passage.h"
#include "Semantic/Room.h"
#include "SemanticsManager.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <thread>
#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace
{
semantic::VerificationVerdict passVerdict()
{
    semantic::VerificationVerdict verdict;
    verdict.status      = semantic::VerificationStatus::PASS;
    verdict.hasPassed   = true;
    verdict.inlierCount = 3U;
    verdict.inlierRatio = 1.0;
    verdict.confidence  = 1.0;
    return verdict;
}

semantic::VerificationVerdict rejectedVerdict()
{
    semantic::VerificationVerdict verdict;
    verdict.status = semantic::VerificationStatus::REJECTED;
    return verdict;
}

std::size_t countAcceptedEvents(
    const std::vector<semantic::TransitionEvent> &history_in,
    semantic::RoomTrackingEvent                   event_in)
{
    return static_cast<std::size_t>(std::count_if(
        history_in.begin(),
        history_in.end(),
        [event_in](const semantic::TransitionEvent &record_in)
        { return record_in.isAccepted && record_in.event == event_in; }));
}

class ProductionCrossingScene
{
  public:
    explicit ProductionCrossingScene(unsigned long keyFrameIdBase_in) :
        atlas(0),
        p_map(nullptr),
        manager(&atlas),
        returnKeyFrameAdded(false),
        thirdCrossingKeyFrameAdded(false),
        groundPlaneValid(false)
    {
        bool atlasWasEventPending{};
        EXPECT_EQ((atlas.consumeNewMapCreatedEvent(atlasWasEventPending)),
                  AtlasStatus::ATLAS_STATUS_SUCCESS);
        static_cast<void>(atlasWasEventPending);
        EXPECT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
        Map *p_atlasCurrentMap = nullptr;
        EXPECT_EQ((atlas.getCurrentMap(p_atlasCurrentMap)),
                  AtlasStatus::ATLAS_STATUS_SUCCESS);
        p_map = p_atlasCurrentMap;
        bool atlasWasEventPending2{};
        EXPECT_EQ((atlas.consumeNewMapCreatedEvent(atlasWasEventPending2)),
                  AtlasStatus::ATLAS_STATUS_SUCCESS);
        static_cast<void>(atlasWasEventPending2);

        EXPECT_EQ((groundPlane.setId(0)),
                  geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        EXPECT_EQ((groundPlane.setMap(p_map)),
                  geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        EXPECT_EQ(
            (groundPlane.setPlaneType(geometric::Plane::PlaneVariant::GROUND)),
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        EXPECT_EQ((groundPlane.setGlobalEquation(
                      g2o::Plane3D(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0)))),
                  geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr groundCloud(
            new pcl::PointCloud<pcl::PointXYZRGBA>);
        for (int xIndex = 0; xIndex < 5; ++xIndex)
        {
            for (int yIndex = 0; yIndex < 5; ++yIndex)
            {
                pcl::PointXYZRGBA point;
                point.x = static_cast<float>(xIndex) * 0.25F;
                point.y = static_cast<float>(yIndex) * 0.25F;
                point.z = 0.0F;
                groundCloud->push_back(point);
            }
        }
        EXPECT_EQ((groundPlane.setMapClouds(groundCloud)),
                  geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        bool wasPlaneRefit{};
        EXPECT_EQ((GeoSemHelpers::refitMappedPlaneFromCloud(&groundPlane,
                                                            wasPlaneRefit)),
                  GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS);
        groundPlaneValid = wasPlaneRefit;
        EXPECT_EQ((p_map->addMapPlane(&groundPlane)),
                  MapStatus::MAP_STATUS_SUCCESS);

        EXPECT_EQ((knownWall.setId(1)),
                  geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        EXPECT_EQ((knownWall.setMap(p_map)),
                  geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        EXPECT_EQ(
            (knownWall.setPlaneType(geometric::Plane::PlaneVariant::WALL)),
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        EXPECT_EQ((farWall.setId(2)),
                  geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        EXPECT_EQ((farWall.setMap(p_map)),
                  geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        EXPECT_EQ((farWall.setPlaneType(geometric::Plane::PlaneVariant::WALL)),
                  geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        EXPECT_EQ((p_map->addMapPlane(&knownWall)),
                  MapStatus::MAP_STATUS_SUCCESS);
        EXPECT_EQ((p_map->addMapPlane(&farWall)),
                  MapStatus::MAP_STATUS_SUCCESS);

        EXPECT_EQ((knownRoom.setId(10)),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        EXPECT_EQ((knownRoom.setMap(p_map)),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        EXPECT_EQ((knownRoom.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        EXPECT_EQ((knownRoom.setWalls(&knownWall)),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        EXPECT_EQ((farRoom.setId(11)),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        EXPECT_EQ((farRoom.setMap(p_map)),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        EXPECT_EQ((farRoom.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        EXPECT_EQ((farRoom.setWalls(&farWall)),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        EXPECT_EQ((p_map->addDetectedMapRoom(&knownRoom)),
                  MapStatus::MAP_STATUS_SUCCESS);
        EXPECT_EQ((p_map->addDetectedMapRoom(&farRoom)),
                  MapStatus::MAP_STATUS_SUCCESS);

        EXPECT_EQ(
            (passage.setId(20)),
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
        EXPECT_EQ(
            (passage.setMap(p_map)),
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
        EXPECT_EQ(
            (passage.setPassable(true)),
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
        EXPECT_EQ(
            (passage.setWidth(2.0)),
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
        EXPECT_EQ(
            (passage.setHeight(2.0)),
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
        EXPECT_EQ(
            (passage.setCentroid(Eigen::Vector3d(0.0, 0.0, 1.0))),
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
        EXPECT_EQ(
            (passage.setGlobalEquation(
                g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)))),
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
        EXPECT_EQ(
            (passage.setKnownSideRoom(&knownRoom)),
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
        EXPECT_EQ(
            (passage.setKnownSideDirection(Eigen::Vector3d(-1.0, 0.0, 0.0))),
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
        EXPECT_EQ(
            (passage.setProspectiveRoom(&farRoom)),
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
        EXPECT_EQ((p_map->addMapPassage(&passage)),
                  MapStatus::MAP_STATUS_SUCCESS);

        addKeyFrame(knownSideKeyFrame,
                    keyFrameIdBase_in,
                    Eigen::Vector3f(-1.0F, 0.0F, 1.0F));
        addKeyFrame(farSideKeyFrame,
                    keyFrameIdBase_in + 1U,
                    Eigen::Vector3f(1.0F, 0.0F, 1.0F));
        returnSideKeyFrame.id = keyFrameIdBase_in + 2U;
        EXPECT_EQ((returnSideKeyFrame.setPose(
                      Sophus::SE3f(Eigen::Matrix3f::Identity(),
                                   Eigen::Vector3f(1.0F, 0.0F, -1.0F)))),
                  KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
        thirdCrossingKeyFrame.id = keyFrameIdBase_in + 3U;
        EXPECT_EQ((thirdCrossingKeyFrame.setPose(
                      Sophus::SE3f(Eigen::Matrix3f::Identity(),
                                   Eigen::Vector3f(-1.0F, 0.0F, 1.0F)))),
                  KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    }

    void confirmFirstRoom(double now_s_in)
    {
        ASSERT_EQ((manager.submitVerificationVerdict(passVerdict())),
                  SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
        ASSERT_EQ((manager.processRoomTrackerPendingForTest(now_s_in)),
                  SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    }

    void produceTraversalEvidence()
    {
        ASSERT_EQ((manager.updateTraversalEvidence(&atlas)),
                  SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    }

    void addReturnCrossing()
    {
        if (!returnKeyFrameAdded)
        {
            ASSERT_EQ((p_map->addKeyFrame(&returnSideKeyFrame)),
                      MapStatus::MAP_STATUS_SUCCESS);
            returnKeyFrameAdded = true;
        }
    }

    void addThirdCrossing()
    {
        if (!thirdCrossingKeyFrameAdded)
        {
            ASSERT_EQ((p_map->addKeyFrame(&thirdCrossingKeyFrame)),
                      MapStatus::MAP_STATUS_SUCCESS);
            thirdCrossingKeyFrameAdded = true;
        }
    }

    void addAlternatingCrossing(unsigned long keyFrameId_in, bool farSide_in)
    {
        std::unique_ptr<KeyFrame> p_keyFrame(new KeyFrame());
        p_keyFrame->id = keyFrameId_in;
        const Eigen::Vector3f cameraCenter_world_m =
            farSide_in ? Eigen::Vector3f(1.0F, 0.0F, 1.0F)
                       : Eigen::Vector3f(-1.0F, 0.0F, 1.0F);
        ASSERT_EQ((p_keyFrame->setPose(Sophus::SE3f(Eigen::Matrix3f::Identity(),
                                                    -cameraCenter_world_m))),
                  KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
        ASSERT_EQ((p_map->addKeyFrame(p_keyFrame.get())),
                  MapStatus::MAP_STATUS_SUCCESS);
        extraCrossingKeyFrames.push_back(std::move(p_keyFrame));
    }

    bool hasValidGroundPlane() const
    {
        return groundPlaneValid;
    }

    Atlas                                  atlas;
    Map                                   *p_map;
    SemanticsManager                       manager;
    geometric::Plane                       groundPlane;
    geometric::Plane                       knownWall;
    geometric::Plane                       farWall;
    semantic::Room                         knownRoom;
    semantic::Room                         farRoom;
    semantic::Passage                      passage;
    KeyFrame                               knownSideKeyFrame;
    KeyFrame                               farSideKeyFrame;
    KeyFrame                               returnSideKeyFrame;
    KeyFrame                               thirdCrossingKeyFrame;
    std::vector<std::unique_ptr<KeyFrame>> extraCrossingKeyFrames;

  private:
    void addKeyFrame(KeyFrame              &keyFrame_inout,
                     unsigned long          keyFrameId_in,
                     const Eigen::Vector3f &cameraCenter_world_m_in) const
    {
        keyFrame_inout.id = keyFrameId_in;
        ASSERT_EQ(
            (keyFrame_inout.setPose(Sophus::SE3f(Eigen::Matrix3f::Identity(),
                                                 -cameraCenter_world_m_in))),
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
        ASSERT_EQ((p_map->addKeyFrame(&keyFrame_inout)),
                  MapStatus::MAP_STATUS_SUCCESS);
    }

    bool returnKeyFrameAdded;
    bool thirdCrossingKeyFrameAdded;
    bool groundPlaneValid;
};

bool overlapProductionPublishAndConsume(ProductionCrossingScene &scene_inout,
                                        double                   now_s_in)
{
    std::mutex              synchronizationMutex;
    std::condition_variable synchronizationCondition;
    bool                    producerHasPendingMutex    = false;
    bool                    consumerHasStarted         = false;
    bool                    consumerObservedContention = false;

    if (scene_inout.manager.setRoomTrackerPendingPublishHookForTest(
            [&synchronizationMutex,
             &synchronizationCondition,
             &producerHasPendingMutex,
             &consumerHasStarted]()
            {
                std::unique_lock<std::mutex> synchronizationLock(
                    synchronizationMutex);
                producerHasPendingMutex = true;
                synchronizationCondition.notify_all();
                synchronizationCondition.wait(synchronizationLock,
                                              [&consumerHasStarted]()
                                              { return consumerHasStarted; });
            }) != SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: setRoomTrackerPendingPublishHookForTest returned a failure "
            "status although it cannot fail; continuing as before.",
            __func__);
    }

    std::thread producer([&scene_inout]()
                         { scene_inout.produceTraversalEvidence(); });

    {
        std::unique_lock<std::mutex> synchronizationLock(synchronizationMutex);
        synchronizationCondition.wait(synchronizationLock,
                                      [&producerHasPendingMutex]()
                                      { return producerHasPendingMutex; });
    }

    std::thread consumer(
        [&scene_inout,
         now_s_in,
         &synchronizationMutex,
         &synchronizationCondition,
         &consumerHasStarted,
         &consumerObservedContention]()
        {
            bool isLocked{};
            if (scene_inout.manager.tryLockRoomTrackerPendingMutexForTest(
                    isLocked) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: tryLockRoomTrackerPendingMutexForTest "
                             "returned a failure status although it cannot "
                             "fail; continuing as before.",
                             __func__);
            }
            consumerObservedContention = !isLocked;
            {
                std::lock_guard<std::mutex> synchronizationLock(
                    synchronizationMutex);
                consumerHasStarted = true;
            }
            synchronizationCondition.notify_all();
            if (scene_inout.manager.processRoomTrackerPendingForTest(
                    now_s_in) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: processRoomTrackerPendingForTest returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
        });

    producer.join();
    consumer.join();
    return consumerObservedContention;
}
} // namespace

TEST(RoomTrackerProductionIntegration, LossIsOneEventPerRecoveredEpisode)
{
    Atlas atlas(0);
    bool  wasEventPending{};
    ASSERT_EQ((atlas.consumeNewMapCreatedEvent(wasEventPending)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    EXPECT_TRUE(wasEventPending);
    SemanticsManager manager(&atlas);

    ASSERT_EQ((manager.submitVerificationVerdict(passVerdict())),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((manager.processRoomTrackerPendingForTest(0.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);

    std::thread firstLoss(
        [&manager]()
        {
            if (manager.onTrackingLost() !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: onTrackingLost returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        });
    std::thread secondLoss(
        [&manager]()
        {
            if (manager.onTrackingLost() !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: onTrackingLost returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        });
    firstLoss.join();
    secondLoss.join();
    ASSERT_EQ((manager.processRoomTrackerPendingForTest(1.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);

    const std::vector<semantic::TransitionEvent> *p_historyRef = nullptr;
    ASSERT_EQ((manager.getRoomTrackerEventHistoryForTest(p_historyRef)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    const std::vector<semantic::TransitionEvent> &history = *p_historyRef;
    ASSERT_EQ(history.size(), 2U);
    EXPECT_EQ(history.back().event, semantic::RoomTrackingEvent::TRACKING_LOST);
    int getLastKnownRoomId2{};
    ASSERT_EQ((manager.getLastKnownRoomId(getLastKnownRoomId2)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ(getLastKnownRoomId2, -1);

    ASSERT_EQ((manager.onTrackingRecovered()),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((manager.submitVerificationVerdict(passVerdict())),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((manager.processRoomTrackerPendingForTest(2.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((manager.submitVerificationVerdict(passVerdict())),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((manager.processRoomTrackerPendingForTest(3.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((manager.onTrackingLost()),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((manager.processRoomTrackerPendingForTest(4.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    const std::vector<semantic::TransitionEvent>
        *p_getRoomTrackerEventHistoryForTest = nullptr;
    ASSERT_EQ((manager.getRoomTrackerEventHistoryForTest(
                  p_getRoomTrackerEventHistoryForTest)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ((*p_getRoomTrackerEventHistoryForTest).size(), 5U);
}

TEST(RoomTrackerProductionIntegration,
     NewMapEventSurvivesUnavailableAndRejectedVerdicts)
{
    Atlas atlas(0);
    bool  wasEventPending{};
    ASSERT_EQ((atlas.consumeNewMapCreatedEvent(wasEventPending)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    EXPECT_TRUE(wasEventPending);
    SemanticsManager manager(&atlas);

    ASSERT_EQ((manager.submitVerificationVerdict(passVerdict())),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((manager.processRoomTrackerPendingForTest(0.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((manager.onTrackingLost()),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((manager.processRoomTrackerPendingForTest(1.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);

    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((manager.processRoomTrackerPendingForTest(2.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    const std::vector<semantic::TransitionEvent>
        *p_getRoomTrackerEventHistoryForTest = nullptr;
    ASSERT_EQ((manager.getRoomTrackerEventHistoryForTest(
                  p_getRoomTrackerEventHistoryForTest)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ((*p_getRoomTrackerEventHistoryForTest).size(), 2U);

    ASSERT_EQ((manager.submitVerificationVerdict(rejectedVerdict())),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((manager.processRoomTrackerPendingForTest(3.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    const std::vector<semantic::TransitionEvent>
        *p_getRoomTrackerEventHistoryForTest2 = nullptr;
    ASSERT_EQ((manager.getRoomTrackerEventHistoryForTest(
                  p_getRoomTrackerEventHistoryForTest2)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ((*p_getRoomTrackerEventHistoryForTest2).size(), 2U);

    ASSERT_EQ((manager.submitVerificationVerdict(passVerdict())),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((manager.processRoomTrackerPendingForTest(4.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    const std::vector<semantic::TransitionEvent>
        *p_getRoomTrackerEventHistoryForTest3 = nullptr;
    ASSERT_EQ((manager.getRoomTrackerEventHistoryForTest(
                  p_getRoomTrackerEventHistoryForTest3)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((*p_getRoomTrackerEventHistoryForTest3).size(), 3U);
    const std::vector<semantic::TransitionEvent>
        *p_getRoomTrackerEventHistoryForTest4 = nullptr;
    ASSERT_EQ((manager.getRoomTrackerEventHistoryForTest(
                  p_getRoomTrackerEventHistoryForTest4)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ((*p_getRoomTrackerEventHistoryForTest4).back().event,
              semantic::RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH);

    ASSERT_EQ((manager.submitVerificationVerdict(passVerdict())),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((manager.processRoomTrackerPendingForTest(5.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    const std::vector<semantic::TransitionEvent>
        *p_getRoomTrackerEventHistoryForTest5 = nullptr;
    ASSERT_EQ((manager.getRoomTrackerEventHistoryForTest(
                  p_getRoomTrackerEventHistoryForTest5)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((*p_getRoomTrackerEventHistoryForTest5).size(), 4U);
    const std::vector<semantic::TransitionEvent>
        *p_getRoomTrackerEventHistoryForTest6 = nullptr;
    ASSERT_EQ((manager.getRoomTrackerEventHistoryForTest(
                  p_getRoomTrackerEventHistoryForTest6)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ((*p_getRoomTrackerEventHistoryForTest6).back().event,
              semantic::RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM);
}

TEST(RoomTrackerProductionIntegration, AtlasMapEventIsConsumedExactlyOnce)
{
    Atlas atlas(0);
    bool  wasEventPending{};
    ASSERT_EQ((atlas.consumeNewMapCreatedEvent(wasEventPending)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    EXPECT_TRUE(wasEventPending);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);

    std::atomic<unsigned int> deliveries{0U};
    std::vector<std::thread>  consumers;
    for (unsigned int consumerIndex = 0U; consumerIndex < 8U; ++consumerIndex)
    {
        consumers.emplace_back(
            [&atlas, &deliveries]()
            {
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
                if (atlasWasEventPending)
                {
                    deliveries.fetch_add(1U, std::memory_order_relaxed);
                }
            });
    }
    for (std::thread &consumer : consumers)
    {
        consumer.join();
    }

    EXPECT_EQ(deliveries.load(std::memory_order_relaxed), 1U);
    bool wasEventPending2{};
    ASSERT_EQ((atlas.consumeNewMapCreatedEvent(wasEventPending2)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    EXPECT_FALSE(wasEventPending2);
}

TEST(RoomTrackerProductionIntegration,
     ProductionTraversalEvidenceReachesTrackerExactlyOnce)
{
    ProductionCrossingScene scene(100U);
    ASSERT_TRUE(scene.hasValidGroundPlane());
    std::vector<Map *> mapsBefore{};
    ASSERT_EQ((scene.atlas.getAllMaps(mapsBefore)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    std::vector<geometric::Plane *> knownWallsBefore{};
    ASSERT_EQ((scene.knownRoom.getWalls(knownWallsBefore)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    std::vector<geometric::Plane *> farWallsBefore{};
    ASSERT_EQ((scene.farRoom.getWalls(farWallsBefore)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    scene.confirmFirstRoom(0.0);
    scene.produceTraversalEvidence();
    std::pair<bool, bool> getRoomTrackerPendingForTest2{};
    ASSERT_EQ((scene.manager.getRoomTrackerPendingForTest(
                  getRoomTrackerPendingForTest2)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ(getRoomTrackerPendingForTest2, std::make_pair(true, false));
    ASSERT_EQ((scene.manager.processRoomTrackerPendingForTest(1.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);

    scene.addReturnCrossing();
    scene.produceTraversalEvidence();
    std::pair<bool, bool> getRoomTrackerPendingForTest3{};
    ASSERT_EQ((scene.manager.getRoomTrackerPendingForTest(
                  getRoomTrackerPendingForTest3)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ(getRoomTrackerPendingForTest3, std::make_pair(true, true));
    ASSERT_EQ((scene.manager.processRoomTrackerPendingForTest(3.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);

    semantic::RoomTrackingState getRoomTrackerStateForTest2{};
    ASSERT_EQ(
        (scene.manager.getRoomTrackerStateForTest(getRoomTrackerStateForTest2)),
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ(getRoomTrackerStateForTest2,
              semantic::RoomTrackingState::CROSSING_PASSAGE);
    std::size_t traversalKnownToFarCount{};
    ASSERT_EQ(
        (scene.passage.getTraversalKnownToFarCount(traversalKnownToFarCount)),
        vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(traversalKnownToFarCount, 1U);
    std::size_t traversalFarToKnownCount{};
    ASSERT_EQ(
        (scene.passage.getTraversalFarToKnownCount(traversalFarToKnownCount)),
        vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(traversalFarToKnownCount, 1U);
    std::size_t traversalObservationCount{};
    ASSERT_EQ(
        (scene.passage.getTraversalObservationCount(traversalObservationCount)),
        vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(traversalObservationCount, 2U);

    bool hasBidirectionalTraversalEvidence2{};
    ASSERT_EQ((scene.passage.hasBidirectionalTraversalEvidence(
                  hasBidirectionalTraversalEvidence2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_TRUE(hasBidirectionalTraversalEvidence2);
    scene.addThirdCrossing();
    scene.produceTraversalEvidence();
    std::pair<bool, bool> getRoomTrackerPendingForTest4{};
    ASSERT_EQ((scene.manager.getRoomTrackerPendingForTest(
                  getRoomTrackerPendingForTest4)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ(getRoomTrackerPendingForTest4, std::make_pair(true, true));
    ASSERT_EQ((scene.manager.submitVerificationVerdict(passVerdict())),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((scene.manager.processRoomTrackerPendingForTest(4.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((scene.manager.submitVerificationVerdict(passVerdict())),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((scene.manager.processRoomTrackerPendingForTest(6.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);

    semantic::RoomTrackingState getRoomTrackerStateForTest3{};
    ASSERT_EQ(
        (scene.manager.getRoomTrackerStateForTest(getRoomTrackerStateForTest3)),
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ(getRoomTrackerStateForTest3,
              semantic::RoomTrackingState::CONFIRMED_ROOM);
    const std::vector<semantic::TransitionEvent> *p_historyRef = nullptr;
    ASSERT_EQ((scene.manager.getRoomTrackerEventHistoryForTest(p_historyRef)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    const std::vector<semantic::TransitionEvent> &history = *p_historyRef;
    EXPECT_EQ(countAcceptedEvents(
                  history,
                  semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED),
              1U);
    EXPECT_EQ(countAcceptedEvents(
                  history,
                  semantic::RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE),
              1U);
    const std::size_t historySizeAfterTraversal = history.size();
    std::size_t       observationsAfterTraversal{};
    ASSERT_EQ((scene.passage.getTraversalObservationCount(
                  observationsAfterTraversal)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    for (double now_s = 7.0; now_s <= 10.0; now_s += 1.0)
    {
        scene.produceTraversalEvidence();
        ASSERT_EQ((scene.manager.processRoomTrackerPendingForTest(now_s)),
                  SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    }
    const std::vector<semantic::TransitionEvent>
        *p_getRoomTrackerEventHistoryForTest = nullptr;
    ASSERT_EQ((scene.manager.getRoomTrackerEventHistoryForTest(
                  p_getRoomTrackerEventHistoryForTest)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ((*p_getRoomTrackerEventHistoryForTest).size(),
              historySizeAfterTraversal);
    std::size_t traversalObservationCount2{};
    ASSERT_EQ((scene.passage.getTraversalObservationCount(
                  traversalObservationCount2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(traversalObservationCount2, observationsAfterTraversal);
    const std::vector<semantic::TransitionEvent>
        *p_getRoomTrackerEventHistoryForTest2 = nullptr;
    ASSERT_EQ((scene.manager.getRoomTrackerEventHistoryForTest(
                  p_getRoomTrackerEventHistoryForTest2)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ(countAcceptedEvents(
                  (*p_getRoomTrackerEventHistoryForTest2),
                  semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED),
              1U);
    const std::vector<semantic::TransitionEvent>
        *p_getRoomTrackerEventHistoryForTest3 = nullptr;
    ASSERT_EQ((scene.manager.getRoomTrackerEventHistoryForTest(
                  p_getRoomTrackerEventHistoryForTest3)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ(countAcceptedEvents(
                  (*p_getRoomTrackerEventHistoryForTest3),
                  semantic::RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE),
              1U);

    bool hasRoomTag2{};
    ASSERT_EQ((scene.knownRoom.hasRoomTag(hasRoomTag2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_FALSE(hasRoomTag2);
    bool hasRoomTag3{};
    ASSERT_EQ((scene.farRoom.hasRoomTag(hasRoomTag3)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_FALSE(hasRoomTag3);
    std::vector<vs_graphs::core::geometric::Plane *> walls{};
    ASSERT_EQ((scene.knownRoom.getWalls(walls)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(walls, knownWallsBefore);
    std::vector<vs_graphs::core::geometric::Plane *> walls2{};
    ASSERT_EQ((scene.farRoom.getWalls(walls2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(walls2, farWallsBefore);
    vs_graphs::core::semantic::Passage::KnownSideProvenance
        knownSideProvenance{};
    ASSERT_EQ((scene.passage.getKnownSideProvenance(knownSideProvenance)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(knownSideProvenance.p_room, &scene.knownRoom);
    vs_graphs::core::semantic::Room *p_prospectiveRoom = nullptr;
    ASSERT_EQ((scene.passage.getProspectiveRoom(p_prospectiveRoom)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(p_prospectiveRoom, &scene.farRoom);
    std::vector<Map *> allMaps{};
    ASSERT_EQ((scene.atlas.getAllMaps(allMaps)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    EXPECT_EQ(allMaps, mapsBefore);
    ASSERT_EQ(mapsBefore.size(), 2U);
    for (Map *p_map : mapsBefore)
    {
        bool isActiveMap2{};
        ASSERT_EQ((scene.atlas.isActiveMap(p_map, isActiveMap2)),
                  AtlasStatus::ATLAS_STATUS_SUCCESS);
        EXPECT_TRUE(isActiveMap2);
    }
}

TEST(RoomTrackerProductionIntegration,
     ProducerRateMismatchAndOneCycleFlickerFailClosed)
{
    ProductionCrossingScene scene(200U);
    ASSERT_TRUE(scene.hasValidGroundPlane());
    scene.confirmFirstRoom(100.0);

    for (unsigned int producerUpdate = 0U; producerUpdate < 5U;
         ++producerUpdate)
    {
        scene.produceTraversalEvidence();
    }
    std::pair<bool, bool> getRoomTrackerPendingForTest2{};
    ASSERT_EQ((scene.manager.getRoomTrackerPendingForTest(
                  getRoomTrackerPendingForTest2)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ(getRoomTrackerPendingForTest2, std::make_pair(true, false));
    std::size_t traversalObservationCount{};
    ASSERT_EQ(
        (scene.passage.getTraversalObservationCount(traversalObservationCount)),
        vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(traversalObservationCount, 1U);
    ASSERT_EQ((scene.manager.processRoomTrackerPendingForTest(101.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    std::pair<bool, bool> getRoomTrackerPendingForTest3{};
    ASSERT_EQ((scene.manager.getRoomTrackerPendingForTest(
                  getRoomTrackerPendingForTest3)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ(getRoomTrackerPendingForTest3, std::make_pair(false, false));

    /* One crossing cycle cannot satisfy dwell. A consumer-only cycle supplies
     * zero confidence and resets the production semantic::RoomTracker dwell. */
    ASSERT_EQ((scene.manager.processRoomTrackerPendingForTest(105.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    semantic::RoomTrackingState getRoomTrackerStateForTest2{};
    ASSERT_EQ(
        (scene.manager.getRoomTrackerStateForTest(getRoomTrackerStateForTest2)),
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ(getRoomTrackerStateForTest2,
              semantic::RoomTrackingState::CONFIRMED_ROOM);

    scene.addReturnCrossing();
    for (unsigned int producerUpdate = 0U; producerUpdate < 4U;
         ++producerUpdate)
    {
        scene.produceTraversalEvidence();
    }
    ASSERT_EQ((scene.manager.processRoomTrackerPendingForTest(106.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    semantic::RoomTrackingState getRoomTrackerStateForTest3{};
    ASSERT_EQ(
        (scene.manager.getRoomTrackerStateForTest(getRoomTrackerStateForTest3)),
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ(getRoomTrackerStateForTest3,
              semantic::RoomTrackingState::CONFIRMED_ROOM);
    std::size_t traversalObservationCount2{};
    ASSERT_EQ((scene.passage.getTraversalObservationCount(
                  traversalObservationCount2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(traversalObservationCount2, 2U);

    scene.addAlternatingCrossing(203U, true);
    scene.produceTraversalEvidence();
    ASSERT_EQ((scene.manager.processRoomTrackerPendingForTest(108.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    semantic::RoomTrackingState getRoomTrackerStateForTest4{};
    ASSERT_EQ(
        (scene.manager.getRoomTrackerStateForTest(getRoomTrackerStateForTest4)),
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ(getRoomTrackerStateForTest4,
              semantic::RoomTrackingState::CROSSING_PASSAGE);

    scene.addAlternatingCrossing(204U, false);
    scene.produceTraversalEvidence();
    ASSERT_EQ((scene.manager.submitVerificationVerdict(passVerdict())),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((scene.manager.processRoomTrackerPendingForTest(110.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);

    /* A one-cycle unavailable verdict is weak input and resets completion
     * dwell even though the real passage retains both-side evidence. */
    ASSERT_EQ((scene.manager.processRoomTrackerPendingForTest(111.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    semantic::RoomTrackingState getRoomTrackerStateForTest5{};
    ASSERT_EQ(
        (scene.manager.getRoomTrackerStateForTest(getRoomTrackerStateForTest5)),
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ(getRoomTrackerStateForTest5,
              semantic::RoomTrackingState::CROSSING_PASSAGE);

    scene.addAlternatingCrossing(205U, true);
    scene.produceTraversalEvidence();
    ASSERT_EQ((scene.manager.submitVerificationVerdict(passVerdict())),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((scene.manager.processRoomTrackerPendingForTest(112.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((scene.manager.submitVerificationVerdict(passVerdict())),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((scene.manager.processRoomTrackerPendingForTest(114.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    semantic::RoomTrackingState getRoomTrackerStateForTest6{};
    ASSERT_EQ(
        (scene.manager.getRoomTrackerStateForTest(getRoomTrackerStateForTest6)),
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ(getRoomTrackerStateForTest6,
              semantic::RoomTrackingState::CONFIRMED_ROOM);
    const std::vector<semantic::TransitionEvent> *p_historyRef = nullptr;
    ASSERT_EQ((scene.manager.getRoomTrackerEventHistoryForTest(p_historyRef)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    const std::vector<semantic::TransitionEvent> &history = *p_historyRef;
    EXPECT_EQ(countAcceptedEvents(
                  history,
                  semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED),
              1U);
    EXPECT_EQ(countAcceptedEvents(
                  history,
                  semantic::RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE),
              1U);
    std::size_t traversalObservationCount3{};
    ASSERT_EQ((scene.passage.getTraversalObservationCount(
                  traversalObservationCount3)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(traversalObservationCount3, 5U);
}

TEST(RoomTrackerProductionIntegration,
     ConcurrentCrossingAndBothSidesHandoffIsLossless)
{
    constexpr unsigned int repetitionCount           = 16U;
    constexpr unsigned int expectedAcceptedHandoffs  = repetitionCount * 2U;
    unsigned int           acceptedCrossingHandoffs  = 0U;
    unsigned int           acceptedBothSidesHandoffs = 0U;
    unsigned int           observedMutexContentions  = 0U;

    for (unsigned int repetition = 0U; repetition < repetitionCount;
         ++repetition)
    {
        ProductionCrossingScene scene(1000U + repetition * 10U);
        ASSERT_TRUE(scene.hasValidGroundPlane());
        scene.confirmFirstRoom(0.0);

        scene.produceTraversalEvidence();
        ASSERT_EQ((scene.manager.processRoomTrackerPendingForTest(1.0)),
                  SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
        scene.addReturnCrossing();
        if (overlapProductionPublishAndConsume(scene, 3.0))
        {
            ++observedMutexContentions;
        }
        semantic::RoomTrackingState getRoomTrackerStateForTest2{};
        ASSERT_EQ((scene.manager.getRoomTrackerStateForTest(
                      getRoomTrackerStateForTest2)),
                  SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
        if (getRoomTrackerStateForTest2 ==
            semantic::RoomTrackingState::CROSSING_PASSAGE)
        {
            ++acceptedCrossingHandoffs;
        }

        scene.addThirdCrossing();
        if (overlapProductionPublishAndConsume(scene, 6.0))
        {
            ++observedMutexContentions;
        }
        bool hasBidirectionalTraversalEvidence2{};
        ASSERT_EQ(
            (scene.passage.hasBidirectionalTraversalEvidence(
                hasBidirectionalTraversalEvidence2)),
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
        ASSERT_TRUE(hasBidirectionalTraversalEvidence2);
        ASSERT_EQ((scene.manager.submitVerificationVerdict(passVerdict())),
                  SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
        ASSERT_EQ((scene.manager.processRoomTrackerPendingForTest(8.0)),
                  SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
        ASSERT_EQ((scene.manager.submitVerificationVerdict(passVerdict())),
                  SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
        ASSERT_EQ((scene.manager.processRoomTrackerPendingForTest(10.0)),
                  SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
        semantic::RoomTrackingState getRoomTrackerStateForTest3{};
        ASSERT_EQ((scene.manager.getRoomTrackerStateForTest(
                      getRoomTrackerStateForTest3)),
                  SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
        if (getRoomTrackerStateForTest3 ==
            semantic::RoomTrackingState::CONFIRMED_ROOM)
        {
            ++acceptedBothSidesHandoffs;
        }

        const std::vector<semantic::TransitionEvent> *p_historyRef = nullptr;
        ASSERT_EQ(
            (scene.manager.getRoomTrackerEventHistoryForTest(p_historyRef)),
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
        const std::vector<semantic::TransitionEvent> &history = *p_historyRef;
        EXPECT_EQ(countAcceptedEvents(
                      history,
                      semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED),
                  1U);
        EXPECT_EQ(countAcceptedEvents(
                      history,
                      semantic::RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE),
                  1U);
        std::size_t traversalObservationCount{};
        ASSERT_EQ(
            (scene.passage.getTraversalObservationCount(
                traversalObservationCount)),
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
        EXPECT_EQ(traversalObservationCount, 3U);
    }

    EXPECT_EQ(acceptedCrossingHandoffs, repetitionCount);
    EXPECT_EQ(acceptedBothSidesHandoffs, repetitionCount);
    EXPECT_EQ(acceptedCrossingHandoffs + acceptedBothSidesHandoffs,
              expectedAcceptedHandoffs);
    EXPECT_EQ(observedMutexContentions, expectedAcceptedHandoffs);
}

TEST(RoomTrackerProductionIntegration, TraversalMarksReachedRoomVisited)
{
    Atlas atlas(0);
    bool  atlasWasEventPending{};
    ASSERT_EQ((atlas.consumeNewMapCreatedEvent(atlasWasEventPending)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    static_cast<void>(atlasWasEventPending);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    bool atlasWasEventPending2{};
    ASSERT_EQ((atlas.consumeNewMapCreatedEvent(atlasWasEventPending2)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    static_cast<void>(atlasWasEventPending2);
    SemanticsManager manager(&atlas);

    /* Ground plane: traversal evidence needs a valid ground normal. */
    geometric::Plane groundPlane;
    ASSERT_EQ((groundPlane.setId(0)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((groundPlane.setMap(p_map)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ(
        (groundPlane.setPlaneType(geometric::Plane::PlaneVariant::GROUND)),
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((groundPlane.setGlobalEquation(
                  g2o::Plane3D(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0)))),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr groundCloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (int xIndex = 0; xIndex < 5; ++xIndex)
    {
        for (int yIndex = 0; yIndex < 5; ++yIndex)
        {
            pcl::PointXYZRGBA point;
            point.x = static_cast<float>(xIndex) * 0.25F;
            point.y = static_cast<float>(yIndex) * 0.25F;
            point.z = 0.0F;
            groundCloud->push_back(point);
        }
    }
    ASSERT_EQ((groundPlane.setMapClouds(groundCloud)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    bool wasPlaneRefit{};
    ASSERT_EQ(
        (GeoSemHelpers::refitMappedPlaneFromCloud(&groundPlane, wasPlaneRefit)),
        GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS);
    ASSERT_TRUE(wasPlaneRefit);
    ASSERT_EQ((p_map->addMapPlane(&groundPlane)),
              MapStatus::MAP_STATUS_SUCCESS);

    /* Known room: entered earlier, so already visited. */
    semantic::Room knownRoom;
    ASSERT_EQ((knownRoom.setId(10)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((knownRoom.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((knownRoom.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((knownRoom.setCentroid(Eigen::Vector3d(-1.0, 0.0, 1.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((knownRoom.setPreviouslyVisited(true)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&knownRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    /* Prospective far-side room: observed but never entered. */
    semantic::Room farRoom;
    ASSERT_EQ((farRoom.setId(11)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((farRoom.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((farRoom.setRoomVariant(semantic::Room::RoomVariant::UNDEFINED)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((farRoom.setCentroid(Eigen::Vector3d(1.0, 0.0, 1.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&farRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    semantic::Passage passage;
    ASSERT_EQ((passage.setId(20)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setMap(p_map)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setPassable(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setWidth(2.0)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setHeight(2.0)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setCentroid(Eigen::Vector3d(0.0, 0.0, 1.0))),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setGlobalEquation(
                  g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)))),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setKnownSideRoom(&knownRoom)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setKnownSideDirection(Eigen::Vector3d(-1.0, 0.0, 0.0))),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setProspectiveRoom(&farRoom)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);

    /* Camera trajectory crossing the aperture from the known side. SetPose
     * takes world-to-camera, so the translation is the negated center. */
    KeyFrame nearKeyFrame;
    nearKeyFrame.id = 0U;
    ASSERT_EQ((nearKeyFrame.setPose(
                  Sophus::SE3f(Eigen::Matrix3f::Identity(),
                               Eigen::Vector3f(1.0F, 0.0F, -1.0F)))),
              KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addKeyFrame(&nearKeyFrame)),
              MapStatus::MAP_STATUS_SUCCESS);

    KeyFrame farKeyFrame;
    farKeyFrame.id = 1U;
    ASSERT_EQ((farKeyFrame.setPose(
                  Sophus::SE3f(Eigen::Matrix3f::Identity(),
                               Eigen::Vector3f(-1.0F, 0.0F, -1.0F)))),
              KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addKeyFrame(&farKeyFrame)),
              MapStatus::MAP_STATUS_SUCCESS);

    bool hasPreviouslyVisited2{};
    ASSERT_EQ((farRoom.hasPreviouslyVisited(hasPreviouslyVisited2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_FALSE(hasPreviouslyVisited2);

    ASSERT_EQ((manager.updateTraversalEvidence(&atlas)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);

    int getCurrentRoomId2{};
    ASSERT_EQ((manager.getCurrentRoomId(getCurrentRoomId2)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ(getCurrentRoomId2, 11);
    bool hasPreviouslyVisited3{};
    ASSERT_EQ((farRoom.hasPreviouslyVisited(hasPreviouslyVisited3)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_TRUE(hasPreviouslyVisited3);
    bool hasPreviouslyVisited4{};
    ASSERT_EQ((knownRoom.hasPreviouslyVisited(hasPreviouslyVisited4)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_TRUE(hasPreviouslyVisited4);
}

TEST(RoomTrackerProductionIntegration, SeedFallbackLeavesRoomUnvisited)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticsManager manager(&atlas);

    semantic::Room room;
    ASSERT_EQ((room.setId(3)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setCentroid(Eigen::Vector3d(0.0, 0.0, 1.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    ASSERT_EQ((manager.setCurrentRoomIdForTest(-1)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((manager.seedCurrentRoomFromActiveMapForTest(p_map)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);

    /* Fallback belief assigns the current room without entry evidence, so
     * the room must not be marked visited. */
    int getCurrentRoomId2{};
    ASSERT_EQ((manager.getCurrentRoomId(getCurrentRoomId2)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ(getCurrentRoomId2, 3);
    bool hasPreviouslyVisited2{};
    ASSERT_EQ((room.hasPreviouslyVisited(hasPreviouslyVisited2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_FALSE(hasPreviouslyVisited2);
}

} // namespace core
} // namespace vs_graphs

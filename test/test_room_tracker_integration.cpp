/**
 * Focused production seam tests for WP13 Phase 1 event delivery.
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
#include <thread>
#include <utility>
#include <vector>

namespace ORB_SLAM3
{
namespace
{
VerificationVerdict passVerdict()
{
    VerificationVerdict verdict;
    verdict.status      = VerificationStatus::PASS;
    verdict.pass        = true;
    verdict.inlierCount = 3U;
    verdict.inlierRatio = 1.0;
    verdict.confidence  = 1.0;
    return verdict;
}

VerificationVerdict rejectedVerdict()
{
    VerificationVerdict verdict;
    verdict.status = VerificationStatus::REJECTED;
    return verdict;
}

std::size_t countAcceptedEvents(const std::vector<TransitionEvent> &history_in,
                                RoomTrackingEvent                   event_in)
{
    return static_cast<std::size_t>(std::count_if(
        history_in.begin(),
        history_in.end(),
        [event_in](const TransitionEvent &record_in)
        { return record_in.accepted && record_in.event == event_in; }));
}

class ProductionCrossingScene
{
  public:
    explicit ProductionCrossingScene(unsigned long keyFrameIdBase_in) :
        atlas(0),
        p_map(atlas.GetCurrentMap()),
        manager(&atlas),
        returnKeyFrameAdded(false),
        thirdCrossingKeyFrameAdded(false),
        groundPlaneValid(false)
    {
        static_cast<void>(atlas.consumeNewMapCreatedEvent());
        atlas.CreateNewMap();
        p_map = atlas.GetCurrentMap();
        static_cast<void>(atlas.consumeNewMapCreatedEvent());

        groundPlane.setId(0);
        groundPlane.SetMap(p_map);
        groundPlane.setPlaneType(Plane::planeVariant::GROUND);
        groundPlane.setGlobalEquation(
            g2o::Plane3D(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0)));
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
        groundPlane.setMapClouds(groundCloud);
        groundPlaneValid =
            GeoSemHelpers::refitMappedPlaneFromCloud(&groundPlane);
        p_map->AddMapPlane(&groundPlane);

        knownWall.setId(1);
        knownWall.SetMap(p_map);
        knownWall.setPlaneType(Plane::planeVariant::WALL);
        farWall.setId(2);
        farWall.SetMap(p_map);
        farWall.setPlaneType(Plane::planeVariant::WALL);
        p_map->AddMapPlane(&knownWall);
        p_map->AddMapPlane(&farWall);

        knownRoom.setId(10);
        knownRoom.setMap(p_map);
        knownRoom.setRoomVariant(Room::roomVariant::ROOM);
        knownRoom.setWalls(&knownWall);
        farRoom.setId(11);
        farRoom.setMap(p_map);
        farRoom.setRoomVariant(Room::roomVariant::ROOM);
        farRoom.setWalls(&farWall);
        p_map->AddDetectedMapRoom(&knownRoom);
        p_map->AddDetectedMapRoom(&farRoom);

        passage.setId(20);
        passage.setMap(p_map);
        passage.setPassable(true);
        passage.setWidth(2.0);
        passage.setHeight(2.0);
        passage.setCentroid(Eigen::Vector3d(0.0, 0.0, 1.0));
        passage.setGlobalEquation(
            g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)));
        passage.setKnownSideRoom(&knownRoom);
        passage.setKnownSideDirection(Eigen::Vector3d(-1.0, 0.0, 0.0));
        passage.setProspectiveRoom(&farRoom);
        p_map->AddMapPassage(&passage);

        addKeyFrame(knownSideKeyFrame,
                    keyFrameIdBase_in,
                    Eigen::Vector3f(-1.0F, 0.0F, 1.0F));
        addKeyFrame(farSideKeyFrame,
                    keyFrameIdBase_in + 1U,
                    Eigen::Vector3f(1.0F, 0.0F, 1.0F));
        returnSideKeyFrame.mnId = keyFrameIdBase_in + 2U;
        returnSideKeyFrame.SetPose(
            Sophus::SE3f(Eigen::Matrix3f::Identity(),
                         Eigen::Vector3f(1.0F, 0.0F, -1.0F)));
        thirdCrossingKeyFrame.mnId = keyFrameIdBase_in + 3U;
        thirdCrossingKeyFrame.SetPose(
            Sophus::SE3f(Eigen::Matrix3f::Identity(),
                         Eigen::Vector3f(-1.0F, 0.0F, 1.0F)));
    }

    void confirmFirstRoom(double now_s_in)
    {
        manager.submitVerificationVerdict(passVerdict());
        manager.processRoomTrackerPendingForTest(now_s_in);
    }

    void produceTraversalEvidence()
    {
        manager.updateTraversalEvidence(&atlas);
    }

    void addReturnCrossing()
    {
        if (!returnKeyFrameAdded)
        {
            p_map->AddKeyFrame(&returnSideKeyFrame);
            returnKeyFrameAdded = true;
        }
    }

    void addThirdCrossing()
    {
        if (!thirdCrossingKeyFrameAdded)
        {
            p_map->AddKeyFrame(&thirdCrossingKeyFrame);
            thirdCrossingKeyFrameAdded = true;
        }
    }

    void addAlternatingCrossing(unsigned long keyFrameId_in, bool farSide_in)
    {
        std::unique_ptr<KeyFrame> p_keyFrame(new KeyFrame());
        p_keyFrame->mnId = keyFrameId_in;
        const Eigen::Vector3f cameraCenter_World_m =
            farSide_in ? Eigen::Vector3f(1.0F, 0.0F, 1.0F)
                       : Eigen::Vector3f(-1.0F, 0.0F, 1.0F);
        p_keyFrame->SetPose(
            Sophus::SE3f(Eigen::Matrix3f::Identity(), -cameraCenter_World_m));
        p_map->AddKeyFrame(p_keyFrame.get());
        extraCrossingKeyFrames.push_back(std::move(p_keyFrame));
    }

    bool hasValidGroundPlane() const
    {
        return groundPlaneValid;
    }

    Atlas                                  atlas;
    Map                                   *p_map;
    SemanticsManager                       manager;
    Plane                                  groundPlane;
    Plane                                  knownWall;
    Plane                                  farWall;
    Room                                   knownRoom;
    Room                                   farRoom;
    Passage                                passage;
    KeyFrame                               knownSideKeyFrame;
    KeyFrame                               farSideKeyFrame;
    KeyFrame                               returnSideKeyFrame;
    KeyFrame                               thirdCrossingKeyFrame;
    std::vector<std::unique_ptr<KeyFrame>> extraCrossingKeyFrames;

  private:
    void addKeyFrame(KeyFrame              &keyFrame_inout,
                     unsigned long          keyFrameId_in,
                     const Eigen::Vector3f &cameraCenter_World_m_in)
    {
        keyFrame_inout.mnId = keyFrameId_in;
        keyFrame_inout.SetPose(Sophus::SE3f(Eigen::Matrix3f::Identity(),
                                            -cameraCenter_World_m_in));
        p_map->AddKeyFrame(&keyFrame_inout);
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

    scene_inout.manager.setRoomTrackerPendingPublishHookForTest(
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
        });

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
            consumerObservedContention =
                !scene_inout.manager.tryLockRoomTrackerPendingMutexForTest();
            {
                std::lock_guard<std::mutex> synchronizationLock(
                    synchronizationMutex);
                consumerHasStarted = true;
            }
            synchronizationCondition.notify_all();
            scene_inout.manager.processRoomTrackerPendingForTest(now_s_in);
        });

    producer.join();
    consumer.join();
    return consumerObservedContention;
}
} // namespace

TEST(RoomTrackerProductionIntegration, LossIsOneEventPerRecoveredEpisode)
{
    Atlas atlas(0);
    EXPECT_TRUE(atlas.consumeNewMapCreatedEvent());
    SemanticsManager manager(&atlas);

    manager.submitVerificationVerdict(passVerdict());
    manager.processRoomTrackerPendingForTest(0.0);

    std::thread firstLoss([&manager]() { manager.onTrackingLost(); });
    std::thread secondLoss([&manager]() { manager.onTrackingLost(); });
    firstLoss.join();
    secondLoss.join();
    manager.processRoomTrackerPendingForTest(1.0);

    const std::vector<TransitionEvent> &history =
        manager.getRoomTrackerEventHistoryForTest();
    ASSERT_EQ(history.size(), 2U);
    EXPECT_EQ(history.back().event, RoomTrackingEvent::TRACKING_LOST);
    EXPECT_EQ(manager.getLastKnownRoomId(), -1);

    manager.onTrackingRecovered();
    atlas.CreateNewMap();
    manager.submitVerificationVerdict(passVerdict());
    manager.processRoomTrackerPendingForTest(2.0);
    manager.submitVerificationVerdict(passVerdict());
    manager.processRoomTrackerPendingForTest(3.0);
    manager.onTrackingLost();
    manager.processRoomTrackerPendingForTest(4.0);
    EXPECT_EQ(manager.getRoomTrackerEventHistoryForTest().size(), 5U);
}

TEST(RoomTrackerProductionIntegration,
     NewMapEventSurvivesUnavailableAndRejectedVerdicts)
{
    Atlas atlas(0);
    EXPECT_TRUE(atlas.consumeNewMapCreatedEvent());
    SemanticsManager manager(&atlas);

    manager.submitVerificationVerdict(passVerdict());
    manager.processRoomTrackerPendingForTest(0.0);
    manager.onTrackingLost();
    manager.processRoomTrackerPendingForTest(1.0);

    atlas.CreateNewMap();
    manager.processRoomTrackerPendingForTest(2.0);
    EXPECT_EQ(manager.getRoomTrackerEventHistoryForTest().size(), 2U);

    manager.submitVerificationVerdict(rejectedVerdict());
    manager.processRoomTrackerPendingForTest(3.0);
    EXPECT_EQ(manager.getRoomTrackerEventHistoryForTest().size(), 2U);

    manager.submitVerificationVerdict(passVerdict());
    manager.processRoomTrackerPendingForTest(4.0);
    ASSERT_EQ(manager.getRoomTrackerEventHistoryForTest().size(), 3U);
    EXPECT_EQ(manager.getRoomTrackerEventHistoryForTest().back().event,
              RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH);

    manager.submitVerificationVerdict(passVerdict());
    manager.processRoomTrackerPendingForTest(5.0);
    ASSERT_EQ(manager.getRoomTrackerEventHistoryForTest().size(), 4U);
    EXPECT_EQ(manager.getRoomTrackerEventHistoryForTest().back().event,
              RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM);
}

TEST(RoomTrackerProductionIntegration, AtlasMapEventIsConsumedExactlyOnce)
{
    Atlas atlas(0);
    EXPECT_TRUE(atlas.consumeNewMapCreatedEvent());
    atlas.CreateNewMap();

    std::atomic<unsigned int> deliveries{0U};
    std::vector<std::thread>  consumers;
    for (unsigned int consumerIndex = 0U; consumerIndex < 8U; ++consumerIndex)
    {
        consumers.emplace_back(
            [&atlas, &deliveries]()
            {
                if (atlas.consumeNewMapCreatedEvent())
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
    EXPECT_FALSE(atlas.consumeNewMapCreatedEvent());
}

TEST(RoomTrackerProductionIntegration,
     ProductionTraversalEvidenceReachesTrackerExactlyOnce)
{
    ProductionCrossingScene scene(100U);
    ASSERT_TRUE(scene.hasValidGroundPlane());
    const std::vector<Map *>   mapsBefore       = scene.atlas.GetAllMaps();
    const std::vector<Plane *> knownWallsBefore = scene.knownRoom.getWalls();
    const std::vector<Plane *> farWallsBefore   = scene.farRoom.getWalls();

    scene.confirmFirstRoom(0.0);
    scene.produceTraversalEvidence();
    EXPECT_EQ(scene.manager.getRoomTrackerPendingForTest(),
              std::make_pair(true, false));
    scene.manager.processRoomTrackerPendingForTest(1.0);

    scene.addReturnCrossing();
    scene.produceTraversalEvidence();
    EXPECT_EQ(scene.manager.getRoomTrackerPendingForTest(),
              std::make_pair(true, true));
    scene.manager.processRoomTrackerPendingForTest(3.0);

    ASSERT_EQ(scene.manager.getRoomTrackerStateForTest(),
              RoomTrackingState::CROSSING_PASSAGE);
    EXPECT_EQ(scene.passage.getTraversalKnownToFarCount(), 1U);
    EXPECT_EQ(scene.passage.getTraversalFarToKnownCount(), 1U);
    EXPECT_EQ(scene.passage.getTraversalObservationCount(), 2U);

    EXPECT_TRUE(scene.passage.hasBidirectionalTraversalEvidence());
    scene.addThirdCrossing();
    scene.produceTraversalEvidence();
    EXPECT_EQ(scene.manager.getRoomTrackerPendingForTest(),
              std::make_pair(true, true));
    scene.manager.submitVerificationVerdict(passVerdict());
    scene.manager.processRoomTrackerPendingForTest(4.0);
    scene.manager.submitVerificationVerdict(passVerdict());
    scene.manager.processRoomTrackerPendingForTest(6.0);

    ASSERT_EQ(scene.manager.getRoomTrackerStateForTest(),
              RoomTrackingState::CONFIRMED_ROOM);
    const std::vector<TransitionEvent> &history =
        scene.manager.getRoomTrackerEventHistoryForTest();
    EXPECT_EQ(countAcceptedEvents(history,
                                  RoomTrackingEvent::PASSAGE_CROSSING_DETECTED),
              1U);
    EXPECT_EQ(
        countAcceptedEvents(history,
                            RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE),
        1U);
    const std::size_t historySizeAfterTraversal = history.size();
    const std::size_t observationsAfterTraversal =
        scene.passage.getTraversalObservationCount();
    for (double now_s = 7.0; now_s <= 10.0; now_s += 1.0)
    {
        scene.produceTraversalEvidence();
        scene.manager.processRoomTrackerPendingForTest(now_s);
    }
    EXPECT_EQ(scene.manager.getRoomTrackerEventHistoryForTest().size(),
              historySizeAfterTraversal);
    EXPECT_EQ(scene.passage.getTraversalObservationCount(),
              observationsAfterTraversal);
    EXPECT_EQ(
        countAcceptedEvents(scene.manager.getRoomTrackerEventHistoryForTest(),
                            RoomTrackingEvent::PASSAGE_CROSSING_DETECTED),
        1U);
    EXPECT_EQ(
        countAcceptedEvents(scene.manager.getRoomTrackerEventHistoryForTest(),
                            RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE),
        1U);

    EXPECT_FALSE(scene.knownRoom.hasRoomTag());
    EXPECT_FALSE(scene.farRoom.hasRoomTag());
    EXPECT_EQ(scene.knownRoom.getWalls(), knownWallsBefore);
    EXPECT_EQ(scene.farRoom.getWalls(), farWallsBefore);
    EXPECT_EQ(scene.passage.getKnownSideProvenance().pRoom, &scene.knownRoom);
    EXPECT_EQ(scene.passage.getProspectiveRoom(), &scene.farRoom);
    EXPECT_EQ(scene.atlas.GetAllMaps(), mapsBefore);
    ASSERT_EQ(mapsBefore.size(), 2U);
    for (Map *p_map : mapsBefore)
    {
        EXPECT_TRUE(scene.atlas.isActiveMap(p_map));
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
    EXPECT_EQ(scene.manager.getRoomTrackerPendingForTest(),
              std::make_pair(true, false));
    EXPECT_EQ(scene.passage.getTraversalObservationCount(), 1U);
    scene.manager.processRoomTrackerPendingForTest(101.0);
    EXPECT_EQ(scene.manager.getRoomTrackerPendingForTest(),
              std::make_pair(false, false));

    /* One crossing cycle cannot satisfy dwell. A consumer-only cycle supplies
     * zero confidence and resets the production RoomTracker dwell. */
    scene.manager.processRoomTrackerPendingForTest(105.0);
    EXPECT_EQ(scene.manager.getRoomTrackerStateForTest(),
              RoomTrackingState::CONFIRMED_ROOM);

    scene.addReturnCrossing();
    for (unsigned int producerUpdate = 0U; producerUpdate < 4U;
         ++producerUpdate)
    {
        scene.produceTraversalEvidence();
    }
    scene.manager.processRoomTrackerPendingForTest(106.0);
    EXPECT_EQ(scene.manager.getRoomTrackerStateForTest(),
              RoomTrackingState::CONFIRMED_ROOM);
    EXPECT_EQ(scene.passage.getTraversalObservationCount(), 2U);

    scene.addAlternatingCrossing(203U, true);
    scene.produceTraversalEvidence();
    scene.manager.processRoomTrackerPendingForTest(108.0);
    ASSERT_EQ(scene.manager.getRoomTrackerStateForTest(),
              RoomTrackingState::CROSSING_PASSAGE);

    scene.addAlternatingCrossing(204U, false);
    scene.produceTraversalEvidence();
    scene.manager.submitVerificationVerdict(passVerdict());
    scene.manager.processRoomTrackerPendingForTest(110.0);

    /* A one-cycle unavailable verdict is weak input and resets completion
     * dwell even though the real passage retains both-side evidence. */
    scene.manager.processRoomTrackerPendingForTest(111.0);
    EXPECT_EQ(scene.manager.getRoomTrackerStateForTest(),
              RoomTrackingState::CROSSING_PASSAGE);

    scene.addAlternatingCrossing(205U, true);
    scene.produceTraversalEvidence();
    scene.manager.submitVerificationVerdict(passVerdict());
    scene.manager.processRoomTrackerPendingForTest(112.0);
    scene.manager.submitVerificationVerdict(passVerdict());
    scene.manager.processRoomTrackerPendingForTest(114.0);
    ASSERT_EQ(scene.manager.getRoomTrackerStateForTest(),
              RoomTrackingState::CONFIRMED_ROOM);
    const std::vector<TransitionEvent> &history =
        scene.manager.getRoomTrackerEventHistoryForTest();
    EXPECT_EQ(countAcceptedEvents(history,
                                  RoomTrackingEvent::PASSAGE_CROSSING_DETECTED),
              1U);
    EXPECT_EQ(
        countAcceptedEvents(history,
                            RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE),
        1U);
    EXPECT_EQ(scene.passage.getTraversalObservationCount(), 5U);
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
        scene.manager.processRoomTrackerPendingForTest(1.0);
        scene.addReturnCrossing();
        if (overlapProductionPublishAndConsume(scene, 3.0))
        {
            ++observedMutexContentions;
        }
        if (scene.manager.getRoomTrackerStateForTest() ==
            RoomTrackingState::CROSSING_PASSAGE)
        {
            ++acceptedCrossingHandoffs;
        }

        scene.addThirdCrossing();
        if (overlapProductionPublishAndConsume(scene, 6.0))
        {
            ++observedMutexContentions;
        }
        ASSERT_TRUE(scene.passage.hasBidirectionalTraversalEvidence());
        scene.manager.submitVerificationVerdict(passVerdict());
        scene.manager.processRoomTrackerPendingForTest(8.0);
        scene.manager.submitVerificationVerdict(passVerdict());
        scene.manager.processRoomTrackerPendingForTest(10.0);
        if (scene.manager.getRoomTrackerStateForTest() ==
            RoomTrackingState::CONFIRMED_ROOM)
        {
            ++acceptedBothSidesHandoffs;
        }

        const std::vector<TransitionEvent> &history =
            scene.manager.getRoomTrackerEventHistoryForTest();
        EXPECT_EQ(
            countAcceptedEvents(history,
                                RoomTrackingEvent::PASSAGE_CROSSING_DETECTED),
            1U);
        EXPECT_EQ(
            countAcceptedEvents(history,
                                RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE),
            1U);
        EXPECT_EQ(scene.passage.getTraversalObservationCount(), 3U);
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
    static_cast<void>(atlas.consumeNewMapCreatedEvent());
    atlas.CreateNewMap();
    Map *p_map = atlas.GetCurrentMap();
    static_cast<void>(atlas.consumeNewMapCreatedEvent());
    SemanticsManager manager(&atlas);

    /* Ground plane: traversal evidence needs a valid ground normal. */
    Plane groundPlane;
    groundPlane.setId(0);
    groundPlane.SetMap(p_map);
    groundPlane.setPlaneType(Plane::planeVariant::GROUND);
    groundPlane.setGlobalEquation(
        g2o::Plane3D(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0)));
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
    groundPlane.setMapClouds(groundCloud);
    ASSERT_TRUE(GeoSemHelpers::refitMappedPlaneFromCloud(&groundPlane));
    p_map->AddMapPlane(&groundPlane);

    /* Known room: entered earlier, so already visited. */
    Room knownRoom;
    knownRoom.setId(10);
    knownRoom.setMap(p_map);
    knownRoom.setRoomVariant(Room::roomVariant::ROOM);
    knownRoom.setCentroid(Eigen::Vector3d(-1.0, 0.0, 1.0));
    knownRoom.setPreviouslyVisited(true);
    p_map->AddDetectedMapRoom(&knownRoom);

    /* Prospective far-side room: observed but never entered. */
    Room farRoom;
    farRoom.setId(11);
    farRoom.setMap(p_map);
    farRoom.setRoomVariant(Room::roomVariant::UNDEFINED);
    farRoom.setCentroid(Eigen::Vector3d(1.0, 0.0, 1.0));
    p_map->AddDetectedMapRoom(&farRoom);

    Passage passage;
    passage.setId(20);
    passage.setMap(p_map);
    passage.setPassable(true);
    passage.setWidth(2.0);
    passage.setHeight(2.0);
    passage.setCentroid(Eigen::Vector3d(0.0, 0.0, 1.0));
    passage.setGlobalEquation(
        g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)));
    passage.setKnownSideRoom(&knownRoom);
    passage.setKnownSideDirection(Eigen::Vector3d(-1.0, 0.0, 0.0));
    passage.setProspectiveRoom(&farRoom);
    p_map->AddMapPassage(&passage);

    /* Camera trajectory crossing the aperture from the known side. SetPose
     * takes world-to-camera, so the translation is the negated center. */
    KeyFrame nearKeyFrame;
    nearKeyFrame.mnId = 0U;
    nearKeyFrame.SetPose(Sophus::SE3f(Eigen::Matrix3f::Identity(),
                                      Eigen::Vector3f(1.0F, 0.0F, -1.0F)));
    p_map->AddKeyFrame(&nearKeyFrame);

    KeyFrame farKeyFrame;
    farKeyFrame.mnId = 1U;
    farKeyFrame.SetPose(Sophus::SE3f(Eigen::Matrix3f::Identity(),
                                     Eigen::Vector3f(-1.0F, 0.0F, -1.0F)));
    p_map->AddKeyFrame(&farKeyFrame);

    ASSERT_FALSE(farRoom.hasPreviouslyVisited());

    manager.updateTraversalEvidence(&atlas);

    EXPECT_EQ(manager.getCurrentRoomId(), 11);
    EXPECT_TRUE(farRoom.hasPreviouslyVisited());
    EXPECT_TRUE(knownRoom.hasPreviouslyVisited());
}

TEST(RoomTrackerProductionIntegration, SeedFallbackLeavesRoomUnvisited)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.GetCurrentMap();
    SemanticsManager manager(&atlas);

    Room room;
    room.setId(3);
    room.setMap(p_map);
    room.setRoomVariant(Room::roomVariant::ROOM);
    room.setCentroid(Eigen::Vector3d(0.0, 0.0, 1.0));
    p_map->AddDetectedMapRoom(&room);

    manager.setCurrentRoomIdForTest(-1);
    manager.seedCurrentRoomFromActiveMapForTest(p_map);

    /* Fallback belief assigns the current room without entry evidence, so
     * the room must not be marked visited. */
    EXPECT_EQ(manager.getCurrentRoomId(), 3);
    EXPECT_FALSE(room.hasPreviouslyVisited());
}

} // namespace ORB_SLAM3

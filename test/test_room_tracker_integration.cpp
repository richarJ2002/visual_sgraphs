/*!
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
        p_map(atlas.getCurrentMap()),
        manager(&atlas),
        returnKeyFrameAdded(false),
        thirdCrossingKeyFrameAdded(false),
        groundPlaneValid(false)
    {
        static_cast<void>(atlas.consumeNewMapCreatedEvent());
        atlas.createNewMap();
        p_map = atlas.getCurrentMap();
        static_cast<void>(atlas.consumeNewMapCreatedEvent());

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
        p_map->addMapPlane(&groundPlane);

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
        p_map->addMapPlane(&knownWall);
        p_map->addMapPlane(&farWall);

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
        p_map->addDetectedMapRoom(&knownRoom);
        p_map->addDetectedMapRoom(&farRoom);

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
        p_map->addMapPassage(&passage);

        addKeyFrame(knownSideKeyFrame,
                    keyFrameIdBase_in,
                    Eigen::Vector3f(-1.0F, 0.0F, 1.0F));
        addKeyFrame(farSideKeyFrame,
                    keyFrameIdBase_in + 1U,
                    Eigen::Vector3f(1.0F, 0.0F, 1.0F));
        returnSideKeyFrame.id = keyFrameIdBase_in + 2U;
        returnSideKeyFrame.setPose(
            Sophus::SE3f(Eigen::Matrix3f::Identity(),
                         Eigen::Vector3f(1.0F, 0.0F, -1.0F)));
        thirdCrossingKeyFrame.id = keyFrameIdBase_in + 3U;
        thirdCrossingKeyFrame.setPose(
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
            p_map->addKeyFrame(&returnSideKeyFrame);
            returnKeyFrameAdded = true;
        }
    }

    void addThirdCrossing()
    {
        if (!thirdCrossingKeyFrameAdded)
        {
            p_map->addKeyFrame(&thirdCrossingKeyFrame);
            thirdCrossingKeyFrameAdded = true;
        }
    }

    void addAlternatingCrossing(unsigned long keyFrameId_in, bool farSide_in)
    {
        std::unique_ptr<KeyFrame> p_keyFrame(new KeyFrame());
        p_keyFrame->id = keyFrameId_in;
        const Eigen::Vector3f cameraCenter_World_m =
            farSide_in ? Eigen::Vector3f(1.0F, 0.0F, 1.0F)
                       : Eigen::Vector3f(-1.0F, 0.0F, 1.0F);
        p_keyFrame->setPose(
            Sophus::SE3f(Eigen::Matrix3f::Identity(), -cameraCenter_World_m));
        p_map->addKeyFrame(p_keyFrame.get());
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
                     const Eigen::Vector3f &cameraCenter_World_m_in)
    {
        keyFrame_inout.id = keyFrameId_in;
        keyFrame_inout.setPose(Sophus::SE3f(Eigen::Matrix3f::Identity(),
                                            -cameraCenter_World_m_in));
        p_map->addKeyFrame(&keyFrame_inout);
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

    const std::vector<semantic::TransitionEvent> &history =
        manager.getRoomTrackerEventHistoryForTest();
    ASSERT_EQ(history.size(), 2U);
    EXPECT_EQ(history.back().event, semantic::RoomTrackingEvent::TRACKING_LOST);
    EXPECT_EQ(manager.getLastKnownRoomId(), -1);

    manager.onTrackingRecovered();
    atlas.createNewMap();
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

    atlas.createNewMap();
    manager.processRoomTrackerPendingForTest(2.0);
    EXPECT_EQ(manager.getRoomTrackerEventHistoryForTest().size(), 2U);

    manager.submitVerificationVerdict(rejectedVerdict());
    manager.processRoomTrackerPendingForTest(3.0);
    EXPECT_EQ(manager.getRoomTrackerEventHistoryForTest().size(), 2U);

    manager.submitVerificationVerdict(passVerdict());
    manager.processRoomTrackerPendingForTest(4.0);
    ASSERT_EQ(manager.getRoomTrackerEventHistoryForTest().size(), 3U);
    EXPECT_EQ(manager.getRoomTrackerEventHistoryForTest().back().event,
              semantic::RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH);

    manager.submitVerificationVerdict(passVerdict());
    manager.processRoomTrackerPendingForTest(5.0);
    ASSERT_EQ(manager.getRoomTrackerEventHistoryForTest().size(), 4U);
    EXPECT_EQ(manager.getRoomTrackerEventHistoryForTest().back().event,
              semantic::RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM);
}

TEST(RoomTrackerProductionIntegration, AtlasMapEventIsConsumedExactlyOnce)
{
    Atlas atlas(0);
    EXPECT_TRUE(atlas.consumeNewMapCreatedEvent());
    atlas.createNewMap();

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
    const std::vector<Map *>        mapsBefore = scene.atlas.getAllMaps();
    std::vector<geometric::Plane *> knownWallsBefore{};
    ASSERT_EQ((scene.knownRoom.getWalls(knownWallsBefore)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    std::vector<geometric::Plane *> farWallsBefore{};
    ASSERT_EQ((scene.farRoom.getWalls(farWallsBefore)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

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
    EXPECT_EQ(scene.manager.getRoomTrackerPendingForTest(),
              std::make_pair(true, true));
    scene.manager.submitVerificationVerdict(passVerdict());
    scene.manager.processRoomTrackerPendingForTest(4.0);
    scene.manager.submitVerificationVerdict(passVerdict());
    scene.manager.processRoomTrackerPendingForTest(6.0);

    ASSERT_EQ(scene.manager.getRoomTrackerStateForTest(),
              semantic::RoomTrackingState::CONFIRMED_ROOM);
    const std::vector<semantic::TransitionEvent> &history =
        scene.manager.getRoomTrackerEventHistoryForTest();
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
        scene.manager.processRoomTrackerPendingForTest(now_s);
    }
    EXPECT_EQ(scene.manager.getRoomTrackerEventHistoryForTest().size(),
              historySizeAfterTraversal);
    std::size_t traversalObservationCount2{};
    ASSERT_EQ((scene.passage.getTraversalObservationCount(
                  traversalObservationCount2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(traversalObservationCount2, observationsAfterTraversal);
    EXPECT_EQ(countAcceptedEvents(
                  scene.manager.getRoomTrackerEventHistoryForTest(),
                  semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED),
              1U);
    EXPECT_EQ(countAcceptedEvents(
                  scene.manager.getRoomTrackerEventHistoryForTest(),
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
    EXPECT_EQ(scene.atlas.getAllMaps(), mapsBefore);
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
    std::size_t traversalObservationCount{};
    ASSERT_EQ(
        (scene.passage.getTraversalObservationCount(traversalObservationCount)),
        vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(traversalObservationCount, 1U);
    scene.manager.processRoomTrackerPendingForTest(101.0);
    EXPECT_EQ(scene.manager.getRoomTrackerPendingForTest(),
              std::make_pair(false, false));

    /* One crossing cycle cannot satisfy dwell. A consumer-only cycle supplies
     * zero confidence and resets the production semantic::RoomTracker dwell. */
    scene.manager.processRoomTrackerPendingForTest(105.0);
    EXPECT_EQ(scene.manager.getRoomTrackerStateForTest(),
              semantic::RoomTrackingState::CONFIRMED_ROOM);

    scene.addReturnCrossing();
    for (unsigned int producerUpdate = 0U; producerUpdate < 4U;
         ++producerUpdate)
    {
        scene.produceTraversalEvidence();
    }
    scene.manager.processRoomTrackerPendingForTest(106.0);
    EXPECT_EQ(scene.manager.getRoomTrackerStateForTest(),
              semantic::RoomTrackingState::CONFIRMED_ROOM);
    std::size_t traversalObservationCount2{};
    ASSERT_EQ((scene.passage.getTraversalObservationCount(
                  traversalObservationCount2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(traversalObservationCount2, 2U);

    scene.addAlternatingCrossing(203U, true);
    scene.produceTraversalEvidence();
    scene.manager.processRoomTrackerPendingForTest(108.0);
    ASSERT_EQ(scene.manager.getRoomTrackerStateForTest(),
              semantic::RoomTrackingState::CROSSING_PASSAGE);

    scene.addAlternatingCrossing(204U, false);
    scene.produceTraversalEvidence();
    scene.manager.submitVerificationVerdict(passVerdict());
    scene.manager.processRoomTrackerPendingForTest(110.0);

    /* A one-cycle unavailable verdict is weak input and resets completion
     * dwell even though the real passage retains both-side evidence. */
    scene.manager.processRoomTrackerPendingForTest(111.0);
    EXPECT_EQ(scene.manager.getRoomTrackerStateForTest(),
              semantic::RoomTrackingState::CROSSING_PASSAGE);

    scene.addAlternatingCrossing(205U, true);
    scene.produceTraversalEvidence();
    scene.manager.submitVerificationVerdict(passVerdict());
    scene.manager.processRoomTrackerPendingForTest(112.0);
    scene.manager.submitVerificationVerdict(passVerdict());
    scene.manager.processRoomTrackerPendingForTest(114.0);
    ASSERT_EQ(scene.manager.getRoomTrackerStateForTest(),
              semantic::RoomTrackingState::CONFIRMED_ROOM);
    const std::vector<semantic::TransitionEvent> &history =
        scene.manager.getRoomTrackerEventHistoryForTest();
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
        scene.manager.processRoomTrackerPendingForTest(1.0);
        scene.addReturnCrossing();
        if (overlapProductionPublishAndConsume(scene, 3.0))
        {
            ++observedMutexContentions;
        }
        if (scene.manager.getRoomTrackerStateForTest() ==
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
        scene.manager.submitVerificationVerdict(passVerdict());
        scene.manager.processRoomTrackerPendingForTest(8.0);
        scene.manager.submitVerificationVerdict(passVerdict());
        scene.manager.processRoomTrackerPendingForTest(10.0);
        if (scene.manager.getRoomTrackerStateForTest() ==
            semantic::RoomTrackingState::CONFIRMED_ROOM)
        {
            ++acceptedBothSidesHandoffs;
        }

        const std::vector<semantic::TransitionEvent> &history =
            scene.manager.getRoomTrackerEventHistoryForTest();
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
    static_cast<void>(atlas.consumeNewMapCreatedEvent());
    atlas.createNewMap();
    Map *p_map = atlas.getCurrentMap();
    static_cast<void>(atlas.consumeNewMapCreatedEvent());
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
    p_map->addMapPlane(&groundPlane);

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
    p_map->addDetectedMapRoom(&knownRoom);

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
    p_map->addDetectedMapRoom(&farRoom);

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
    p_map->addMapPassage(&passage);

    /* Camera trajectory crossing the aperture from the known side. SetPose
     * takes world-to-camera, so the translation is the negated center. */
    KeyFrame nearKeyFrame;
    nearKeyFrame.id = 0U;
    nearKeyFrame.setPose(Sophus::SE3f(Eigen::Matrix3f::Identity(),
                                      Eigen::Vector3f(1.0F, 0.0F, -1.0F)));
    p_map->addKeyFrame(&nearKeyFrame);

    KeyFrame farKeyFrame;
    farKeyFrame.id = 1U;
    farKeyFrame.setPose(Sophus::SE3f(Eigen::Matrix3f::Identity(),
                                     Eigen::Vector3f(-1.0F, 0.0F, -1.0F)));
    p_map->addKeyFrame(&farKeyFrame);

    bool hasPreviouslyVisited2{};
    ASSERT_EQ((farRoom.hasPreviouslyVisited(hasPreviouslyVisited2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_FALSE(hasPreviouslyVisited2);

    manager.updateTraversalEvidence(&atlas);

    EXPECT_EQ(manager.getCurrentRoomId(), 11);
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
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
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
    p_map->addDetectedMapRoom(&room);

    manager.setCurrentRoomIdForTest(-1);
    manager.seedCurrentRoomFromActiveMapForTest(p_map);

    /* Fallback belief assigns the current room without entry evidence, so
     * the room must not be marked visited. */
    EXPECT_EQ(manager.getCurrentRoomId(), 3);
    bool hasPreviouslyVisited2{};
    ASSERT_EQ((room.hasPreviouslyVisited(hasPreviouslyVisited2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_FALSE(hasPreviouslyVisited2);
}

} // namespace core
} // namespace vs_graphs

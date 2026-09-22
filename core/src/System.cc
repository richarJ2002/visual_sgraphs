/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file         System.cc
 *
 * @brief        Implements System declared in System.h.
 */

#include "Utils/Converter/objects/Converter.h"
#include "ResetCause.h"
#include "System.h"
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <iomanip>
#include <memory>
#include <openssl/evp.h>
#include <pangolin/pangolin.h>
#include <thread>

namespace vs_graphs
{
namespace core
{

Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

System::System(const string         &strVocFile,
               const string         &strSettingsFile,
               const string         &strSysParamsFile,
               const eSensor         sensor,
               const bool            bUseViewer,
               const int             initFr,
               const string         &strSequence,
               const Verbose::eLevel verboseLevel) :
    sensor(sensor),
    p_viewer(static_cast<Viewer *>(nullptr)),
    mptGeometricSegmentation(static_cast<std::thread *>(nullptr)),
    resetRequested(false),
    resetActiveMapRequested(false),
    activateLocalizationModeRequested(false),
    deactivateLocalizationModeRequested(false),
    shutdownRequested(false)
{
    /* Output welcome message */
    std::cout << std::endl
              << "------------------------------------------------------"
              << std::endl
              << "🚀 Visual S-Graphs (vS-Graphs) Copyright © 2023-2025 by A. "
                 "Tourani, S. Ejaz, H. Bavle, J.L. Sanchez-Lopez, and H. Voos, "
                 "SnT - University of Luxembourg."
              << std::endl
              << "✨ Based on ORB-SLAM3 Copyright © 2017-2023 by C. Campos, R. "
                 "Elvira, J.J. Gómez, J.M.M. Montiel, and J.D. Tardós, "
                 "University of Zaragoza."
              << std::endl
              << "To redistribute the software please see LICENSE.txt."
              << std::endl
              << "------------------------------------------------------"
              << std::endl
              << std::endl;

    /* Output msg of what sensor is being used */
    std::cout << "[System] Input sensor is set to: ";
    if (sensor == MONOCULAR)
    {
        std::cout << "Monocular" << std::endl;
    }
    else if (sensor == STEREO)
    {
        std::cout << "Stereo" << std::endl;
    }
    else if (sensor == RGBD)
    {
        std::cout << "RGB-D" << std::endl;
    }
    else if (sensor == IMU_MONOCULAR)
    {
        std::cout << "Monocular-Inertial" << std::endl;
    }
    else if (sensor == IMU_STEREO)
    {
        std::cout << "Stereo-Inertial" << std::endl;
    }
    else if (sensor == IMU_RGBD)
    {
        std::cout << "RGB-D-Inertial" << std::endl;
    }

    /* Check settings file can be opened */
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "[System] Failed to open settings file at '"
                  << strSettingsFile << "'! Exiting ..." << std::endl;
        exit(-1);
    }

    cv::FileNode node = fsSettings["File.version"];
    if (!node.empty() && node.isString() && node.string() == "1.0")
    {
        settings_     = new utils::settings::Settings(strSettingsFile, sensor);
        loadAtlasFile = settings_->atlasLoadFile();
        saveAtlasFile = settings_->atlasSaveFile();
        std::cout << (*settings_) << std::endl;
    }
    else
    {
        settings_         = nullptr;
        cv::FileNode node = fsSettings["System.LoadAtlasFromFile"];
        if (!node.empty() && node.isString())
            loadAtlasFile = (string)node;

        node = fsSettings["System.SaveAtlasToFile"];
        if (!node.empty() && node.isString())
            saveAtlasFile = (string)node;
    }

    if ((sensor == RGBD || sensor == IMU_RGBD) && settings_ != nullptr)
    {
        const double stereoDepthThreshold = settings_->thDepth();
        const double metricCloseDepth_m = settings_->b() * stereoDepthThreshold;
        std::cout << "Stereo.ThDepth=" << stereoDepthThreshold
                  << " closeDepthMeters=" << metricCloseDepth_m << std::endl;
    }

    node          = fsSettings["loopClosing"];
    bool activeLC = true;
    if (!node.empty())
    {
        activeLC = static_cast<int>(fsSettings["loopClosing"]) != 0;
    }

    vocabularyFilePath = strVocFile;

    /* Init the ORB vocabulary */
    std::cout << "[System] Loading ORB Vocabulary ..." << std::endl;
    p_vocabulary  = new ORBVocabulary();
    bool bVocLoad = p_vocabulary->loadFromBinFile(strVocFile);
    if (!bVocLoad)
    {
        cerr << "- Wrong path to vocabulary. " << endl;
        cerr << "- Failed to open at: " << strVocFile << endl;
        exit(-1);
    }

    /* Create keyframe database */
    p_keyFrameDatabase = new KeyFrameDatabase(*p_vocabulary);

    /* Check to see if there is a string to an Atlas map file to load */
    if (loadAtlasFile.empty())
    {
        /* If no file given, create a new Atlas map */
        p_atlas = new Atlas(0);

        std::cout << "[System] Initializing Atlas from scratch in 'mpAtlas'"
                  << std::endl;
    }
    else
    {
        /* If file given, load Atlas map from earlier session */
        bool isRead = loadAtlas(FileType::BINARY_FILE);

        std::cout << "[System] Initializing Atlas from file: " << loadAtlasFile
                  << "... " << std::endl;

        if (!isRead)
        {
            std::cout << "[System] Error while loading Atlas file! Previous "
                         "Atlas file could not be loaded. Exiting ..."
                      << std::endl;
            exit(-1);
        }

        p_atlas->createNewMap();
    }

    /* Load the system parameters */
    types::SystemParams *p_sysParams = types::SystemParams::getParams();
    p_sysParams->setParams(strSysParamsFile);

    /* Parse the environment database, if provided */
    parseJsonDatabase(p_sysParams->general.envDatabase);

    /* If the sensor is integrated with IMU, initialize the IMU first */
    if (sensor == IMU_STEREO || sensor == IMU_MONOCULAR || sensor == IMU_RGBD)
    {
        p_atlas->setInertialSensor();
    }

    /* ---------------------------------------------------------------------- *
     * FRAME + MAP + TRACKER OBJECTS
     * ---------------------------------------------------------------------- */

    /* Create Drawers. These are used by the Viewer */
    p_frameDrawer = new FrameDrawer(p_atlas);
    p_mapDrawer   = new MapDrawer(p_atlas, strSettingsFile, settings_);

    /* Initialize the Tracking thread */
    p_tracker = new Tracking(this,
                             p_vocabulary,
                             p_frameDrawer,
                             p_mapDrawer,
                             p_atlas,
                             p_keyFrameDatabase,
                             strSettingsFile,
                             sensor,
                             settings_,
                             strSequence);

    /* Set the value of marker impact */
    p_tracker->setMarkerImpact(p_sysParams->markers.impact);

    /* ---------------------------------------------------------------------- *
     * LOCAL MAPPING THREAD
     * ---------------------------------------------------------------------- */

    /* Initialize the Local Mapping object */
    p_localMapper = new LocalMapping(
        this,
        p_atlas,
        sensor == MONOCULAR || sensor == IMU_MONOCULAR,
        sensor == IMU_MONOCULAR || sensor == IMU_STEREO || sensor == IMU_RGBD,
        strSequence);

    /* Set up thread to run the mpLocalMapper and call Run() method */
    mptLocalMapping =
        new thread(&vs_graphs::core::LocalMapping::run, p_localMapper);

    p_localMapper->initFrame = initFr;
    if (settings_)
    {
        p_localMapper->farPointsThreshold = settings_->thFarPoints();
    }
    else
    {
        p_localMapper->farPointsThreshold = fsSettings["thFarPoints"];
    }

    if (p_localMapper->farPointsThreshold != 0)
    {
        cout << "Discard points further than "
             << p_localMapper->farPointsThreshold << " m from current camera"
             << endl;
        p_localMapper->farPoints = true;
    }
    else
    {
        p_localMapper->farPoints = false;
    }

    /* ---------------------------------------------------------------------- *
     * LOOP CLOSING THREAD
     * ---------------------------------------------------------------------- */

    /* Initialize the Loop Closing thread */
    p_loopCloser = new LoopClosing(p_atlas,
                                   p_keyFrameDatabase,
                                   p_vocabulary,
                                   sensor != MONOCULAR,
                                   activeLC);

    /* Launch the loop closing thread */
    mptLoopClosing =
        new thread(&vs_graphs::core::LoopClosing::run, p_loopCloser);

    /* ---------------------------------------------------------------------- *
     * SEMANTIC SEGMENTATION THREAD
     * ---------------------------------------------------------------------- */

    /* Initialize the Semantic Segmentation thread */
    p_semanticSegmentation = new SemanticSegmentation(p_atlas);

    /* Launch the Semantic Segmentation thread */
    mptSemanticSegmentation =
        new thread(&SemanticSegmentation::run, p_semanticSegmentation);

    /* ---------------------------------------------------------------------- *
     * SEMANTIC MANAGER THREAD
     * ---------------------------------------------------------------------- */

    /* Initialize the Semantic Manager thread */
    p_semanticsManager = new SemanticsManager(p_atlas);

    /* Launch the Semantic Manager thread */
    mptSemanticsManager =
        new thread(&SemanticsManager::run, p_semanticsManager);

    /* ---------------------------------------------------------------------- *
     * THREAD POINTER STORAGE
     * ---------------------------------------------------------------------- */

    /* Store loop closing and local mapper thread pointers in tracker object */
    p_tracker->setLoopClosing(p_loopCloser);
    p_tracker->setLocalMapper(p_localMapper);

    /* Store tracking object and loop closing thread pointer in local mapper */
    p_localMapper->setTracker(p_tracker);
    p_localMapper->setLoopCloser(p_loopCloser);

    /* Store tracking object and local mapper thread pointer in loop closer */
    p_loopCloser->setTracker(p_tracker);
    p_loopCloser->setLocalMapper(p_localMapper);

    /* If enabled, init the viewer */
    if (bUseViewer)
    {
        p_viewer  = new Viewer(this,
                              p_frameDrawer,
                              p_mapDrawer,
                              p_tracker,
                              strSettingsFile,
                              settings_);
        mptViewer = new thread(&Viewer::run, p_viewer);
        p_tracker->setViewer(p_viewer);
        p_loopCloser->p_viewer = p_viewer;
        p_viewer->both         = p_frameDrawer->both;
    }

    /* Set verbosity level */
    Verbose::setTh(verboseLevel);
}

System::~System()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        shutdownRequested = true;
    }

    /* Request a graceful stop on every running worker thread. */
    p_localMapper->requestFinish();
    p_loopCloser->requestFinish();
    p_semanticSegmentation->requestFinish();
    p_semanticsManager->requestFinish();
    if (p_viewer != static_cast<Viewer *>(nullptr))
    {
        p_viewer->requestFinish();
    }

    /* Wait for each worker to report finished before joining. Shutdown() may
     * have already stopped Local Mapping / Loop Closing; isFinished() is
     * idempotent, join() below is the only join in the process. */
    while (
        !p_localMapper->isFinished() || !p_loopCloser->isFinished() ||
        !p_semanticSegmentation->isFinished() ||
        !p_semanticsManager->isFinished() ||
        (p_viewer != static_cast<Viewer *>(nullptr) && !p_viewer->isFinished()))
    {
        usleep(1000);
    }
    /* Join and free the thread objects (first and only join). */
    mptLocalMapping->join();
    mptLoopClosing->join();
    mptSemanticSegmentation->join();
    mptSemanticsManager->join();
    if (p_viewer != static_cast<Viewer *>(nullptr))
    {
        mptViewer->join();
    }
    clearResetCause(this);

    delete mptLocalMapping;
    delete mptLoopClosing;
    delete mptSemanticSegmentation;
    delete mptSemanticsManager;
    if (p_viewer != static_cast<Viewer *>(nullptr))
    {
        delete mptViewer;
    }
    delete mptGeometricSegmentation;
}

void System::parseJsonDatabase(string jsonFilePath)
{
    // Skip the parsing
    if (jsonFilePath.empty())
    {
        std::cout << "[System] No JSON file describing the environment is "
                     "provided. Skipping ..."
                  << std::endl;
        return;
    }
    // Creating an object of the database loader
    vs_graphs::core::DBParser parser;
    // Load JSON file
    json                      envData = parser.parseJsonFile(jsonFilePath);
    // Getting semantic entities
    envRooms = parser.getEnvironmentRooms(envData);
    // Printing the success message
    std::cout << "- JSON loaded and candidates created!\n";
}

void System::addSegmentedImage(
    std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> *tuple)
{
    // Adding the segmented image to the buffer of the SemanticSegmentation
    if (types::SystemParams::getParams()->general.modeOfOperation ==
        types::SystemParams::General::ModeOfOperation::GEO)
    {
        // just clear the pointcloud of the keyframe and return, as semantic
        // segmentation is not running. Still counts as "returned" -- the
        // keyframe's round trip through the pipeline is over either way, and
        // the lockstep backlog signal must not stall forever in GEO mode.
        vs_graphs::core::KeyFrame *pKF =
            p_atlas->getKeyFrameById(std::get<0>(*tuple));
        if (pKF)
        {
            pKF->clearPointCloud();
        }
        segmentationReturnedCount.fetch_add(1U, std::memory_order_relaxed);
        lastReturnedKeyFrameId.store(std::get<0>(*tuple),
                                     std::memory_order_relaxed);
        return;
    }

    p_semanticSegmentation->addSegmentedFrameToBuffer(tuple);
    segmentationReturnedCount.fetch_add(1U, std::memory_order_relaxed);
    lastReturnedKeyFrameId.store(std::get<0>(*tuple),
                                 std::memory_order_relaxed);
}

void System::incrementSegmentationPublishedCount()
{
    segmentationPublishedCount.fetch_add(1U, std::memory_order_relaxed);
}

std::vector<std::vector<Eigen::Vector3d>> System::getSkeletonCluster()
{
    return p_atlas->getSkeletonClusterPoints();
}

void System::setSkeletonCluster(const std::vector<std::vector<Eigen::Vector3d>>
                                    &skeletonClusterPoints_World_m_in)
{
    /* Keep asynchronous skeleton replacement atomic with map remerging. */
    std::unique_lock<std::mutex> semanticUpdateLock =
        p_atlas->acquireSemanticUpdateLock();

    /* Add the skeleton cluster to the current semantic map. */
    p_atlas->setSkeletonClusterPoints(skeletonClusterPoints_World_m_in);
}

void System::setSkeletonEdges(
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        &skeletonEdges_World_m_in)
{
    /* Keep asynchronous skeleton replacement atomic with map remerging. */
    std::unique_lock<std::mutex> semanticUpdateLock =
        p_atlas->acquireSemanticUpdateLock();

    /* Store the connected skeleton edges in the current semantic map. */
    p_atlas->setSkeletonEdges(skeletonEdges_World_m_in);
}

void System::setGNNRoomCandidates(
    [[maybe_unused]] const std::vector<vs_graphs::core::semantic::Room *>
        &gnnRoomCandidates)
{
    // [TODO] Add the GNN room candidates to the SemanticsManager
}

Sophus::SE3f System::trackStereo(const cv::Mat            &imLeft,
                                 const cv::Mat            &imRight,
                                 const double             &timestamp,
                                 const vector<IMU::Point> &vImuMeas,
                                 string                    filename,
                                 const std::vector<semantic::Marker *> markers)
{
    if (sensor != STEREO && sensor != IMU_STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to "
                "Stereo nor Stereo-Inertial."
             << endl;
        exit(-1);
    }

    cv::Mat imLeftToFeed, imRightToFeed;
    if (settings_ && settings_->needToRectify())
    {
        cv::Mat M1l = settings_->M1l();
        cv::Mat M2l = settings_->M2l();
        cv::Mat M1r = settings_->M1r();
        cv::Mat M2r = settings_->M2r();

        cv::remap(imLeft, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
    }
    else if (settings_ && settings_->needToResize())
    {
        cv::resize(imLeft, imLeftToFeed, settings_->newImSize());
        cv::resize(imRight, imRightToFeed, settings_->newImSize());
    }
    else
    {
        imLeftToFeed  = imLeft.clone();
        imRightToFeed = imRight.clone();
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if (activateLocalizationModeRequested)
        {
            p_localMapper->requestStop();

            // Wait until Local Mapping has effectively stopped
            while (!p_localMapper->isStopped())
            {
                usleep(1000);
            }

            p_tracker->informOnlyTracking(true);
            activateLocalizationModeRequested = false;
        }
        if (deactivateLocalizationModeRequested)
        {
            p_tracker->informOnlyTracking(false);
            p_localMapper->release();
            deactivateLocalizationModeRequested = false;
        }
    }

    {
        unique_lock<mutex> lock(mMutexReset);
        if (resetRequested)
        {
            (void)consumeResetCause(this);
            p_tracker->reset();
            resetCount.fetch_add(1U, std::memory_order_relaxed);
            resetRequested          = false;
            resetActiveMapRequested = false;
        }
        else if (resetActiveMapRequested)
        {
            reportResetAttribution(consumeResetCause(this),
                                   ResetAction::RESET_ACTIVE_MAP_EXECUTION);
            p_tracker->resetActiveMap();
            resetCount.fetch_add(1U, std::memory_order_relaxed);
            resetActiveMapRequested = false;
        }
    }

    if (sensor == System::IMU_STEREO)
        for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            p_tracker->grabImuData(vImuMeas[i_imu]);

    Sophus::SE3f Tcw = p_tracker->grabImageStereo(imLeftToFeed,
                                                  imRightToFeed,
                                                  timestamp,
                                                  filename,
                                                  markers,
                                                  envRooms);

    unique_lock<mutex> lock2(mMutexState);
    trackingState           = p_tracker->state;
    trackingInliers         = p_tracker->getMatchesInliers();
    lastFrameTimestamp      = timestamp;
    trackedMapPoints        = p_tracker->currentFrame.mapPoints;
    trackedKeyPointsUn      = p_tracker->currentFrame.keyPointsUndistorted;
    currentCameraPose_World = Tcw.inverse();
    currentCameraPoseValid =
        trackingState == Tracking::OK &&
        currentCameraPose_World.translation().allFinite() &&
        currentCameraPose_World.rotationMatrix().allFinite();

    return Tcw;
}

Sophus::SE3f
    System::trackRGBD(const cv::Mat                                &colorImg,
                      const cv::Mat                                &depthmap,
                      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &mainCloud,
                      const double                                 &timestamp,
                      const vector<IMU::Point>                     &vImuMeas,
                      string                                        filename,
                      const std::vector<semantic::Marker *>         markers)
{
    // Check if the sensor is correctly set as RGB-D
    if (sensor != RGBD && sensor != IMU_RGBD)
    {
        cerr << "[Error] Improper sensor-type is set for 'TrackRGBD'! Exiting "
                "..."
             << endl;
        exit(-1);
    }

    // Obtain the images
    cv::Mat imToFeed      = colorImg.clone();
    cv::Mat imDepthToFeed = depthmap.clone();
    if (settings_ && settings_->needToResize())
    {
        cv::Mat resizedImage;
        cv::resize(colorImg, resizedImage, settings_->newImSize());
        imToFeed = resizedImage;
        cv::resize(depthmap, imDepthToFeed, settings_->newImSize());
    }

    // Check for mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if (activateLocalizationModeRequested)
        {
            p_localMapper->requestStop();
            // Wait until Local Mapping has effectively stopped
            while (!p_localMapper->isStopped())
                usleep(1000);
            p_tracker->informOnlyTracking(true);
            activateLocalizationModeRequested = false;
        }
        if (deactivateLocalizationModeRequested)
        {
            p_tracker->informOnlyTracking(false);
            p_localMapper->release();
            deactivateLocalizationModeRequested = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if (resetRequested)
        {
            (void)consumeResetCause(this);
            p_tracker->reset();
            resetCount.fetch_add(1U, std::memory_order_relaxed);
            resetRequested          = false;
            resetActiveMapRequested = false;
        }
        else if (resetActiveMapRequested)
        {
            reportResetAttribution(consumeResetCause(this),
                                   ResetAction::RESET_ACTIVE_MAP_EXECUTION);
            p_tracker->resetActiveMap();
            resetCount.fetch_add(1U, std::memory_order_relaxed);
            resetActiveMapRequested = false;
        }
    }

    // Apply IMU measurements
    if (sensor == System::IMU_RGBD)
        for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            p_tracker->grabImuData(vImuMeas[i_imu]);

    // Track RGB-D images
    Sophus::SE3f Tcw = p_tracker->grabImageRGBD(imToFeed,
                                                imDepthToFeed,
                                                mainCloud,
                                                timestamp,
                                                filename,
                                                markers,
                                                envRooms);

    unique_lock<mutex> lock2(mMutexState);
    trackingState      = p_tracker->state;
    trackingInliers    = p_tracker->getMatchesInliers();
    lastFrameTimestamp = timestamp;
    trackedMapPoints   = p_tracker->currentFrame.mapPoints;
    trackedKeyPointsUn = p_tracker->currentFrame.keyPointsUndistorted;

    currentCameraPose_World = Tcw.inverse();
    currentCameraPoseValid =
        trackingState == Tracking::OK &&
        currentCameraPose_World.translation().allFinite() &&
        currentCameraPose_World.rotationMatrix().allFinite();

    /* Feed the real per-frame tracking state to SemanticsManager's reset
     * anchor (lastKnownRoomId_ via onTrackingLost()/onTrackingRecovered()).
     * Previously the ONLY caller of these was GetMissionHealthSnapshot(),
     * itself only invoked from the get_mission_health ROS service -- which
     * nothing calls unless scripts/sim_lockstep_controller.py's opt-in
     * --lockstep mode is running. Every run without --lockstep therefore
     * left lastKnownRoomId_ at its unset default (-1) for the whole
     * mission: SemanticCandidates::generate() silently fell back to
     * unanchored scoring on every single reset, never told which room the
     * UAV was actually in when tracking was lost. This is the same
     * TrackRGBD() call every real frame already goes through, so it fires
     * at real tracking-loss/recovery cadence instead of only on-demand. */
    if (p_semanticsManager != nullptr)
    {
        if (trackingState == Tracking::LOST)
        {
            p_semanticsManager->onTrackingLost();
        }
        else
        {
            p_semanticsManager->onTrackingRecovered();
        }
    }

    /* Detect map restart for room-context carryover.
     * The SemanticsManager::Run() thread performs the actual room
     * matching once rooms exist in the new map; we only log here. */
    {
        Map *currentMap = p_atlas->getCurrentMap();
        if (currentMap)
        {
            long unsigned int mapId = currentMap->getId();
            if (firstMapInit)
            {
                lastProcessedMapId = mapId;
                firstMapInit       = false;
            }
            else if (mapId != lastProcessedMapId)
            {
                const long unsigned int previousMapId = lastProcessedMapId;
                lastProcessedMapId                    = mapId;
                std::cout << "[System] Map restart detected (mapId: "
                          << previousMapId << " -> " << mapId << ")"
                          << std::endl;
                /* NOTE: matchRoomsToContext() is now called from
                 * SemanticsManager::Run() after room detection, not here. */
            }
        }
    }

    return Tcw;
}

Sophus::SE3f
    System::trackMonocular(const cv::Mat                        &im,
                           const double                         &timestamp,
                           const vector<IMU::Point>             &vImuMeas,
                           string                                filename,
                           const std::vector<semantic::Marker *> markers)
{
    // Multi-thread to prevent race conditions
    {
        unique_lock<mutex> lock(mMutexReset);
        if (shutdownRequested)
            return Sophus::SE3f();
    }

    // Check if the sensor is Monocular
    if (sensor != MONOCULAR && sensor != IMU_MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set "
                "to Monocular nor Monocular-Inertial."
             << endl;
        exit(-1);
    }

    // Obtain the images
    cv::Mat imToFeed = im.clone();
    if (settings_ && settings_->needToResize())
    {
        cv::Mat resizedImage;
        cv::resize(im, resizedImage, settings_->newImSize());
        imToFeed = resizedImage;
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if (activateLocalizationModeRequested)
        {
            p_localMapper->requestStop();

            // Wait until Local Mapping has effectively stopped
            while (!p_localMapper->isStopped())
            {
                usleep(1000);
            }

            p_tracker->informOnlyTracking(true);
            activateLocalizationModeRequested = false;
        }
        if (deactivateLocalizationModeRequested)
        {
            p_tracker->informOnlyTracking(false);
            p_localMapper->release();
            deactivateLocalizationModeRequested = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if (resetRequested)
        {
            (void)consumeResetCause(this);
            p_tracker->reset();
            resetCount.fetch_add(1U, std::memory_order_relaxed);
            resetRequested          = false;
            resetActiveMapRequested = false;
        }
        else if (resetActiveMapRequested)
        {
            reportResetAttribution(consumeResetCause(this),
                                   ResetAction::RESET_ACTIVE_MAP_EXECUTION);
            p_tracker->resetActiveMap();
            resetCount.fetch_add(1U, std::memory_order_relaxed);
            resetActiveMapRequested = false;
        }
    }

    if (sensor == System::IMU_MONOCULAR)
        for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            p_tracker->grabImuData(vImuMeas[i_imu]);

    Sophus::SE3f Tcw = p_tracker->grabImageMonocular(imToFeed,
                                                     timestamp,
                                                     filename,
                                                     markers,
                                                     envRooms);

    unique_lock<mutex> lock2(mMutexState);
    trackingState      = p_tracker->state;
    trackedMapPoints   = p_tracker->currentFrame.mapPoints;
    trackedKeyPointsUn = p_tracker->currentFrame.keyPointsUndistorted;
    return Tcw;
}

void System::activateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    activateLocalizationModeRequested = true;
}

void System::deactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    deactivateLocalizationModeRequested = true;
}

bool System::mapChanged()
{
    static int n    = 0;
    int        curn = p_atlas->getLastBigChangeIndex();
    if (n < curn)
    {
        n = curn;
        return true;
    }
    else
        return false;
}

System::MissionHealthSnapshot
    System::getMissionHealthSnapshot(bool includeSemantics)
{
    MissionHealthSnapshot snapshot;
    snapshot.inertial =
        sensor == IMU_MONOCULAR || sensor == IMU_STEREO || sensor == IMU_RGBD;

    {
        std::lock_guard<std::mutex> stateLock(mMutexState);
        snapshot.frameTimestamp   = lastFrameTimestamp;
        snapshot.trackingState    = trackingState;
        snapshot.trackingInliers  = trackingInliers;
        snapshot.poseValid        = currentCameraPoseValid;
        snapshot.cameraPose_World = currentCameraPose_World;
    }

    std::unique_lock<std::mutex> semanticUpdateLock;
    if (includeSemantics)
    {
        semanticUpdateLock = p_atlas->acquireSemanticUpdateLock();
    }
    Map *p_activeMap = p_atlas->getCurrentMap();
    snapshot.mapCount =
        static_cast<std::uint32_t>(std::max(0, p_atlas->countMaps()));
    snapshot.inertialInitialized =
        snapshot.inertial && p_atlas->isImuInitialized();
    snapshot.resetCount = resetCount.load(std::memory_order_relaxed);
    snapshot.rgbdFrontendAcceptedCount =
        rgbdFrontendAcceptedCount.load(std::memory_order_relaxed);
    snapshot.rgbdFrontendProcessedCount =
        rgbdFrontendProcessedCount.load(std::memory_order_relaxed);
    snapshot.rgbdFrontendOverwrittenCount =
        rgbdFrontendOverwrittenCount.load(std::memory_order_relaxed);
    snapshot.rgbdFrontendWorkerInFlight =
        rgbdFrontendWorkerInFlight.load(std::memory_order_relaxed);
    snapshot.rgbdFrontendLastProcessedSensorTimestampNanoseconds =
        rgbdFrontendLastProcessedSensorTimestampNanoseconds.load(
            std::memory_order_relaxed);
    snapshot.segmentationPublishedCount =
        segmentationPublishedCount.load(std::memory_order_relaxed);
    snapshot.segmentationReturnedCount =
        segmentationReturnedCount.load(std::memory_order_relaxed);
    snapshot.lastReturnedKeyFrameId =
        lastReturnedKeyFrameId.load(std::memory_order_relaxed);

    if (types::SystemParams::getParams()->general.modeOfOperation ==
        types::SystemParams::General::ModeOfOperation::GEO)
    {
        snapshot.segmentationTerminalCount = snapshot.segmentationReturnedCount;
        snapshot.lastTerminalKeyFrameId    = snapshot.lastReturnedKeyFrameId;
    }
    else if (p_semanticSegmentation != nullptr)
    {
        const SemanticSegmentation::ProcessingStats processingStats =
            p_semanticSegmentation->getProcessingStats();
        snapshot.segmentationEnqueuedCount = processingStats.enqueuedCount;
        snapshot.segmentationDequeuedCount = processingStats.dequeuedCount;
        snapshot.segmentationTerminalCount = processingStats.terminalCount;
        snapshot.segmentationAcceptedCount = processingStats.acceptedCount;
        snapshot.segmentationDroppedCount  = processingStats.droppedCount;
        snapshot.segmentationMissingKeyFrameCount =
            processingStats.missingKeyFrameCount;
        snapshot.segmentationMissingCloudCount =
            processingStats.missingCloudCount;
        snapshot.segmentationStaleMapCount = processingStats.staleMapCount;
        snapshot.lastTerminalKeyFrameId =
            processingStats.lastTerminalKeyFrameId;
        snapshot.segmentationQueueDepth = processingStats.queueDepth;
        snapshot.segmentationQueueHighWatermark =
            processingStats.queueHighWatermark;
    }

    if (p_semanticsManager != nullptr)
    {
        snapshot.currentRoomId = p_semanticsManager->getCurrentRoomId();
        if (snapshot.trackingState == Tracking::LOST)
        {
            p_semanticsManager->onTrackingLost();
        }
        else
        {
            p_semanticsManager->onTrackingRecovered();
        }
        snapshot.lastKnownRoomId = p_semanticsManager->getLastKnownRoomId();
    }

    if (p_activeMap != nullptr)
    {
        snapshot.mapId = static_cast<std::uint64_t>(p_activeMap->getId());
        const std::vector<KeyFrame *> keyFrames =
            p_activeMap->getAllKeyFrames();
        snapshot.keyFrameCount = static_cast<std::uint32_t>(keyFrames.size());
        KeyFrame *p_latestKeyFrame = nullptr;
        for (KeyFrame *p_keyFrame : keyFrames)
        {
            if (p_keyFrame != nullptr && !p_keyFrame->isBad() &&
                (p_latestKeyFrame == nullptr ||
                 p_keyFrame->mnId > p_latestKeyFrame->mnId))
            {
                p_latestKeyFrame = p_keyFrame;
            }
        }
        if (p_latestKeyFrame != nullptr)
        {
            snapshot.latestKeyFrameTimestamp = p_latestKeyFrame->timeStamp;
            snapshot.latestKeyFramePose_World =
                p_latestKeyFrame->getPoseInverse();
            snapshot.latestKeyFramePoseValid =
                snapshot.latestKeyFramePose_World.translation().allFinite() &&
                snapshot.latestKeyFramePose_World.rotationMatrix().allFinite();
        }

        if (includeSemantics)
        {
            for (semantic::Room *p_room : p_activeMap->getAllRooms())
            {
                if (p_room == nullptr || p_room->isBad())
                {
                    continue;
                }
                if (p_room->getRoomVariant() !=
                    semantic::Room::RoomVariant::ROOM)
                {
                    ++snapshot.unresolvedRoomCount;
                    continue;
                }

                ++snapshot.confirmedRoomCount;
                RoomHealth room;
                room.id = p_room->getId();
                for (semantic::Passage *p_passage : p_room->getPassages())
                {
                    if (p_passage != nullptr)
                    {
                        room.passageIds.push_back(p_passage->getId());
                    }
                }
                std::sort(room.passageIds.begin(), room.passageIds.end());
                snapshot.rooms.push_back(std::move(room));
            }

            for (semantic::Floor *p_floor : p_activeMap->getAllFloors())
            {
                if (p_floor == nullptr)
                {
                    continue;
                }
                FloorHealth floor;
                floor.id = p_floor->getId();
                for (semantic::Room *p_room : p_floor->getRooms())
                {
                    if (p_room != nullptr && !p_room->isBad() &&
                        p_room->getRoomVariant() ==
                            semantic::Room::RoomVariant::ROOM)
                    {
                        floor.roomIds.push_back(p_room->getId());
                        ++snapshot.floorRoomLinkCount;
                    }
                }
                std::sort(floor.roomIds.begin(), floor.roomIds.end());
                snapshot.floors.push_back(std::move(floor));
            }

            for (semantic::Passage *p_passage : p_activeMap->getAllPassages())
            {
                if (p_passage == nullptr)
                {
                    continue;
                }
                PassageHealth passage;
                passage.id       = p_passage->getId();
                passage.passable = p_passage->isPassable();
                passage.primaryRoomId =
                    p_passage->getKnownSideProvenance().pRoom
                        ? p_passage->getKnownSideProvenance().pRoom->getId()
                        : -1;
                passage.secondaryRoomId =
                    p_passage->getProspectiveRoom()
                        ? p_passage->getProspectiveRoom()->getId()
                        : -1;
                passage.primaryTraversalCount =
                    p_passage->getTraversalKnownToFarCount();
                passage.secondaryTraversalCount =
                    p_passage->getTraversalFarToKnownCount();
                passage.unknownCount = p_passage->getTraversalUnknownCount();
                const semantic::Passage::KnownSideProvenance knownSide =
                    p_passage->getKnownSideProvenance();
                if (knownSide.pRoom != nullptr)
                {
                    passage.primaryRoomId = knownSide.pRoom->getId();
                }
                semantic::Room *p_farSideRoom = p_passage->getProspectiveRoom();
                if (p_farSideRoom != nullptr)
                {
                    passage.secondaryRoomId = p_farSideRoom->getId();
                }
                snapshot.passages.push_back(passage);
            }
        }
    }

    if (semanticUpdateLock.owns_lock())
    {
        semanticUpdateLock.unlock();
    }
    if (p_loopCloser != nullptr)
    {
        const LoopClosing::LoopCorrectionStatus loop =
            p_loopCloser->getLoopCorrectionStatus();
        snapshot.loopSequence              = loop.sequence;
        snapshot.acceptedLoopCount         = loop.acceptedCount;
        snapshot.rejectedLoopCount         = loop.rejectedCount;
        snapshot.hasLoopEvent              = loop.hasEvent;
        snapshot.lastLoopAccepted          = loop.lastAccepted;
        snapshot.lastLoopMapId             = loop.lastMapId;
        snapshot.lastLoopCurrentKeyFrameId = loop.lastCurrentKeyFrameId;
        snapshot.lastLoopMatchedKeyFrameId = loop.lastMatchedKeyFrameId;
        snapshot.lastLoopCurrentTimestamp  = loop.lastCurrentTimestamp;
        snapshot.lastLoopMatchedTimestamp  = loop.lastMatchedTimestamp;
        snapshot.lastLoopReason            = loop.lastReason;
    }
    return snapshot;
}

void System::updateRgbdFrontendHealth(
    const std::uint64_t acceptedCount_in,
    const std::uint64_t processedCount_in,
    const std::uint64_t overwrittenCount_in,
    const bool          isWorkerInFlight_in,
    const std::int64_t  lastProcessedSensorTimestampNanoseconds_in) noexcept
{
    rgbdFrontendAcceptedCount.store(acceptedCount_in,
                                    std::memory_order_relaxed);
    rgbdFrontendProcessedCount.store(processedCount_in,
                                     std::memory_order_relaxed);
    rgbdFrontendOverwrittenCount.store(overwrittenCount_in,
                                       std::memory_order_relaxed);
    rgbdFrontendWorkerInFlight.store(isWorkerInFlight_in,
                                     std::memory_order_relaxed);
    rgbdFrontendLastProcessedSensorTimestampNanoseconds.store(
        lastProcessedSensorTimestampNanoseconds_in,
        std::memory_order_relaxed);
}

semantic::SemanticReportCacheEntry System::getSemanticReportCacheEntry() const
{
    if (p_semanticsManager == nullptr)
    {
        return semantic::SemanticReportCacheEntry();
    }
    return p_semanticsManager->getSemanticReportCacheEntry();
}

bool System::isSemanticReportCacheAvailable() const
{
    return p_semanticsManager != nullptr &&
           p_semanticsManager->isSemanticReportCacheAvailable();
}

void System::reset()
{
    unique_lock<mutex> lock(mMutexReset);
    resetRequested = true;
}

void System::resetActiveMap()
{
    requestResetActiveMapWithCause(ResetCause::UNATTRIBUTED_PUBLIC_REQUEST);
}

void System::requestResetActiveMapWithCause(const ResetCause cause_in)
{
    unique_lock<mutex> lock(mMutexReset);
    retainResetCause(this, cause_in);
    resetActiveMapRequested = true;
    reportResetAttribution(cause_in, ResetAction::RESET_ACTIVE_MAP_REQUEST);
}

void System::shutdown()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        shutdownRequested = true;
    }

    cout << "Shutdown" << endl;

    p_localMapper->requestFinish();
    p_loopCloser->requestFinish();
    p_semanticSegmentation->requestFinish();
    p_semanticsManager->requestFinish();
    if (p_viewer != static_cast<Viewer *>(nullptr))
    {
        p_viewer->requestFinish();
    }

    /*
     * Workers report finished only after their owned work has drained. Waiting
     * here prevents Atlas serialization from racing final worker updates.
     */
    std::size_t shutdownPollCount = 0U;
    while (
        !p_localMapper->isFinished() || !p_loopCloser->isFinished() ||
        !p_semanticSegmentation->isFinished() ||
        !p_semanticsManager->isFinished() ||
        (p_viewer != static_cast<Viewer *>(nullptr) && !p_viewer->isFinished()))
    {
        usleep(1000);
        ++shutdownPollCount;
        if (shutdownPollCount % 1000U == 0U)
        {
            const bool localMappingFinished = p_localMapper->isFinished();
            const bool loopClosingFinished  = p_loopCloser->isFinished();
            const bool semanticSegmentationFinished =
                p_semanticSegmentation->isFinished();
            const bool semanticsManagerFinished =
                p_semanticsManager->isFinished();
            const bool viewerFinished =
                p_viewer == static_cast<Viewer *>(nullptr) ||
                p_viewer->isFinished();
            std::cout << "[System::Shutdown] local_mapping="
                      << localMappingFinished
                      << " loop_closing=" << loopClosingFinished
                      << " semantic_segmentation="
                      << semanticSegmentationFinished
                      << " semantics_manager=" << semanticsManagerFinished
                      << " viewer=" << viewerFinished << std::endl;
        }
    }

    std::cout << "[System::Shutdown] all workers completed [flushed]"
              << std::endl;

    if (!saveAtlasFile.empty())
    {
        Verbose::printMess("Atlas saving to file " + saveAtlasFile,
                           Verbose::VERBOSITY_NORMAL);

        std::unique_lock<std::mutex> semanticUpdateLock =
            p_atlas->acquireSemanticUpdateLock();
        saveAtlas(FileType::BINARY_FILE);
    }

#ifdef REGISTER_TIMES
    p_tracker->printTimeStats();
#endif
}

bool System::isShutDown()
{
    unique_lock<mutex> lock(mMutexReset);
    return shutdownRequested;
}

void System::saveTrajectoryTUM(const string &filename)
{
    cout << endl
         << "Saving camera trajectory to " << filename << " ..." << endl;
    if (sensor == MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular."
             << endl;
        return;
    }

    vector<KeyFrame *> vpKFs = p_atlas->getAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Two = vpKFs[0]->getPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is
    // optimized by BA and pose graph). We need to get first the keyframe pose
    // and then concatenate the relative transformation. Frames not localized
    // (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
    // and a flag which is true when tracking failed (lbL).
    list<vs_graphs::core::KeyFrame *>::iterator lRit =
        p_tracker->mlpReferences.begin();
    list<double>::iterator lT  = p_tracker->frameTimes.begin();
    list<bool>::iterator   lbL = p_tracker->mlbLost.begin();
    for (list<Sophus::SE3f>::iterator
             lit  = p_tracker->relativeFramePoses.begin(),
             lend = p_tracker->relativeFramePoses.end();
         lit != lend;
         lit++, lRit++, lT++, lbL++)
    {
        if (*lbL)
            continue;

        KeyFrame *pKF = *lRit;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to
        // get a suitable keyframe.
        while (pKF->isBad())
        {
            Trw = Trw * pKF->tcp;
            pKF = pKF->getParent();
        }

        Trw = Trw * pKF->getPose() * Two;

        Sophus::SE3f Tcw = (*lit) * Trw;
        Sophus::SE3f Twc = Tcw.inverse();

        Eigen::Vector3f    twc = Twc.translation();
        Eigen::Quaternionf q   = Twc.unit_quaternion();

        f << setprecision(6) << *lT << " " << setprecision(9) << twc(0) << " "
          << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " "
          << q.z() << " " << q.w() << endl;
    }
    f.close();
}

void System::saveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl
         << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame *> vpKFs = p_atlas->getAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame *pKF = vpKFs[i];

        if (pKF->isBad())
            continue;

        Sophus::SE3f       Twc = pKF->getPoseInverse();
        Eigen::Quaternionf q   = Twc.unit_quaternion();
        Eigen::Vector3f    t   = Twc.translation();
        f << setprecision(6) << pKF->timeStamp << setprecision(7) << " " << t(0)
          << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " "
          << q.z() << " " << q.w() << endl;
    }

    f.close();
}

void System::saveTrajectoryEuRoC(const string &filename)
{

    cout << endl << "Saving trajectory to " << filename << " ..." << endl;

    vector<Map *> vpMaps      = p_atlas->getAllMaps();
    std::size_t   numMaxKFs   = 0;
    Map          *p_biggerMap = nullptr;
    std::cout << "There are " << std::to_string(vpMaps.size())
              << " maps in the atlas" << std::endl;
    for (Map *pMap : vpMaps)
    {
        if (pMap == nullptr)
        {
            continue;
        }

        const std::size_t keyFrameCount = pMap->getAllKeyFrames().size();

        std::cout << "  Map " << std::to_string(pMap->getId()) << " has "
                  << std::to_string(keyFrameCount) << " KFs" << std::endl;
        if (keyFrameCount > numMaxKFs)
        {
            numMaxKFs   = keyFrameCount;
            p_biggerMap = pMap;
        }
    }

    if (p_biggerMap == nullptr)
    {
        std::cerr << "Cannot save a trajectory: the Atlas has no keyframes."
                  << std::endl;
        return;
    }

    vector<KeyFrame *> vpKFs = p_biggerMap->getAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f
        Twb; // Can be word to cam0 or world to b depending on IMU or not.
    if (sensor == IMU_MONOCULAR || sensor == IMU_STEREO || sensor == IMU_RGBD)
        Twb = vpKFs[0]->getImuPose();
    else
        Twb = vpKFs[0]->getPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    // cout << "file open" << endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is
    // optimized by BA and pose graph). We need to get first the keyframe pose
    // and then concatenate the relative transformation. Frames not localized
    // (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
    // and a flag which is true when tracking failed (lbL).
    list<vs_graphs::core::KeyFrame *>::iterator lRit =
        p_tracker->mlpReferences.begin();
    list<double>::iterator lT  = p_tracker->frameTimes.begin();
    list<bool>::iterator   lbL = p_tracker->mlbLost.begin();

    for (auto lit  = p_tracker->relativeFramePoses.begin(),
              lend = p_tracker->relativeFramePoses.end();
         lit != lend;
         lit++, lRit++, lT++, lbL++)
    {
        if (*lbL)
            continue;

        KeyFrame *pKF = *lRit;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to
        // get a suitable keyframe.
        if (!pKF)
            continue;

        while (pKF->isBad())
        {
            Trw = Trw * pKF->tcp;
            pKF = pKF->getParent();
        }

        if (!pKF || pKF->getMap() != p_biggerMap)
            continue;

        Trw = Trw * pKF->getPose() *
              Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        if (sensor == IMU_MONOCULAR || sensor == IMU_STEREO ||
            sensor == IMU_RGBD)
        {
            Sophus::SE3f Twb =
                (pKF->imuCalibration.mTbc * (*lit) * Trw).inverse();
            Eigen::Quaternionf q   = Twb.unit_quaternion();
            Eigen::Vector3f    twb = Twb.translation();
            f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9)
              << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " "
              << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        else
        {
            Sophus::SE3f       Twc = ((*lit) * Trw).inverse();
            Eigen::Quaternionf q   = Twc.unit_quaternion();
            Eigen::Vector3f    twc = Twc.translation();
            f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9)
              << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " "
              << q.y() << " " << q.z() << " " << q.w() << endl;
        }
    }

    f.close();
    cout << endl
         << "End of saving trajectory to " << filename << " ..." << endl;
}

void System::saveTrajectoryEuRoC(const string &filename, Map *pMap)
{

    cout << endl
         << "Saving trajectory of map " << pMap->getId() << " to " << filename
         << " ..." << endl;

    vector<KeyFrame *> vpKFs = pMap->getAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f
        Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
    if (sensor == IMU_MONOCULAR || sensor == IMU_STEREO || sensor == IMU_RGBD)
        Twb = vpKFs[0]->getImuPose();
    else
        Twb = vpKFs[0]->getPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is
    // optimized by BA and pose graph). We need to get first the keyframe pose
    // and then concatenate the relative transformation. Frames not localized
    // (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
    // and a flag which is true when tracking failed (lbL).
    list<vs_graphs::core::KeyFrame *>::iterator lRit =
        p_tracker->mlpReferences.begin();
    list<double>::iterator lT  = p_tracker->frameTimes.begin();
    list<bool>::iterator   lbL = p_tracker->mlbLost.begin();

    for (auto lit  = p_tracker->relativeFramePoses.begin(),
              lend = p_tracker->relativeFramePoses.end();
         lit != lend;
         lit++, lRit++, lT++, lbL++)
    {
        if (*lbL)
            continue;

        KeyFrame *pKF = *lRit;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to
        // get a suitable keyframe.
        if (!pKF)
            continue;

        while (pKF->isBad())
        {
            Trw = Trw * pKF->tcp;
            pKF = pKF->getParent();
        }

        if (!pKF || pKF->getMap() != pMap)
            continue;

        Trw = Trw * pKF->getPose() *
              Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        if (sensor == IMU_MONOCULAR || sensor == IMU_STEREO ||
            sensor == IMU_RGBD)
        {
            Sophus::SE3f Twb =
                (pKF->imuCalibration.mTbc * (*lit) * Trw).inverse();
            Eigen::Quaternionf q   = Twb.unit_quaternion();
            Eigen::Vector3f    twb = Twb.translation();
            f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9)
              << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " "
              << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        else
        {
            Sophus::SE3f       Twc = ((*lit) * Trw).inverse();
            Eigen::Quaternionf q   = Twc.unit_quaternion();
            Eigen::Vector3f    twc = Twc.translation();
            f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9)
              << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " "
              << q.y() << " " << q.z() << " " << q.w() << endl;
        }
    }
    f.close();
    cout << endl
         << "End of saving trajectory to " << filename << " ..." << endl;
}

void System::saveKeyFrameTrajectoryEuRoC(const string &filename)
{
    cout << endl
         << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<Map *> vpMaps      = p_atlas->getAllMaps();
    Map          *p_biggerMap = nullptr;
    std::size_t   numMaxKFs   = 0;
    for (Map *pMap : vpMaps)
    {
        if (pMap && pMap->getAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs   = pMap->getAllKeyFrames().size();
            p_biggerMap = pMap;
        }
    }

    if (!p_biggerMap)
    {
        std::cout << "There is not a map!!" << std::endl;
        return;
    }

    vector<KeyFrame *> vpKFs = p_biggerMap->getAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame *pKF = vpKFs[i];

        if (!pKF || pKF->isBad())
            continue;
        if (sensor == IMU_MONOCULAR || sensor == IMU_STEREO ||
            sensor == IMU_RGBD)
        {
            Sophus::SE3f       Twb = pKF->getImuPose();
            Eigen::Quaternionf q   = Twb.unit_quaternion();
            Eigen::Vector3f    twb = Twb.translation();
            f << setprecision(6) << 1e9 * pKF->timeStamp << " "
              << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2)
              << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
              << endl;
        }
        else
        {
            Sophus::SE3f       Twc = pKF->getPoseInverse();
            Eigen::Quaternionf q   = Twc.unit_quaternion();
            Eigen::Vector3f    t   = Twc.translation();
            f << setprecision(6) << 1e9 * pKF->timeStamp << " "
              << setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " "
              << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
    }
    f.close();
}

void System::saveKeyFrameTrajectoryEuRoC(const string &filename, Map *pMap)
{
    cout << endl
         << "Saving keyframe trajectory of map " << pMap->getId() << " to "
         << filename << " ..." << endl;

    vector<KeyFrame *> vpKFs = pMap->getAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame *pKF = vpKFs[i];

        if (!pKF || pKF->isBad())
            continue;
        if (sensor == IMU_MONOCULAR || sensor == IMU_STEREO ||
            sensor == IMU_RGBD)
        {
            Sophus::SE3f       Twb = pKF->getImuPose();
            Eigen::Quaternionf q   = Twb.unit_quaternion();
            Eigen::Vector3f    twb = Twb.translation();
            f << setprecision(6) << 1e9 * pKF->timeStamp << " "
              << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2)
              << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
              << endl;
        }
        else
        {
            Sophus::SE3f       Twc = pKF->getPoseInverse();
            Eigen::Quaternionf q   = Twc.unit_quaternion();
            Eigen::Vector3f    t   = Twc.translation();
            f << setprecision(6) << 1e9 * pKF->timeStamp << " "
              << setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " "
              << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
    }
    f.close();
}

void System::saveTrajectoryKITTI(const string &filename)
{
    cout << endl
         << "Saving camera trajectory to " << filename << " ..." << endl;
    if (sensor == MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular."
             << endl;
        return;
    }

    vector<KeyFrame *> vpKFs = p_atlas->getAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Tow = vpKFs[0]->getPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is
    // optimized by BA and pose graph). We need to get first the keyframe pose
    // and then concatenate the relative transformation. Frames not localized
    // (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
    // and a flag which is true when tracking failed (lbL).
    list<vs_graphs::core::KeyFrame *>::iterator lRit =
        p_tracker->mlpReferences.begin();
    list<double>::iterator lT = p_tracker->frameTimes.begin();
    for (list<Sophus::SE3f>::iterator
             lit  = p_tracker->relativeFramePoses.begin(),
             lend = p_tracker->relativeFramePoses.end();
         lit != lend;
         lit++, lRit++, lT++)
    {
        vs_graphs::core::KeyFrame *pKF = *lRit;

        Sophus::SE3f Trw;

        if (!pKF)
            continue;

        while (pKF->isBad())
        {
            Trw = Trw * pKF->tcp;
            pKF = pKF->getParent();
        }

        Trw = Trw * pKF->getPose() * Tow;

        Sophus::SE3f    Tcw = (*lit) * Trw;
        Sophus::SE3f    Twc = Tcw.inverse();
        Eigen::Matrix3f Rwc = Twc.rotationMatrix();
        Eigen::Vector3f twc = Twc.translation();

        f << setprecision(9) << Rwc(0, 0) << " " << Rwc(0, 1) << " "
          << Rwc(0, 2) << " " << twc(0) << " " << Rwc(1, 0) << " " << Rwc(1, 1)
          << " " << Rwc(1, 2) << " " << twc(1) << " " << Rwc(2, 0) << " "
          << Rwc(2, 1) << " " << Rwc(2, 2) << " " << twc(2) << endl;
    }
    f.close();
}

void System::saveDebugData(const int &initIdx)
{
    // 0. Save initialization trajectory
    saveTrajectoryEuRoC("init_FrameTrajectoy_" +
                        to_string(p_localMapper->initSection) + "_" +
                        to_string(initIdx) + ".txt");

    // 1. Save scale
    ofstream f;
    f.open("init_Scale_" + to_string(p_localMapper->initSection) + ".txt",
           ios_base::app);
    f << fixed;
    f << p_localMapper->scale << endl;
    f.close();

    // 2. Save gravity direction
    f.open("init_GDir_" + to_string(p_localMapper->initSection) + ".txt",
           ios_base::app);
    f << fixed;
    f << p_localMapper->mRwg(0, 0) << "," << p_localMapper->mRwg(0, 1) << ","
      << p_localMapper->mRwg(0, 2) << endl;
    f << p_localMapper->mRwg(1, 0) << "," << p_localMapper->mRwg(1, 1) << ","
      << p_localMapper->mRwg(1, 2) << endl;
    f << p_localMapper->mRwg(2, 0) << "," << p_localMapper->mRwg(2, 1) << ","
      << p_localMapper->mRwg(2, 2) << endl;
    f.close();

    // 3. Save computational cost
    f.open("init_CompCost_" + to_string(p_localMapper->initSection) + ".txt",
           ios_base::app);
    f << fixed;
    f << p_localMapper->costTime << endl;
    f.close();

    // 4. Save biases
    f.open("init_Biases_" + to_string(p_localMapper->initSection) + ".txt",
           ios_base::app);
    f << fixed;
    f << p_localMapper->mbg(0) << "," << p_localMapper->mbg(1) << ","
      << p_localMapper->mbg(2) << endl;
    f << p_localMapper->mba(0) << "," << p_localMapper->mba(1) << ","
      << p_localMapper->mba(2) << endl;
    f.close();

    // 5. Save covariance matrix
    f.open("init_CovMatrix_" + to_string(p_localMapper->initSection) + "_" +
               to_string(initIdx) + ".txt",
           ios_base::app);
    f << fixed;
    for (int i = 0; i < p_localMapper->mcovInertial.rows(); i++)
    {
        for (int j = 0; j < p_localMapper->mcovInertial.cols(); j++)
        {
            if (j != 0)
                f << ",";
            f << setprecision(15) << p_localMapper->mcovInertial(i, j);
        }
        f << endl;
    }
    f.close();

    // 6. Save initialization time
    f.open("init_Time_" + to_string(p_localMapper->initSection) + ".txt",
           ios_base::app);
    f << fixed;
    f << p_localMapper->initTime << endl;
    f.close();
}

int System::getTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return trackingState;
}

vector<MapPoint *> System::getTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return trackedMapPoints;
}

vector<cv::KeyPoint> System::getTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return trackedKeyPointsUn;
}

cv::Mat System::getCurrentFrame()
{
    return p_frameDrawer->drawFrame();
}

std::vector<KeyFrame *> System::getAllKeyFrames()
{
    return p_atlas->getAllKeyFrames();
}

Sophus::SE3f System::getCamTwc()
{
    return p_tracker->getCamTwc();
}

Sophus::SE3f System::getImuTwb()
{
    return p_tracker->getImuTwb();
}

Eigen::Vector3f System::getImuVwb()
{
    return p_tracker->getImuVwb();
}

bool System::isImuPreintegrated()
{
    return p_tracker->isImuPreintegrated();
}

double System::getTimeFromIMUInit()
{
    double aux =
        p_localMapper->getCurrentKeyFrameTime() - p_localMapper->firstTimestamp;
    if ((aux > 0.) && p_atlas->isImuInitialized())
        return p_localMapper->getCurrentKeyFrameTime() -
               p_localMapper->firstTimestamp;
    else
        return 0.f;
}

bool System::isLost()
{
    if (!p_atlas->isImuInitialized())
        return false;
    else
    {
        if ((p_tracker->state ==
             Tracking::LOST)) //||(mpTracker->mState==Tracking::RECENTLY_LOST))
            return true;
        else
            return false;
    }
}

bool System::isFinished()
{
    return (getTimeFromIMUInit() > 0.1);
}

void System::changeDataset()
{
    if (p_atlas->getCurrentMap()->getKeyFrameCount() < 12)
    {
        reportResetAttribution(ResetCause::DATASET_CHANGE_SMALL_MAP,
                               ResetAction::RESET_ACTIVE_MAP_EXECUTION);
        p_tracker->resetActiveMap();
        resetCount.fetch_add(1U, std::memory_order_relaxed);
    }
    else
    {
        reportResetAttribution(ResetCause::DATASET_CHANGE_NEW_MAP,
                               ResetAction::CREATE_MAP_EXECUTION);
        p_tracker->createMapInAtlas();
    }

    p_tracker->newDataset();
}

float System::getImageScale()
{
    return p_tracker->getImageScale();
}

#ifdef REGISTER_TIMES
void System::insertRectTime(double &time)
{
    p_tracker->vdRectStereo_ms.push_back(time);
}

void System::insertResizeTime(double &time)
{
    p_tracker->vdResizeImage_ms.push_back(time);
}

void System::insertTrackTime(double &time)
{
    p_tracker->vdTrackTotal_ms.push_back(time);
}
#endif

bool System::saveAtlas(int type)
{
    try
    {
        if (!saveAtlasFile.empty())
        {
            // Save the current session
            p_atlas->PreSave();

            string pathSaveFileName = "./";
            pathSaveFileName        = pathSaveFileName.append(saveAtlasFile);
            pathSaveFileName        = pathSaveFileName.append(".osa");

            string strVocabularyChecksum =
                calculateCheckSum(vocabularyFilePath, TEXT_FILE);
            std::size_t found        = vocabularyFilePath.find_last_of("/\\");
            string strVocabularyName = vocabularyFilePath.substr(found + 1);

            if (type == TEXT_FILE) // File text
            {
                cout << "Starting to write the save text file to "
                     << pathSaveFileName.c_str() << endl;
                std::remove(pathSaveFileName.c_str());
                std::ofstream ofs(pathSaveFileName, std::ios::binary);
                boost::archive::text_oarchive oa(ofs);

                oa << strVocabularyName;
                oa << strVocabularyChecksum;
                oa << p_atlas;
                cout << "End to write the save text file" << endl;
            }
            else if (type == BINARY_FILE) // File binary
            {
                cout << "Starting to write the save binary file to "
                     << pathSaveFileName.c_str() << endl;
                std::remove(pathSaveFileName.c_str());
                std::ofstream ofs(pathSaveFileName, std::ios::binary);
                boost::archive::binary_oarchive oa(ofs);
                oa << strVocabularyName;
                oa << strVocabularyChecksum;
                oa << p_atlas;
                cout << "End to write save binary file" << endl;
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return false;
    }
    catch (...)
    {
        std::cerr << "Unknows exeption" << std::endl;
        return false;
    }

    return true;
}

bool System::loadAtlas(int type)
{
    string strFileVoc, strVocChecksum;
    bool   isRead = false;

    string pathLoadFileName = "./";
    pathLoadFileName        = pathLoadFileName.append(loadAtlasFile);
    pathLoadFileName        = pathLoadFileName.append(".osa");

    if (type == TEXT_FILE) // File text
    {
        cout << "Starting to read the save text file "
             << pathLoadFileName.c_str() << endl;
        std::ifstream ifs(pathLoadFileName, std::ios::binary);
        if (!ifs.good())
        {
            cout << "Load file not found" << endl;
            return false;
        }
        boost::archive::text_iarchive ia(ifs);
        ia >> strFileVoc;
        ia >> strVocChecksum;
        ia >> p_atlas;
        cout << "End to load the save text file " << endl;
        isRead = true;
    }
    else if (type == BINARY_FILE) // File binary
    {
        cout << "Starting to read the save binary file "
             << pathLoadFileName.c_str() << endl;
        std::ifstream ifs(pathLoadFileName, std::ios::binary);
        if (!ifs.good())
        {
            cout << "Load file not found" << endl;
            return false;
        }
        boost::archive::binary_iarchive ia(ifs);
        ia >> strFileVoc;
        ia >> strVocChecksum;
        ia >> p_atlas;
        cout << "End to load the save binary file" << endl;
        isRead = true;
    }

    if (isRead)
    {
        // Check if the vocabulary is the same
        string strInputVocabularyChecksum =
            calculateCheckSum(vocabularyFilePath, TEXT_FILE);

        if (strInputVocabularyChecksum.compare(strVocChecksum) != 0)
        {
            cout << "The vocabulary load isn't the same which the load session "
                    "was created "
                 << endl;
            cout << "-Vocabulary name: " << strFileVoc << endl;
            return false; // Both are differents
        }

        p_atlas->setKeyFrameDatabase(p_keyFrameDatabase);
        p_atlas->setORBVocabulary(p_vocabulary);
        p_atlas->PostLoad();

        return true;
    }
    return false;
}

string System::calculateCheckSum(string filename, int type)
{
    string checksum = "";

    std::ios_base::openmode flags = std::ios::in;
    if (type == BINARY_FILE) // Binary file
        flags = std::ios::in | std::ios::binary;

    ifstream f(filename.c_str(), flags);
    if (!f.is_open())
    {
        cout << "[E] Unable to open the in file " << filename
             << " for Md5 hash." << endl;
        return checksum;
    }

    /*
     * OpenSSL 3 deprecates the MD5_* calls, so the identical MD5 digest is
     * taken through the EVP interface. The context is owned for the whole
     * scope so that every early return releases it.
     */
    const std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>
        p_digestContext(EVP_MD_CTX_new(), &EVP_MD_CTX_free);

    if (!p_digestContext)
    {
        cout << "[E] Unable to allocate the Md5 context for " << filename
             << "." << endl;
        return checksum;
    }

    if (EVP_DigestInit_ex(p_digestContext.get(), EVP_md5(), nullptr) != 1)
    {
        cout << "[E] Unable to start the Md5 hash of " << filename << "."
             << endl;
        return checksum;
    }

    char buffer[1024];

    while (int count = f.readsome(buffer, sizeof(buffer)))
    {
        if (EVP_DigestUpdate(p_digestContext.get(),
                             buffer,
                             static_cast<std::size_t>(count)) != 1)
        {
            cout << "[E] Unable to hash the contents of " << filename << "."
                 << endl;
            return checksum;
        }
    }

    f.close();

    unsigned char digest[EVP_MAX_MD_SIZE];
    unsigned int  digestLength_bytes = 0U;

    if (EVP_DigestFinal_ex(p_digestContext.get(),
                           digest,
                           &digestLength_bytes) != 1)
    {
        cout << "[E] Unable to finish the Md5 hash of " << filename << "."
             << endl;
        return checksum;
    }

    for (unsigned int i = 0; i < digestLength_bytes; i++)
    {
        char aux[10];
        sprintf(aux, "%02x", digest[i]);
        checksum = checksum + aux;
    }

    return checksum;
}

vs_graphs::core::Map *System::getCurrentMap()
{
    vs_graphs::core::Map *pActiveMap = p_atlas->getCurrentMap();
    return pActiveMap;
}

vs_graphs::core::Atlas *System::getAtlas()
{
    return p_atlas;
}

vector<MapPoint *> System::getAllMapPoints()
{
    Map *pActiveMap = p_atlas->getCurrentMap();
    return pActiveMap->getAllMapPoints();
}

vector<Sophus::SE3f> System::getAllKeyframePoses()
{
    vector<KeyFrame *> vpKFs = p_atlas->getAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    vector<Sophus::SE3f> vKFposes;

    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame *pKF = vpKFs[i];

        if (pKF->isBad())
            continue;

        // Twb can be world frame to cam0 frame (without IMU) or body in world
        // frame (with IMU)
        Sophus::SE3f Twb;
        if (sensor == IMU_MONOCULAR || sensor == IMU_STEREO ||
            sensor == IMU_RGBD) // with IMU
            Twb = vpKFs[i]->getImuPose();
        else // without IMU
            Twb = vpKFs[i]->getPoseInverse();

        vKFposes.push_back(Twb);
    }

    return vKFposes;
}

Sophus::SE3f System::getKeyFramePose(KeyFrame *pKF)
{
    if (pKF->isBad())
        return Sophus::SE3f();

    // Twb can be world frame to cam0 frame (without IMU) or body in world frame
    // (with IMU)
    Sophus::SE3f Twb;
    if (sensor == IMU_MONOCULAR || sensor == IMU_STEREO ||
        sensor == IMU_RGBD) // with IMU
        Twb = pKF->getImuPose();
    else // without IMU
        Twb = pKF->getPoseInverse();

    return Twb;
}

vector<semantic::Marker *> System::getAllMarkers()
{
    Map *pActiveMap = p_atlas->getCurrentMap();
    return pActiveMap->getAllMarkers();
}

std::vector<vs_graphs::core::semantic::Passage *> System::getAllPassages()
{
    Map *pActiveMap = p_atlas->getCurrentMap();
    return pActiveMap->getAllPassages();
}

vector<geometric::Plane *> System::getAllPlanes()
{
    Map *pActiveMap = p_atlas->getCurrentMap();
    return pActiveMap->getAllPlanes();
}

vector<semantic::Room *> System::getAllRooms()
{
    Map *pActiveMap = p_atlas->getCurrentMap();
    return pActiveMap->getAllRooms();
}

std::vector<vs_graphs::core::Door *> System::getAllDoors()
{
    vs_graphs::core::Map *pActiveMap = p_atlas->getCurrentMap();
    return pActiveMap->getAllDoors();
}

std::vector<vs_graphs::core::semantic::Floor *> System::getAllFloors()
{
    vs_graphs::core::Map *pActiveMap = p_atlas->getCurrentMap();
    return pActiveMap->getAllFloors();
}

bool System::saveMap(const string &filename)
{
    saveAtlasFile = filename;
    if (!saveAtlasFile.empty())
    {
        Verbose::printMess("Atlas saving to file " + saveAtlasFile,
                           Verbose::VERBOSITY_NORMAL);
        return saveAtlas(FileType::BINARY_FILE);
    }
    return false;
}

bool System::saveMapPointsAsPCD(const string &filename)
{
    try
    {
        // make a pointcloud out of all map points
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        vector<MapPoint *> vpMPs = p_atlas->getCurrentMap()->getAllMapPoints();
        for (size_t i = 0; i < vpMPs.size(); i++)
        {
            MapPoint *pMP = vpMPs[i];
            if (pMP->isBad())
                continue;

            Eigen::Vector3d P3Dw = pMP->getWorldPos().cast<double>();
            pcl::PointXYZ   point;
            point.x = P3Dw.x();
            point.y = P3Dw.y();
            point.z = P3Dw.z();
            cloud->push_back(point);
        }

        // save the pointcloud
        pcl::io::savePCDFileBinary(filename + ".pcd", *cloud);

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return false;
    }
    catch (...)
    {
        std::cerr << "Unknows exeption" << std::endl;
        return false;
    }
}

} // namespace core
} // namespace vs_graphs

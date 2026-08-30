/**
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

#include "System.h"
#include "Converter.h"
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <iomanip>
#include <openssl/md5.h>
#include <pangolin/pangolin.h>
#include <thread>

namespace ORB_SLAM3
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
    mSensor(sensor),
    mpViewer(static_cast<Viewer *>(NULL)),
    mptGeometricSegmentation(static_cast<std::thread *>(NULL)),
    mbReset(false),
    mbResetActiveMap(false),
    mbActivateLocalizationMode(false),
    mbDeactivateLocalizationMode(false),
    mbShutDown(false)
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
    if (mSensor == MONOCULAR)
        std::cout << "Monocular" << std::endl;
    else if (mSensor == STEREO)
        std::cout << "Stereo" << std::endl;
    else if (mSensor == RGBD)
        std::cout << "RGB-D" << std::endl;
    else if (mSensor == IMU_MONOCULAR)
        std::cout << "Monocular-Inertial" << std::endl;
    else if (mSensor == IMU_STEREO)
        std::cout << "Stereo-Inertial" << std::endl;
    else if (mSensor == IMU_RGBD)
        std::cout << "RGB-D-Inertial" << std::endl;

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
        settings_             = new Settings(strSettingsFile, mSensor);
        mStrLoadAtlasFromFile = settings_->atlasLoadFile();
        mStrSaveAtlasToFile   = settings_->atlasSaveFile();
        std::cout << (*settings_) << std::endl;
    }
    else
    {
        settings_         = nullptr;
        cv::FileNode node = fsSettings["System.LoadAtlasFromFile"];
        if (!node.empty() && node.isString())
            mStrLoadAtlasFromFile = (string)node;

        node = fsSettings["System.SaveAtlasToFile"];
        if (!node.empty() && node.isString())
            mStrSaveAtlasToFile = (string)node;
    }

    if ((mSensor == RGBD || mSensor == IMU_RGBD) && settings_ != nullptr)
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

    mStrVocabularyFilePath = strVocFile;

    /* Init the ORB vocabulary */
    std::cout << "[System] Loading ORB Vocabulary ..." << std::endl;
    mpVocabulary  = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromBinFile(strVocFile);
    if (!bVocLoad)
    {
        cerr << "- Wrong path to vocabulary. " << endl;
        cerr << "- Failed to open at: " << strVocFile << endl;
        exit(-1);
    }

    /* Create keyframe database */
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    /* Init flag to indicate if a previous map is loaded */
    bool loadedAtlas;

    /* Check to see if there is a string to an Atlas map file to load */
    if (mStrLoadAtlasFromFile.empty())
    {
        /* If no file given, create a new Atlas map */
        mpAtlas = new Atlas(0);

        std::cout << "[System] Initializing Atlas from scratch in 'mpAtlas'"
                  << std::endl;

        /* Set flag to indcate that an Atlas map was not previously loaded */
        loadedAtlas = false;
    }
    else
    {
        /* If file given, load Atlas map from earlier session */
        bool isRead = LoadAtlas(FileType::BINARY_FILE);

        std::cout << "[System] Initializing Atlas from file: "
                  << mStrLoadAtlasFromFile << "... " << std::endl;

        if (!isRead)
        {
            std::cout << "[System] Error while loading Atlas file! Previous "
                         "Atlas file could not be loaded. Exiting ..."
                      << std::endl;
            exit(-1);
        }

        loadedAtlas = true;

        mpAtlas->CreateNewMap();
    }

    /* Load the system parameters */
    SystemParams *sysParams = SystemParams::GetParams();
    sysParams->SetParams(strSysParamsFile);

    /* Parse the environment database, if provided */
    parseJsonDatabase(sysParams->general.env_database);

    /* If the sensor is integrated with IMU, initialize the IMU first */
    if (mSensor == IMU_STEREO || mSensor == IMU_MONOCULAR ||
        mSensor == IMU_RGBD)
    {
        mpAtlas->SetInertialSensor();
    }

    /* ---------------------------------------------------------------------- *
     * FRAME + MAP + TRACKER OBJECTS
     * ---------------------------------------------------------------------- */

    /* Create Drawers. These are used by the Viewer */
    mpFrameDrawer = new FrameDrawer(mpAtlas);
    mpMapDrawer   = new MapDrawer(mpAtlas, strSettingsFile, settings_);

    /* Initialize the Tracking thread */
    mpTracker = new Tracking(this,
                             mpVocabulary,
                             mpFrameDrawer,
                             mpMapDrawer,
                             mpAtlas,
                             mpKeyFrameDatabase,
                             strSettingsFile,
                             mSensor,
                             settings_,
                             strSequence);

    /* Set the value of marker impact */
    mpTracker->SetMarkerImpact(sysParams->markers.impact);

    /* ---------------------------------------------------------------------- *
     * LOCAL MAPPTING THREAD
     * ---------------------------------------------------------------------- */

    /* Initialize the Local Mapping object */
    mpLocalMapper =
        new LocalMapping(this,
                         mpAtlas,
                         mSensor == MONOCULAR || mSensor == IMU_MONOCULAR,
                         mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
                             mSensor == IMU_RGBD,
                         strSequence);

    /* Set up thread to run the mpLocalMapper and call Run() method */
    mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run, mpLocalMapper);

    mpLocalMapper->mInitFr = initFr;
    if (settings_)
    {
        mpLocalMapper->mThFarPoints = settings_->thFarPoints();
    }
    else
    {
        mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];
    }

    if (mpLocalMapper->mThFarPoints != 0)
    {
        cout << "Discard points further than " << mpLocalMapper->mThFarPoints
             << " m from current camera" << endl;
        mpLocalMapper->mbFarPoints = true;
    }
    else
    {
        mpLocalMapper->mbFarPoints = false;
    }

    /* ---------------------------------------------------------------------- *
     * LOOP CLOSING THREAD
     * ---------------------------------------------------------------------- */

    /* Initialize the Loop Closing thread */
    mpLoopCloser = new LoopClosing(mpAtlas,
                                   mpKeyFrameDatabase,
                                   mpVocabulary,
                                   mSensor != MONOCULAR,
                                   activeLC);

    /* Launch the loop closing thread */
    mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

    /* ---------------------------------------------------------------------- *
     * SEMANTIC SEGMENTATION THREAD
     * ---------------------------------------------------------------------- */

    /* Initialize the Semantic Segmentation thread */
    mpSemanticSegmentation = new SemanticSegmentation(mpAtlas);

    /* Launch the Semantic Segmentation thread */
    mptSemanticSegmentation =
        new thread(&SemanticSegmentation::Run, mpSemanticSegmentation);

    /* ---------------------------------------------------------------------- *
     * SEMANTIC MANAGER THREAD
     * ---------------------------------------------------------------------- */

    /* Initialize the Semantic Manager thread */
    mpSemanticsManager = new SemanticsManager(mpAtlas);

    /* Launch the Semantic Manager thread */
    mptSemanticsManager =
        new thread(&SemanticsManager::Run, mpSemanticsManager);

    /* ---------------------------------------------------------------------- *
     * THREAD POINTER STORAGE
     * ---------------------------------------------------------------------- */

    /* Store loop closing and local mapper thread pointers in tracker object */
    mpTracker->SetLoopClosing(mpLoopCloser);
    mpTracker->SetLocalMapper(mpLocalMapper);

    /* Store tracking object and loop closing thread pointer in local mapper */
    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    /* Store tracking object and local mapper thread pointer in loop closer */
    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    /* If enabled, init the viewer */
    if (bUseViewer)
    {
        mpViewer  = new Viewer(this,
                              mpFrameDrawer,
                              mpMapDrawer,
                              mpTracker,
                              strSettingsFile,
                              settings_);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
        mpLoopCloser->mpViewer = mpViewer;
        mpViewer->both         = mpFrameDrawer->both;
    }

    /* Set verbosity level */
    Verbose::SetTh(verboseLevel);
}

System::~System()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbShutDown = true;
    }

    /* Request a graceful stop on every running worker thread. */
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    mpSemanticSegmentation->RequestFinish();
    mpSemanticsManager->RequestFinish();
    if (mpViewer != static_cast<Viewer *>(NULL))
    {
        mpViewer->RequestFinish();
    }

    /* Wait for each worker to report finished before joining. Shutdown() may
     * have already stopped Local Mapping / Loop Closing; isFinished() is
     * idempotent, join() below is the only join in the process. */
    while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() ||
           !mpSemanticSegmentation->isFinished() ||
           !mpSemanticsManager->isFinished() ||
           (mpViewer != static_cast<Viewer *>(NULL) && !mpViewer->isFinished()))
    {
        usleep(1000);
    }

    /* Join and free the thread objects (first and only join). */
    mptLocalMapping->join();
    mptLoopClosing->join();
    mptSemanticSegmentation->join();
    mptSemanticsManager->join();
    if (mpViewer != static_cast<Viewer *>(NULL))
    {
        mptViewer->join();
    }

    delete mptLocalMapping;
    delete mptLoopClosing;
    delete mptSemanticSegmentation;
    delete mptSemanticsManager;
    if (mpViewer != static_cast<Viewer *>(NULL))
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
    ORB_SLAM3::DBParser parser;
    // Load JSON file
    json                envData = parser.jsonParser(jsonFilePath);
    // Getting semantic entities
    envRooms = parser.getEnvRooms(envData);
    // Printing the success message
    std::cout << "- JSON loaded and candidates created!\n";
}

void System::addSegmentedImage(
    std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> *tuple)
{
    // Adding the segmented image to the buffer of the SemanticSegmentation
    if (SystemParams::GetParams()->general.mode_of_operation ==
        SystemParams::general::ModeOfOperation::GEO)
    {
        // just clear the pointcloud of the keyframe and return, as semantic
        // segmentation is not running
        ORB_SLAM3::KeyFrame *pKF =
            mpAtlas->GetKeyFrameById(std::get<0>(*tuple));
        if (pKF)
        {
            pKF->clearPointCloud();
        }
        return;
    }

    mpSemanticSegmentation->AddSegmentedFrameToBuffer(tuple);
}

std::vector<std::vector<Eigen::Vector3d>> System::getSkeletonCluster()
{
    return mpAtlas->GetSkeletoClusterPoints();
}

void System::setSkeletonCluster(const std::vector<std::vector<Eigen::Vector3d>>
                                    &skeletonClusterPoints_World_m_in)
{
    /* Keep asynchronous skeleton replacement atomic with map remerging. */
    std::unique_lock<std::mutex> semanticUpdateLock =
        mpAtlas->acquireSemanticUpdateLock();

    /* Add the skeleton cluster to the current semantic map. */
    mpAtlas->SetSkeletonClusterPoints(skeletonClusterPoints_World_m_in);
}

void System::setSkeletonEdges(
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        &skeletonEdges_World_m_in)
{
    /* Keep asynchronous skeleton replacement atomic with map remerging. */
    std::unique_lock<std::mutex> semanticUpdateLock =
        mpAtlas->acquireSemanticUpdateLock();

    /* Store the connected skeleton edges in the current semantic map. */
    mpAtlas->SetSkeletonEdges(skeletonEdges_World_m_in);
}

void System::setGNNRoomCandidates(
    const std::vector<ORB_SLAM3::Room *> &gnnRoomCandidates)
{
    // [TODO] Add the GNN room candidates to the SemanticsManager
}

Sophus::SE3f System::TrackStereo(const cv::Mat              &imLeft,
                                 const cv::Mat              &imRight,
                                 const double               &timestamp,
                                 const vector<IMU::Point>   &vImuMeas,
                                 string                      filename,
                                 const std::vector<Marker *> markers)
{
    if (mSensor != STEREO && mSensor != IMU_STEREO)
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
        if (mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while (!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if (mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    {
        unique_lock<mutex> lock(mMutexReset);
        if (mbReset)
        {
            mpTracker->Reset();
            mResetCount.fetch_add(1U, std::memory_order_relaxed);
            mbReset          = false;
            mbResetActiveMap = false;
        }
        else if (mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mResetCount.fetch_add(1U, std::memory_order_relaxed);
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_STEREO)
        for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    Sophus::SE3f Tcw = mpTracker->GrabImageStereo(imLeftToFeed,
                                                  imRightToFeed,
                                                  timestamp,
                                                  filename,
                                                  markers,
                                                  envRooms);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState           = mpTracker->mState;
    mTrackingInliers         = mpTracker->GetMatchesInliers();
    mLastFrameTimestamp      = timestamp;
    mTrackedMapPoints        = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn      = mpTracker->mCurrentFrame.mvKeysUn;
    mCurrentCameraPose_World = Tcw.inverse();
    mCurrentCameraPoseValid =
        mTrackingState == Tracking::OK &&
        mCurrentCameraPose_World.translation().allFinite() &&
        mCurrentCameraPose_World.rotationMatrix().allFinite();

    return Tcw;
}

Sophus::SE3f
    System::TrackRGBD(const cv::Mat                                &colorImg,
                      const cv::Mat                                &depthmap,
                      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &mainCloud,
                      const double                                 &timestamp,
                      const vector<IMU::Point>                     &vImuMeas,
                      string                                        filename,
                      const std::vector<Marker *>                   markers)
{
    // Check if the sensor is correctly set as RGB-D
    if (mSensor != RGBD && mSensor != IMU_RGBD)
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
        if (mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped
            while (!mpLocalMapper->isStopped())
                usleep(1000);
            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if (mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if (mbReset)
        {
            mpTracker->Reset();
            mResetCount.fetch_add(1U, std::memory_order_relaxed);
            mbReset          = false;
            mbResetActiveMap = false;
        }
        else if (mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mResetCount.fetch_add(1U, std::memory_order_relaxed);
            mbResetActiveMap = false;
        }
    }

    // Apply IMU measurements
    if (mSensor == System::IMU_RGBD)
        for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    // Track RGB-D images
    Sophus::SE3f Tcw = mpTracker->GrabImageRGBD(imToFeed,
                                                imDepthToFeed,
                                                mainCloud,
                                                timestamp,
                                                filename,
                                                markers,
                                                envRooms);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState      = mpTracker->mState;
    mTrackingInliers    = mpTracker->GetMatchesInliers();
    mLastFrameTimestamp = timestamp;
    mTrackedMapPoints   = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    mCurrentCameraPose_World = Tcw.inverse();
    mCurrentCameraPoseValid =
        mTrackingState == Tracking::OK &&
        mCurrentCameraPose_World.translation().allFinite() &&
        mCurrentCameraPose_World.rotationMatrix().allFinite();

    /* Detect map restart for room-context carryover (WP1).
     * The SemanticsManager::Run() thread performs the actual room
     * matching once rooms exist in the new map; we only log here. */
    {
        Map *currentMap = mpAtlas->GetCurrentMap();
        if (currentMap)
        {
            long unsigned int mapId = currentMap->GetId();
            if (mFirstMapInit)
            {
                mLastProcessedMapId = mapId;
                mFirstMapInit       = false;
            }
            else if (mapId != mLastProcessedMapId)
            {
                const long unsigned int previousMapId = mLastProcessedMapId;
                mLastProcessedMapId                   = mapId;
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

Sophus::SE3f System::TrackMonocular(const cv::Mat              &im,
                                    const double               &timestamp,
                                    const vector<IMU::Point>   &vImuMeas,
                                    string                      filename,
                                    const std::vector<Marker *> markers)
{
    // Multi-thread to prevent race conditions
    {
        unique_lock<mutex> lock(mMutexReset);
        if (mbShutDown)
            return Sophus::SE3f();
    }

    // Check if the sensor is Monocular
    if (mSensor != MONOCULAR && mSensor != IMU_MONOCULAR)
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
        if (mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while (!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if (mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if (mbReset)
        {
            mpTracker->Reset();
            mResetCount.fetch_add(1U, std::memory_order_relaxed);
            mbReset          = false;
            mbResetActiveMap = false;
        }
        else if (mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mResetCount.fetch_add(1U, std::memory_order_relaxed);
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
        for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    Sophus::SE3f Tcw = mpTracker->GrabImageMonocular(imToFeed,
                                                     timestamp,
                                                     filename,
                                                     markers,
                                                     envRooms);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState      = mpTracker->mState;
    mTrackedMapPoints   = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n    = 0;
    int        curn = mpAtlas->GetLastBigChangeIdx();
    if (n < curn)
    {
        n = curn;
        return true;
    }
    else
        return false;
}

System::MissionHealthSnapshot
    System::GetMissionHealthSnapshot(bool includeSemantics)
{
    MissionHealthSnapshot snapshot;
    snapshot.inertial = mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
                        mSensor == IMU_RGBD;

    {
        std::lock_guard<std::mutex> stateLock(mMutexState);
        snapshot.frameTimestamp   = mLastFrameTimestamp;
        snapshot.trackingState    = mTrackingState;
        snapshot.trackingInliers  = mTrackingInliers;
        snapshot.poseValid        = mCurrentCameraPoseValid;
        snapshot.cameraPose_World = mCurrentCameraPose_World;
    }

    std::unique_lock<std::mutex> semanticUpdateLock;
    if (includeSemantics)
    {
        semanticUpdateLock = mpAtlas->acquireSemanticUpdateLock();
    }
    Map *p_activeMap = mpAtlas->GetCurrentMap();
    snapshot.mapCount =
        static_cast<std::uint32_t>(std::max(0, mpAtlas->CountMaps()));
    snapshot.inertialInitialized =
        snapshot.inertial && mpAtlas->isImuInitialized();
    snapshot.resetCount = mResetCount.load(std::memory_order_relaxed);

    if (mpSemanticsManager != nullptr)
    {
        snapshot.currentRoomId = mpSemanticsManager->getCurrentRoomId();
        if (snapshot.trackingState == Tracking::LOST)
        {
            mpSemanticsManager->onTrackingLost();
        }
        snapshot.lastKnownRoomId = mpSemanticsManager->getLastKnownRoomId();
    }

    if (p_activeMap != nullptr)
    {
        snapshot.mapId = static_cast<std::uint64_t>(p_activeMap->GetId());
        const std::vector<KeyFrame *> keyFrames =
            p_activeMap->GetAllKeyFrames();
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
            snapshot.latestKeyFrameTimestamp = p_latestKeyFrame->mTimeStamp;
            snapshot.latestKeyFramePose_World =
                p_latestKeyFrame->GetPoseInverse();
            snapshot.latestKeyFramePoseValid =
                snapshot.latestKeyFramePose_World.translation().allFinite() &&
                snapshot.latestKeyFramePose_World.rotationMatrix().allFinite();
        }

        if (includeSemantics)
        {
            for (Room *p_room : p_activeMap->GetAllRooms())
            {
                if (p_room == nullptr || p_room->isBad())
                {
                    continue;
                }
                if (p_room->getRoomVariant() != Room::roomVariant::ROOM)
                {
                    ++snapshot.unresolvedRoomCount;
                    continue;
                }

                ++snapshot.confirmedRoomCount;
                RoomHealth room;
                room.id = p_room->getId();
                for (Passage *p_passage : p_room->getPassages())
                {
                    if (p_passage != nullptr)
                    {
                        room.passageIds.push_back(p_passage->getId());
                    }
                }
                std::sort(room.passageIds.begin(), room.passageIds.end());
                snapshot.rooms.push_back(std::move(room));
            }

            for (Floor *p_floor : p_activeMap->GetAllFloors())
            {
                if (p_floor == nullptr)
                {
                    continue;
                }
                FloorHealth floor;
                floor.id = p_floor->getId();
                for (Room *p_room : p_floor->getRooms())
                {
                    if (p_room != nullptr && !p_room->isBad() &&
                        p_room->getRoomVariant() == Room::roomVariant::ROOM)
                    {
                        floor.roomIds.push_back(p_room->getId());
                        ++snapshot.floorRoomLinkCount;
                    }
                }
                std::sort(floor.roomIds.begin(), floor.roomIds.end());
                snapshot.floors.push_back(std::move(floor));
            }

            for (Passage *p_passage : p_activeMap->GetAllPassages())
            {
                if (p_passage == nullptr)
                {
                    continue;
                }
                PassageHealth passage;
                passage.id       = p_passage->getId();
                passage.passable = p_passage->isPassable();
                passage.knownToFarCount =
                    p_passage->getTraversalKnownToFarCount();
                passage.farToKnownCount =
                    p_passage->getTraversalFarToKnownCount();
                passage.unknownCount = p_passage->getTraversalUnknownCount();
                const Passage::KnownSideProvenance knownSide =
                    p_passage->getKnownSideProvenance();
                if (knownSide.pRoom != nullptr)
                {
                    passage.knownSideRoomId = knownSide.pRoom->getId();
                }
                Room *p_farSideRoom = p_passage->getProspectiveRoom();
                if (p_farSideRoom != nullptr)
                {
                    passage.farSideRoomId = p_farSideRoom->getId();
                }
                snapshot.passages.push_back(passage);
            }
        }
    }

    if (semanticUpdateLock.owns_lock())
    {
        semanticUpdateLock.unlock();
    }
    if (mpLoopCloser != nullptr)
    {
        const LoopClosing::LoopCorrectionStatus loop =
            mpLoopCloser->GetLoopCorrectionStatus();
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

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::ResetActiveMap()
{
    unique_lock<mutex> lock(mMutexReset);
    mbResetActiveMap = true;
}

void System::Shutdown()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbShutDown = true;
    }

    cout << "Shutdown" << endl;

    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();

    /*
     * LoopClosing joins its GBA worker before reporting finished. Waiting here
     * prevents Atlas serialization from racing a final map correction.
     */
    while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished())
    {
        usleep(1000);
    }

    if (!mStrSaveAtlasToFile.empty())
    {
        Verbose::PrintMess("Atlas saving to file " + mStrSaveAtlasToFile,
                           Verbose::VERBOSITY_NORMAL);

        std::unique_lock<std::mutex> semanticUpdateLock =
            mpAtlas->acquireSemanticUpdateLock();
        SaveAtlas(FileType::BINARY_FILE);
    }

#ifdef REGISTER_TIMES
    mpTracker->PrintTimeStats();
#endif
}

bool System::isShutDown()
{
    unique_lock<mutex> lock(mMutexReset);
    return mbShutDown;
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl
         << "Saving camera trajectory to " << filename << " ..." << endl;
    if (mSensor == MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular."
             << endl;
        return;
    }

    vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is
    // optimized by BA and pose graph). We need to get first the keyframe pose
    // and then concatenate the relative transformation. Frames not localized
    // (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
    // and a flag which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame *>::iterator lRit =
        mpTracker->mlpReferences.begin();
    list<double>::iterator lT  = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator   lbL = mpTracker->mlbLost.begin();
    for (list<Sophus::SE3f>::iterator
             lit  = mpTracker->mlRelativeFramePoses.begin(),
             lend = mpTracker->mlRelativeFramePoses.end();
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
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Two;

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

void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl
         << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
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

        Sophus::SE3f       Twc = pKF->GetPoseInverse();
        Eigen::Quaternionf q   = Twc.unit_quaternion();
        Eigen::Vector3f    t   = Twc.translation();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " "
          << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y()
          << " " << q.z() << " " << q.w() << endl;
    }

    f.close();
}

void System::SaveTrajectoryEuRoC(const string &filename)
{

    cout << endl << "Saving trajectory to " << filename << " ..." << endl;

    vector<Map *> vpMaps      = mpAtlas->GetAllMaps();
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

        const std::size_t keyFrameCount = pMap->GetAllKeyFrames().size();

        std::cout << "  Map " << std::to_string(pMap->GetId()) << " has "
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

    vector<KeyFrame *> vpKFs = p_biggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f
        Twb; // Can be word to cam0 or world to b depending on IMU or not.
    if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
        mSensor == IMU_RGBD)
        Twb = vpKFs[0]->GetImuPose();
    else
        Twb = vpKFs[0]->GetPoseInverse();

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
    list<ORB_SLAM3::KeyFrame *>::iterator lRit =
        mpTracker->mlpReferences.begin();
    list<double>::iterator lT  = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator   lbL = mpTracker->mlbLost.begin();

    for (auto lit  = mpTracker->mlRelativeFramePoses.begin(),
              lend = mpTracker->mlRelativeFramePoses.end();
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
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        if (!pKF || pKF->GetMap() != p_biggerMap)
            continue;

        Trw = Trw * pKF->GetPose() *
              Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
            mSensor == IMU_RGBD)
        {
            Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
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

void System::SaveTrajectoryEuRoC(const string &filename, Map *pMap)
{

    cout << endl
         << "Saving trajectory of map " << pMap->GetId() << " to " << filename
         << " ..." << endl;

    int numMaxKFs = 0;

    vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f
        Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
    if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
        mSensor == IMU_RGBD)
        Twb = vpKFs[0]->GetImuPose();
    else
        Twb = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is
    // optimized by BA and pose graph). We need to get first the keyframe pose
    // and then concatenate the relative transformation. Frames not localized
    // (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
    // and a flag which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame *>::iterator lRit =
        mpTracker->mlpReferences.begin();
    list<double>::iterator lT  = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator   lbL = mpTracker->mlbLost.begin();

    for (auto lit  = mpTracker->mlRelativeFramePoses.begin(),
              lend = mpTracker->mlRelativeFramePoses.end();
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
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        if (!pKF || pKF->GetMap() != pMap)
            continue;

        Trw = Trw * pKF->GetPose() *
              Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
            mSensor == IMU_RGBD)
        {
            Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
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

void System::SaveKeyFrameTrajectoryEuRoC(const string &filename)
{
    cout << endl
         << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<Map *> vpMaps      = mpAtlas->GetAllMaps();
    Map          *p_biggerMap = nullptr;
    std::size_t   numMaxKFs   = 0;
    for (Map *pMap : vpMaps)
    {
        if (pMap && pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs   = pMap->GetAllKeyFrames().size();
            p_biggerMap = pMap;
        }
    }

    if (!p_biggerMap)
    {
        std::cout << "There is not a map!!" << std::endl;
        return;
    }

    vector<KeyFrame *> vpKFs = p_biggerMap->GetAllKeyFrames();
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
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
            mSensor == IMU_RGBD)
        {
            Sophus::SE3f       Twb = pKF->GetImuPose();
            Eigen::Quaternionf q   = Twb.unit_quaternion();
            Eigen::Vector3f    twb = Twb.translation();
            f << setprecision(6) << 1e9 * pKF->mTimeStamp << " "
              << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2)
              << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
              << endl;
        }
        else
        {
            Sophus::SE3f       Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q   = Twc.unit_quaternion();
            Eigen::Vector3f    t   = Twc.translation();
            f << setprecision(6) << 1e9 * pKF->mTimeStamp << " "
              << setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " "
              << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
    }
    f.close();
}

void System::SaveKeyFrameTrajectoryEuRoC(const string &filename, Map *pMap)
{
    cout << endl
         << "Saving keyframe trajectory of map " << pMap->GetId() << " to "
         << filename << " ..." << endl;

    vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
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
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
            mSensor == IMU_RGBD)
        {
            Sophus::SE3f       Twb = pKF->GetImuPose();
            Eigen::Quaternionf q   = Twb.unit_quaternion();
            Eigen::Vector3f    twb = Twb.translation();
            f << setprecision(6) << 1e9 * pKF->mTimeStamp << " "
              << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2)
              << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
              << endl;
        }
        else
        {
            Sophus::SE3f       Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q   = Twc.unit_quaternion();
            Eigen::Vector3f    t   = Twc.translation();
            f << setprecision(6) << 1e9 * pKF->mTimeStamp << " "
              << setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " "
              << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
    }
    f.close();
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl
         << "Saving camera trajectory to " << filename << " ..." << endl;
    if (mSensor == MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular."
             << endl;
        return;
    }

    vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Tow = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is
    // optimized by BA and pose graph). We need to get first the keyframe pose
    // and then concatenate the relative transformation. Frames not localized
    // (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
    // and a flag which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame *>::iterator lRit =
        mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for (list<Sophus::SE3f>::iterator
             lit  = mpTracker->mlRelativeFramePoses.begin(),
             lend = mpTracker->mlRelativeFramePoses.end();
         lit != lend;
         lit++, lRit++, lT++)
    {
        ORB_SLAM3::KeyFrame *pKF = *lRit;

        Sophus::SE3f Trw;

        if (!pKF)
            continue;

        while (pKF->isBad())
        {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Tow;

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

void System::SaveDebugData(const int &initIdx)
{
    // 0. Save initialization trajectory
    SaveTrajectoryEuRoC("init_FrameTrajectoy_" +
                        to_string(mpLocalMapper->mInitSect) + "_" +
                        to_string(initIdx) + ".txt");

    // 1. Save scale
    ofstream f;
    f.open("init_Scale_" + to_string(mpLocalMapper->mInitSect) + ".txt",
           ios_base::app);
    f << fixed;
    f << mpLocalMapper->mScale << endl;
    f.close();

    // 2. Save gravity direction
    f.open("init_GDir_" + to_string(mpLocalMapper->mInitSect) + ".txt",
           ios_base::app);
    f << fixed;
    f << mpLocalMapper->mRwg(0, 0) << "," << mpLocalMapper->mRwg(0, 1) << ","
      << mpLocalMapper->mRwg(0, 2) << endl;
    f << mpLocalMapper->mRwg(1, 0) << "," << mpLocalMapper->mRwg(1, 1) << ","
      << mpLocalMapper->mRwg(1, 2) << endl;
    f << mpLocalMapper->mRwg(2, 0) << "," << mpLocalMapper->mRwg(2, 1) << ","
      << mpLocalMapper->mRwg(2, 2) << endl;
    f.close();

    // 3. Save computational cost
    f.open("init_CompCost_" + to_string(mpLocalMapper->mInitSect) + ".txt",
           ios_base::app);
    f << fixed;
    f << mpLocalMapper->mCostTime << endl;
    f.close();

    // 4. Save biases
    f.open("init_Biases_" + to_string(mpLocalMapper->mInitSect) + ".txt",
           ios_base::app);
    f << fixed;
    f << mpLocalMapper->mbg(0) << "," << mpLocalMapper->mbg(1) << ","
      << mpLocalMapper->mbg(2) << endl;
    f << mpLocalMapper->mba(0) << "," << mpLocalMapper->mba(1) << ","
      << mpLocalMapper->mba(2) << endl;
    f.close();

    // 5. Save covariance matrix
    f.open("init_CovMatrix_" + to_string(mpLocalMapper->mInitSect) + "_" +
               to_string(initIdx) + ".txt",
           ios_base::app);
    f << fixed;
    for (int i = 0; i < mpLocalMapper->mcovInertial.rows(); i++)
    {
        for (int j = 0; j < mpLocalMapper->mcovInertial.cols(); j++)
        {
            if (j != 0)
                f << ",";
            f << setprecision(15) << mpLocalMapper->mcovInertial(i, j);
        }
        f << endl;
    }
    f.close();

    // 6. Save initialization time
    f.open("init_Time_" + to_string(mpLocalMapper->mInitSect) + ".txt",
           ios_base::app);
    f << fixed;
    f << mpLocalMapper->mInitTime << endl;
    f.close();
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint *> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

cv::Mat System::GetCurrentFrame()
{
    return mpFrameDrawer->DrawFrame();
}

std::vector<KeyFrame *> System::GetAllKeyFrames()
{
    return mpAtlas->GetAllKeyFrames();
}

Sophus::SE3f System::GetCamTwc()
{
    return mpTracker->GetCamTwc();
}

Sophus::SE3f System::GetImuTwb()
{
    return mpTracker->GetImuTwb();
}

Eigen::Vector3f System::GetImuVwb()
{
    return mpTracker->GetImuVwb();
}

bool System::isImuPreintegrated()
{
    return mpTracker->isImuPreintegrated();
}

double System::GetTimeFromIMUInit()
{
    double aux = mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
    if ((aux > 0.) && mpAtlas->isImuInitialized())
        return mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
    else
        return 0.f;
}

bool System::isLost()
{
    if (!mpAtlas->isImuInitialized())
        return false;
    else
    {
        if ((mpTracker->mState ==
             Tracking::LOST)) //||(mpTracker->mState==Tracking::RECENTLY_LOST))
            return true;
        else
            return false;
    }
}

bool System::isFinished()
{
    return (GetTimeFromIMUInit() > 0.1);
}

void System::ChangeDataset()
{
    if (mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12)
    {
        mpTracker->ResetActiveMap();
        mResetCount.fetch_add(1U, std::memory_order_relaxed);
    }
    else
    {
        mpTracker->CreateMapInAtlas();
    }

    mpTracker->NewDataset();
}

float System::GetImageScale()
{
    return mpTracker->GetImageScale();
}

#ifdef REGISTER_TIMES
void System::InsertRectTime(double &time)
{
    mpTracker->vdRectStereo_ms.push_back(time);
}

void System::InsertResizeTime(double &time)
{
    mpTracker->vdResizeImage_ms.push_back(time);
}

void System::InsertTrackTime(double &time)
{
    mpTracker->vdTrackTotal_ms.push_back(time);
}
#endif

bool System::SaveAtlas(int type)
{
    try
    {
        if (!mStrSaveAtlasToFile.empty())
        {
            // Save the current session
            mpAtlas->PreSave();

            string pathSaveFileName = "./";
            pathSaveFileName = pathSaveFileName.append(mStrSaveAtlasToFile);
            pathSaveFileName = pathSaveFileName.append(".osa");

            string strVocabularyChecksum =
                CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);
            std::size_t found = mStrVocabularyFilePath.find_last_of("/\\");
            string strVocabularyName = mStrVocabularyFilePath.substr(found + 1);

            if (type == TEXT_FILE) // File text
            {
                cout << "Starting to write the save text file to "
                     << pathSaveFileName.c_str() << endl;
                std::remove(pathSaveFileName.c_str());
                std::ofstream ofs(pathSaveFileName, std::ios::binary);
                boost::archive::text_oarchive oa(ofs);

                oa << strVocabularyName;
                oa << strVocabularyChecksum;
                oa << mpAtlas;
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
                oa << mpAtlas;
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

bool System::LoadAtlas(int type)
{
    string strFileVoc, strVocChecksum;
    bool   isRead = false;

    string pathLoadFileName = "./";
    pathLoadFileName        = pathLoadFileName.append(mStrLoadAtlasFromFile);
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
        ia >> mpAtlas;
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
        ia >> mpAtlas;
        cout << "End to load the save binary file" << endl;
        isRead = true;
    }

    if (isRead)
    {
        // Check if the vocabulary is the same
        string strInputVocabularyChecksum =
            CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);

        if (strInputVocabularyChecksum.compare(strVocChecksum) != 0)
        {
            cout << "The vocabulary load isn't the same which the load session "
                    "was created "
                 << endl;
            cout << "-Vocabulary name: " << strFileVoc << endl;
            return false; // Both are differents
        }

        mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
        mpAtlas->SetORBVocabulary(mpVocabulary);
        mpAtlas->PostLoad();

        return true;
    }
    return false;
}

string System::CalculateCheckSum(string filename, int type)
{
    string checksum = "";

    unsigned char c[MD5_DIGEST_LENGTH];

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

    MD5_CTX md5Context;
    char    buffer[1024];

    MD5_Init(&md5Context);
    while (int count = f.readsome(buffer, sizeof(buffer)))
    {
        MD5_Update(&md5Context, buffer, count);
    }

    f.close();

    MD5_Final(c, &md5Context);

    for (int i = 0; i < MD5_DIGEST_LENGTH; i++)
    {
        char aux[10];
        sprintf(aux, "%02x", c[i]);
        checksum = checksum + aux;
    }

    return checksum;
}

ORB_SLAM3::Map *System::GetCurrentMap()
{
    ORB_SLAM3::Map *pActiveMap = mpAtlas->GetCurrentMap();
    return pActiveMap;
}

vector<MapPoint *> System::GetAllMapPoints()
{
    Map *pActiveMap = mpAtlas->GetCurrentMap();
    return pActiveMap->GetAllMapPoints();
}

vector<Sophus::SE3f> System::GetAllKeyframePoses()
{
    vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
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
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
            mSensor == IMU_RGBD) // with IMU
            Twb = vpKFs[i]->GetImuPose();
        else // without IMU
            Twb = vpKFs[i]->GetPoseInverse();

        vKFposes.push_back(Twb);
    }

    return vKFposes;
}

Sophus::SE3f System::GetKeyFramePose(KeyFrame *pKF)
{
    if (pKF->isBad())
        return Sophus::SE3f();

    // Twb can be world frame to cam0 frame (without IMU) or body in world frame
    // (with IMU)
    Sophus::SE3f Twb;
    if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
        mSensor == IMU_RGBD) // with IMU
        Twb = pKF->GetImuPose();
    else // without IMU
        Twb = pKF->GetPoseInverse();

    return Twb;
}

vector<Marker *> System::GetAllMarkers()
{
    Map *pActiveMap = mpAtlas->GetCurrentMap();
    return pActiveMap->GetAllMarkers();
}

std::vector<ORB_SLAM3::Passage *> System::GetAllPassages()
{
    Map *pActiveMap = mpAtlas->GetCurrentMap();
    return pActiveMap->GetAllPassages();
}

vector<Plane *> System::GetAllPlanes()
{
    Map *pActiveMap = mpAtlas->GetCurrentMap();
    return pActiveMap->GetAllPlanes();
}

vector<Room *> System::GetAllRooms()
{
    Map *pActiveMap = mpAtlas->GetCurrentMap();
    return pActiveMap->GetAllRooms();
}

std::vector<ORB_SLAM3::Door *> System::GetAllDoors()
{
    ORB_SLAM3::Map *pActiveMap = mpAtlas->GetCurrentMap();
    return pActiveMap->GetAllDoors();
}

std::vector<ORB_SLAM3::Floor *> System::GetAllFloors()
{
    ORB_SLAM3::Map *pActiveMap = mpAtlas->GetCurrentMap();
    return pActiveMap->GetAllFloors();
}

bool System::SaveMap(const string &filename)
{
    mStrSaveAtlasToFile = filename;
    if (!mStrSaveAtlasToFile.empty())
    {
        Verbose::PrintMess("Atlas saving to file " + mStrSaveAtlasToFile,
                           Verbose::VERBOSITY_NORMAL);
        return SaveAtlas(FileType::BINARY_FILE);
    }
    return false;
}

bool System::SaveMapPointsAsPCD(const string &filename)
{
    try
    {
        // make a pointcloud out of all map points
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        vector<MapPoint *> vpMPs = mpAtlas->GetCurrentMap()->GetAllMapPoints();
        for (size_t i = 0; i < vpMPs.size(); i++)
        {
            MapPoint *pMP = vpMPs[i];
            if (pMP->isBad())
                continue;

            Eigen::Vector3d P3Dw = pMP->GetWorldPos().cast<double>();
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

} // namespace ORB_SLAM3

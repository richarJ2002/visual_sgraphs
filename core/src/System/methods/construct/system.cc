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
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

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

} // namespace core
} // namespace vs_graphs

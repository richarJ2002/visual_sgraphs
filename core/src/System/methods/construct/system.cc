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
#include "FrameDrawer.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "SemanticSegmentation.h"
#include "SemanticsManager.h"
#include "Tracking.h"
#include "Viewer.h"

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
#include <rclcpp/logging.hpp>
#include <thread>

namespace vs_graphs
{
namespace core
{

System::System(const string                 &vocabularyFile_in,
               const string                 &settingsFile_in,
               const string                 &sysParamsFile_in,
               const SensorType              sensor_in,
               const bool                    shouldUseViewer_in,
               const int                     initialFr_in,
               const string                 &sequence_in,
               const Verbose::VerbosityLevel verboseLevel_in) :
    sensor(sensor_in),
    p_viewer(static_cast<Viewer *>(nullptr)),
    p_geometricSegmentationThread(static_cast<std::thread *>(nullptr)),
    isResetRequested(false),
    isResetActiveMapRequested(false),
    isLocalizationModeActivationRequested(false),
    isLocalizationModeDeactivationRequested(false),
    isShutdownRequested(false)
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
    if (sensor_in == MONOCULAR)
    {
        std::cout << "Monocular" << std::endl;
    }
    else if (sensor_in == STEREO)
    {
        std::cout << "Stereo" << std::endl;
    }
    else if (sensor_in == RGBD)
    {
        std::cout << "RGB-D" << std::endl;
    }
    else if (sensor_in == IMU_MONOCULAR)
    {
        std::cout << "Monocular-Inertial" << std::endl;
    }
    else if (sensor_in == IMU_STEREO)
    {
        std::cout << "Stereo-Inertial" << std::endl;
    }
    else if (sensor_in == IMU_RGBD)
    {
        std::cout << "RGB-D-Inertial" << std::endl;
    }

    /* Check settings file can be opened */
    cv::FileStorage fsSettings(settingsFile_in.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "[System] Failed to open settings file at '"
                  << settingsFile_in << "'! Exiting ..." << std::endl;
        exit(-1);
    }

    cv::FileNode node = fsSettings["File.version"];
    if (!node.empty() && node.isString() && node.string() == "1.0")
    {
        p_settings = new utils::settings::Settings(settingsFile_in, sensor_in);
        std::string settingsAtlasLoadFile{};
        if (p_settings->atlasLoadFile(settingsAtlasLoadFile) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: atlasLoadFile returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        loadAtlasFile = settingsAtlasLoadFile;
        std::string settingsAtlasSaveFile{};
        if (p_settings->atlasSaveFile(settingsAtlasSaveFile) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: atlasSaveFile returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        saveAtlasFile = settingsAtlasSaveFile;
        std::cout << (*p_settings) << std::endl;
    }
    else
    {
        p_settings        = nullptr;
        cv::FileNode node = fsSettings["System.LoadAtlasFromFile"];
        if (!node.empty() && node.isString())
            loadAtlasFile = (string)node;

        node = fsSettings["System.SaveAtlasToFile"];
        if (!node.empty() && node.isString())
            saveAtlasFile = (string)node;
    }

    if ((sensor_in == RGBD || sensor_in == IMU_RGBD) && p_settings != nullptr)
    {
        double stereoDepthThreshold{};
        if (p_settings->thDepth(stereoDepthThreshold) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: thDepth returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        double settingsB{};
        if (p_settings->b(settingsB) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: b returned a failure status although it cannot "
                         "fail; continuing as before.",
                         __func__);
        }
        const double metricCloseDepth_m = settingsB * stereoDepthThreshold;
        std::cout << "Stereo.ThDepth=" << stereoDepthThreshold
                  << " closeDepthMeters=" << metricCloseDepth_m << std::endl;
    }

    node          = fsSettings["loopClosing"];
    bool activeLc = true;
    if (!node.empty())
    {
        activeLc = static_cast<int>(fsSettings["loopClosing"]) != 0;
    }

    vocabularyFilePath = vocabularyFile_in;

    /* Init the ORB vocabulary */
    std::cout << "[System] Loading ORB Vocabulary ..." << std::endl;
    p_vocabulary            = new ORBVocabulary();
    bool isVocabularyLoaded = p_vocabulary->loadFromBinFile(vocabularyFile_in);
    if (!isVocabularyLoaded)
    {
        cerr << "- Wrong path to vocabulary. " << endl;
        cerr << "- Failed to open at: " << vocabularyFile_in << endl;
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
    types::SystemParams *p_sysParams = nullptr;
    if (types::SystemParams::getParams(p_sysParams) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_sysParams->setParams(sysParamsFile_in) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Parse the environment database, if provided */
    parseJsonDatabase(p_sysParams->general.envDatabase);

    /* If the sensor is integrated with IMU, initialize the IMU first */
    if (sensor_in == IMU_STEREO || sensor_in == IMU_MONOCULAR ||
        sensor_in == IMU_RGBD)
    {
        p_atlas->setInertialSensor();
    }

    /* ---------------------------------------------------------------------- *
     * FRAME + MAP + TRACKER OBJECTS
     * ---------------------------------------------------------------------- */

    /* Create Drawers. These are used by the Viewer */
    p_frameDrawer = new FrameDrawer(p_atlas);
    p_mapDrawer   = new MapDrawer(p_atlas, settingsFile_in, p_settings);

    /* Initialize the Tracking thread */
    p_tracker = new Tracking(this,
                             p_vocabulary,
                             p_frameDrawer,
                             p_mapDrawer,
                             p_atlas,
                             p_keyFrameDatabase,
                             settingsFile_in,
                             sensor_in,
                             p_settings,
                             sequence_in);

    /* Set the value of marker impact */
    p_tracker->setMarkerImpact(p_sysParams->markers.impact);

    /* ---------------------------------------------------------------------- *
     * LOCAL MAPPING THREAD
     * ---------------------------------------------------------------------- */

    /* Initialize the Local Mapping object */
    p_localMapper =
        new LocalMapping(this,
                         p_atlas,
                         sensor_in == MONOCULAR || sensor_in == IMU_MONOCULAR,
                         sensor_in == IMU_MONOCULAR ||
                             sensor_in == IMU_STEREO || sensor_in == IMU_RGBD,
                         sequence_in);

    /* Set up thread to run the mpLocalMapper and call Run() method */
    p_localMappingThread =
        new thread(&vs_graphs::core::LocalMapping::run, p_localMapper);

    p_localMapper->initFrame = initialFr_in;
    if (p_settings)
    {
        double settingsThFarPoints{};
        if (p_settings->thFarPoints(settingsThFarPoints) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: thFarPoints returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        p_localMapper->farPointsThreshold = settingsThFarPoints;
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
        p_localMapper->shouldSkipFarPoints = true;
    }
    else
    {
        p_localMapper->shouldSkipFarPoints = false;
    }

    /* ---------------------------------------------------------------------- *
     * LOOP CLOSING THREAD
     * ---------------------------------------------------------------------- */

    /* Initialize the Loop Closing thread */
    p_loopCloser = new LoopClosing(p_atlas,
                                   p_keyFrameDatabase,
                                   p_vocabulary,
                                   sensor_in != MONOCULAR,
                                   activeLc);

    /* Launch the loop closing thread */
    p_loopClosingThread =
        new thread(&vs_graphs::core::LoopClosing::run, p_loopCloser);

    /* ---------------------------------------------------------------------- *
     * SEMANTIC SEGMENTATION THREAD
     * ---------------------------------------------------------------------- */

    /* Initialize the Semantic Segmentation thread */
    p_semanticSegmentation = new SemanticSegmentation(p_atlas);

    /* Launch the Semantic Segmentation thread */
    p_semanticSegmentationThread =
        new thread(&SemanticSegmentation::run, p_semanticSegmentation);

    /* ---------------------------------------------------------------------- *
     * SEMANTIC MANAGER THREAD
     * ---------------------------------------------------------------------- */

    /* Initialize the Semantic Manager thread */
    p_semanticsManager = new SemanticsManager(p_atlas);

    /* Launch the Semantic Manager thread */
    p_semanticsManagerThread =
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
    if (shouldUseViewer_in)
    {
        p_viewer       = new Viewer(this,
                              p_frameDrawer,
                              p_mapDrawer,
                              p_tracker,
                              settingsFile_in,
                              p_settings);
        p_viewerThread = new thread(&Viewer::run, p_viewer);
        p_tracker->setViewer(p_viewer);
        p_loopCloser->p_viewer         = p_viewer;
        p_viewer->shouldDrawBothImages = p_frameDrawer->shouldDrawBothImages;
    }

    /* Set verbosity level */
    Verbose::setTh(verboseLevel_in);
}

} // namespace core
} // namespace vs_graphs

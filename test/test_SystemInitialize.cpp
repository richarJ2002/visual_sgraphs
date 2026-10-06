/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            test_GeometricToolsStatus.cpp
 *
 * @brief           Statuses of GeometricTools::triangulate: a point at a
 *                  finite depth succeeds; a solution at infinity (zero
 *                  homogeneous scale) is a numerical failure that leaves the
 *                  output unchanged.
 */

/*!
 * @file            test_SystemInitialize.cpp
 *
 * @brief           Checks that System::initialize() reports an unreadable
 *                  settings or vocabulary file as a status (it used to end the
 *                  process with exit(-1)), that a system that never
 *                  initialised can be destroyed safely, that the tracker's
 *                  parameter-file reader rejects an unknown camera type, and
 *                  that both camera readers accept Camera.type
 *                  "KannalaBrandt8".
 */

#include "Atlas.h"
#include "CameraModels/GeometricCamera/objects/GeometricCamera.h"
#include "System.h"
#include "Tracking.h"
#include "Utils/Settings/objects/Settings.h"

#include <cstdio>
#include <fstream>
#include <gtest/gtest.h>
#include <opencv2/core/persistence.hpp>
#include <string>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace
{

/*!
 * @brief           ORB extractor section of every settings file below; few
 *                  features and levels keep the extractors small.
 */
const std::string ORB_EXTRACTOR_SETTINGS =
    "ORBextractor.nFeatures: 100\nORBextractor.scaleFactor: 1.2\n"
    "ORBextractor.nLevels: 2\nORBextractor.iniThFAST: 20\n"
    "ORBextractor.minThFAST: 7\n";

/*!
 * @brief           Settings file written for one test and removed when the test
 *                  ends. By default it opens but holds no camera, so a System
 *                  run stops at the vocabulary step.
 */
class ReadableSettingsFile
{
  public:
    ReadableSettingsFile() :
        ReadableSettingsFile("system_initialize_settings.yaml",
                             "%YAML:1.0\n---\nunused: 1\n")
    {}
    /*!
     * @brief           Writes the given text to a file in the test directory.
     *
     * @param[in]       fileName_in
     *                  Name of the file inside the test directory.
     *
     * @param[in]       fileContent_in
     *                  Text written to the file.
     */
    ReadableSettingsFile(const std::string &fileName_in,
                         const std::string &fileContent_in) :
        path(testing::TempDir() + fileName_in)
    {
        std::ofstream file(path);
        file << fileContent_in;
    }
    ~ReadableSettingsFile()
    {
        std::remove(path.c_str());
    }
    /*!
     * @brief           Not copyable: each copy would remove the same file.
     */
    ReadableSettingsFile(const ReadableSettingsFile &otherFile_in) = delete;
    ReadableSettingsFile &
        operator=(const ReadableSettingsFile &otherFile_in) = delete;
    const std::string path;
};

} // namespace

/*!
 * @brief           Checks that initializing with a settings file that does not
 *                  exist returns the settings-unreadable status.
 */
TEST(SystemInitialize, ReportsAnUnreadableSettingsFile)
{
    System system;
    EXPECT_EQ(system.initialize("/nonexistent/vocabulary.bin",
                                "/nonexistent/settings.yaml",
                                "/nonexistent/system_params.yaml",
                                System::RGBD,
                                false),
              SystemStatus::SYSTEM_STATUS_SETTINGS_UNREADABLE);
}

/*!
 * @brief           Checks that initializing with a readable settings file but a
 *                  missing vocabulary file returns the vocabulary-unreadable
 *                  status.
 */
TEST(SystemInitialize, ReportsAnUnreadableVocabularyFile)
{
    const ReadableSettingsFile settings;
    System                     system;
    EXPECT_EQ(system.initialize("/nonexistent/vocabulary.bin",
                                settings.path,
                                "/nonexistent/system_params.yaml",
                                System::RGBD,
                                false),
              SystemStatus::SYSTEM_STATUS_VOCABULARY_UNREADABLE);
}

/*!
 * @brief           Checks that a System that was never initialized can be
 *                  destroyed without waiting for or joining any worker thread.
 */
TEST(SystemInitialize, DestroysASystemThatNeverInitialised)
{
    /* No thread was started, so the destructor must not wait for or join
     * any worker. Reaching the end of the scope is the check. */
    {
        System system;
    }
    SUCCEED();
}

/*!
 * @brief           Checks that the tracker's parameter-file camera reader
 *                  reports an unknown Camera.type as not parsed and names it,
 *                  instead of reporting success without building a camera.
 */
TEST(TrackingCameraParameters, ReportsAnUnknownCameraType)
{
    const ReadableSettingsFile monocularSettings(
        "tracking_monocular_settings.yaml",
        "%YAML:1.0\n---\n"
        "Camera.type: \"PinHole\"\n"
        "Camera.fx: 500.0\nCamera.fy: 500.0\n"
        "Camera.cx: 320.0\nCamera.cy: 240.0\n"
        "Camera.k1: 0.0\nCamera.k2: 0.0\nCamera.p1: 0.0\nCamera.p2: 0.0\n"
        "Camera.fps: 30.0\nCamera.RGB: 1\n" +
            ORB_EXTRACTOR_SETTINGS);
    const ReadableSettingsFile unknownCameraSettings(
        "tracking_unknown_camera_settings.yaml",
        "%YAML:1.0\n---\nCamera.type: \"NoSuchModel\"\n");

    /* A pinhole monocular file needs no vocabulary, drawers or database. */
    System   uninitialisedSystem;
    Atlas    trackerAtlas(0);
    Tracking parameterFileTracker(&uninitialisedSystem,
                                  nullptr,
                                  nullptr,
                                  nullptr,
                                  &trackerAtlas,
                                  nullptr,
                                  monocularSettings.path,
                                  System::MONOCULAR,
                                  nullptr);

    cv::FileStorage unknownCameraFile(unknownCameraSettings.path,
                                      cv::FileStorage::READ);
    ASSERT_TRUE(unknownCameraFile.isOpened());
    bool isParsed = true;
    testing::internal::CaptureStderr();
    const TrackingStatus parseStatus =
        parameterFileTracker.parseCamParamFile(unknownCameraFile, isParsed);
    const std::string errorOutput = testing::internal::GetCapturedStderr();
    EXPECT_EQ(parseStatus, TrackingStatus::TRACKING_STATUS_SUCCESS);
    EXPECT_FALSE(isParsed);
    EXPECT_NE(errorOutput.find("NoSuchModel"), std::string::npos);
}

/*!
 * @brief           Checks that the tracker's parameter-file camera reader
 *                  accepts Camera.type "KannalaBrandt8", the spelling the
 *                  fisheye configuration files use, and builds a fisheye
 *                  camera.
 */
TEST(TrackingCameraParameters, BuildsAKannalaBrandt8Camera)
{
    const ReadableSettingsFile fisheyeSettings(
        "tracking_fisheye_settings.yaml",
        "%YAML:1.0\n---\n"
        "Camera.type: \"KannalaBrandt8\"\n"
        "Camera.fx: 190.0\nCamera.fy: 190.0\n"
        "Camera.cx: 254.0\nCamera.cy: 256.0\n"
        "Camera.k1: 0.0\nCamera.k2: 0.0\nCamera.k3: 0.0\nCamera.k4: 0.0\n"
        "Camera.fps: 30.0\nCamera.RGB: 1\n" +
            ORB_EXTRACTOR_SETTINGS);

    /* A monocular file needs no vocabulary, drawers or database. */
    System   uninitialisedSystem;
    Atlas    trackerAtlas(0);
    Tracking fisheyeTracker(&uninitialisedSystem,
                            nullptr,
                            nullptr,
                            nullptr,
                            &trackerAtlas,
                            nullptr,
                            fisheyeSettings.path,
                            System::MONOCULAR,
                            nullptr);

    std::vector<camera_models::geometriccamera::GeometricCamera *>
        atlasCameras{};
    ASSERT_EQ(trackerAtlas.getAllCameras(atlasCameras),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ(atlasCameras.size(), 1U);
    unsigned int cameraModelType{};
    ASSERT_EQ(atlasCameras.front()->getType(cameraModelType),
              camera_models::geometriccamera::GeometricCameraStatus::
                  GEOMETRIC_CAMERA_STATUS_SUCCESS);
    /* == reads CAM_FISHEYE by value; it has no out-of-class definition. */
    EXPECT_TRUE(cameraModelType ==
                camera_models::geometriccamera::GeometricCamera::CAM_FISHEYE);
}

/*!
 * @brief           Checks that the settings reader accepts Camera.type
 *                  "KannalaBrandt8", the spelling the fisheye configuration
 *                  files use, and builds a Kannala-Brandt camera.
 */
TEST(SettingsCamera, BuildsAKannalaBrandt8Camera)
{
    const ReadableSettingsFile fisheyeSettings(
        "settings_fisheye_camera.yaml",
        "%YAML:1.0\n---\n"
        "Camera.type: \"KannalaBrandt8\"\n"
        "Camera1.fx: 190.0\nCamera1.fy: 190.0\n"
        "Camera1.cx: 254.0\nCamera1.cy: 256.0\n"
        "Camera1.k1: 0.0\nCamera1.k2: 0.0\n"
        "Camera1.k3: 0.0\nCamera1.k4: 0.0\n"
        "Camera.width: 512\nCamera.height: 512\n"
        "Camera.fps: 30\nCamera.RGB: 1\n" +
            ORB_EXTRACTOR_SETTINGS +
            "Viewer.KeyFrameSize: 0.05\nViewer.KeyFrameLineWidth: 1.0\n"
            "Viewer.GraphLineWidth: 0.9\nViewer.PointSize: 2.0\n"
            "Viewer.CameraSize: 0.08\nViewer.CameraLineWidth: 3.0\n"
            "Viewer.ViewpointX: 0.0\nViewer.ViewpointY: -0.7\n"
            "Viewer.ViewpointZ: -3.5\nViewer.ViewpointF: 500.0\n");

    utils::settings::Settings fisheyeCameraSettings(fisheyeSettings.path,
                                                    System::MONOCULAR);

    utils::settings::Settings::CameraType cameraType{};
    ASSERT_EQ(fisheyeCameraSettings.cameraType(cameraType),
              utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS);
    EXPECT_EQ(cameraType,
              utils::settings::Settings::CameraType::KANNALA_BRANDT);
    camera_models::geometriccamera::GeometricCamera *p_firstCamera = nullptr;
    ASSERT_EQ(fisheyeCameraSettings.camera1(p_firstCamera),
              utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS);
    ASSERT_NE(p_firstCamera, nullptr);
    unsigned int cameraModelType{};
    ASSERT_EQ(p_firstCamera->getType(cameraModelType),
              camera_models::geometriccamera::GeometricCameraStatus::
                  GEOMETRIC_CAMERA_STATUS_SUCCESS);
    /* == reads CAM_FISHEYE by value; it has no out-of-class definition. */
    EXPECT_TRUE(cameraModelType ==
                camera_models::geometriccamera::GeometricCamera::CAM_FISHEYE);
}

} // namespace core
} // namespace vs_graphs

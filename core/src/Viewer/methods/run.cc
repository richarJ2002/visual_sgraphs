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
 * @file            run.cc
 *
 * @brief           Implements Viewer::run(), declared in Viewer.h.
 */

#include "FrameDrawer.h"
#include "ResetCause.h"
#include "System.h"
#include "Tracking.h"
#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void Viewer::run()
{
    hasFinished = false;
    hasStopped  = false;

    pangolin::CreateWindowAndBind("ORB-SLAM3: Map Viewer", 1024, 768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,
                                            1.0,
                                            0.0,
                                            pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", false, true);
    pangolin::Var<bool> menuCameraView("menu.Camera View", false, false);
    pangolin::Var<bool> menuTopView("menu.Top View", false, false);
    // pangolin::Var<bool> menuSideView("menu.Side View",false,false);
    pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph", false, true);
    pangolin::Var<bool> menuShowInertialGraph("menu.Show Inertial Graph",
                                              true,
                                              true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",
                                             false,
                                             true);
    pangolin::Var<bool> menuReset("menu.Reset", false, false);
    pangolin::Var<bool> menuStop("menu.Stop", false, false);
    pangolin::Var<bool> menuStepByStep("menu.Step By Step",
                                       false,
                                       true); // false, true
    pangolin::Var<bool> menuStep("menu.Step", false, false);

    pangolin::Var<bool> menuShowOptLba("menu.Show LBA opt", false, true);
    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState camera(pangolin::ProjectionMatrix(1024,
                                                                  768,
                                                                  viewpointF,
                                                                  viewpointF,
                                                                  512,
                                                                  389,
                                                                  0.1,
                                                                  1000),
                                       pangolin::ModelViewLookAt(viewpointX,
                                                                 viewpointY,
                                                                 viewpointZ,
                                                                 0,
                                                                 0,
                                                                 0,
                                                                 0.0,
                                                                 -1.0,
                                                                 0.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View &cameraView =
        pangolin::CreateDisplay()
            .SetBounds(0.0,
                       1.0,
                       pangolin::Attach::Pix(175),
                       1.0,
                       -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(camera));

    pangolin::OpenGlMatrix poseCameraToWorld, Twr;
    poseCameraToWorld.SetIdentity();
    pangolin::OpenGlMatrix cameraCenter_World; // Oriented with g in the z axis
    cameraCenter_World.SetIdentity();
    cv::namedWindow("ORB-SLAM3: Current Frame");

    bool isFollowing        = true;
    bool isLocalizationMode = false;
    bool stepByStep         = false;
    bool isCameraView       = true;

    if (p_tracker->sensor == p_system->MONOCULAR ||
        p_tracker->sensor == p_system->STEREO ||
        p_tracker->sensor == p_system->RGBD)
    {
        menuShowGraph = true;
    }

    float trackedImageScale{};
    if (p_tracker->getImageScale(trackedImageScale) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getImageScale returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    std::cout << "Starting the Viewer" << std::endl;
    while (1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (p_mapDrawer->getCurrentOpenGLCameraMatrix(poseCameraToWorld,
                                                      cameraCenter_World) !=
            MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getCurrentOpenGLCameraMatrix returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        if (isTrackingStopRequested)
        {
            menuStepByStep          = true;
            isTrackingStopRequested = false;
        }

        if (menuFollowCamera && isFollowing)
        {
            if (isCameraView)
                camera.Follow(poseCameraToWorld);
            else
                camera.Follow(cameraCenter_World);
        }
        else if (menuFollowCamera && !isFollowing)
        {
            if (isCameraView)
            {
                camera.SetProjectionMatrix(
                    pangolin::ProjectionMatrix(1024,
                                               768,
                                               viewpointF,
                                               viewpointF,
                                               512,
                                               389,
                                               0.1,
                                               1000));
                camera.SetModelViewMatrix(pangolin::ModelViewLookAt(viewpointX,
                                                                    viewpointY,
                                                                    viewpointZ,
                                                                    0,
                                                                    0,
                                                                    0,
                                                                    0.0,
                                                                    -1.0,
                                                                    0.0));
                camera.Follow(poseCameraToWorld);
            }
            else
            {
                camera.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,
                                                                      768,
                                                                      3000,
                                                                      3000,
                                                                      512,
                                                                      389,
                                                                      0.1,
                                                                      1000));
                camera.SetModelViewMatrix(pangolin::ModelViewLookAt(0,
                                                                    0.01,
                                                                    10,
                                                                    0,
                                                                    0,
                                                                    0,
                                                                    0.0,
                                                                    0.0,
                                                                    1.0));
                camera.Follow(cameraCenter_World);
            }
            isFollowing = true;
        }
        else if (!menuFollowCamera && isFollowing)
        {
            isFollowing = false;
        }

        if (menuCameraView)
        {
            menuCameraView = false;
            isCameraView   = true;
            camera.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,
                                                                  768,
                                                                  viewpointF,
                                                                  viewpointF,
                                                                  512,
                                                                  389,
                                                                  0.1,
                                                                  10000));
            camera.SetModelViewMatrix(pangolin::ModelViewLookAt(viewpointX,
                                                                viewpointY,
                                                                viewpointZ,
                                                                0,
                                                                0,
                                                                0,
                                                                0.0,
                                                                -1.0,
                                                                0.0));
            camera.Follow(poseCameraToWorld);
        }

        bool isImuInitialized2{};
        if ((menuTopView) &&
            p_mapDrawer->p_atlas->isImuInitialized(isImuInitialized2) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isImuInitialized returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (menuTopView && isImuInitialized2)
        {
            menuTopView  = false;
            isCameraView = false;
            camera.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,
                                                                  768,
                                                                  3000,
                                                                  3000,
                                                                  512,
                                                                  389,
                                                                  0.1,
                                                                  10000));
            camera.SetModelViewMatrix(
                pangolin::ModelViewLookAt(0, 0.01, 50, 0, 0, 0, 0.0, 0.0, 1.0));
            camera.Follow(cameraCenter_World);
        }

        if (menuLocalizationMode && !isLocalizationMode)
        {
            if (p_system->activateLocalizationMode() !=
                SystemStatus::SYSTEM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: activateLocalizationMode returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            isLocalizationMode = true;
        }
        else if (!menuLocalizationMode && isLocalizationMode)
        {
            if (p_system->deactivateLocalizationMode() !=
                SystemStatus::SYSTEM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: deactivateLocalizationMode returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            isLocalizationMode = false;
        }

        if (menuStepByStep && !stepByStep)
        {
            // cout << "Viewer: step by step" << endl;
            if (p_tracker->setStepByStep(true) !=
                TrackingStatus::TRACKING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setStepByStep returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            stepByStep = true;
        }
        else if (!menuStepByStep && stepByStep)
        {
            if (p_tracker->setStepByStep(false) !=
                TrackingStatus::TRACKING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setStepByStep returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            stepByStep = false;
        }

        if (menuStep)
        {
            p_tracker->isStepRequested = true;
            menuStep                   = false;
        }

        cameraView.Activate(camera);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        if (p_mapDrawer->drawCurrentCamera(poseCameraToWorld) !=
            MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: drawCurrentCamera returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (menuShowKeyFrames || menuShowGraph || menuShowInertialGraph ||
            menuShowOptLba)
        {
            if (p_mapDrawer->drawKeyFrames(menuShowKeyFrames,
                                           menuShowGraph,
                                           menuShowInertialGraph,
                                           menuShowOptLba) !=
                MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: drawKeyFrames returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
        if (menuShowPoints)
        {
            if (p_mapDrawer->drawMapPoints() !=
                MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: drawMapPoints returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        // Draw world frame
        pangolin::glDrawAxis(10.0);

        pangolin::FinishFrame();

        cv::Mat toShow;
        cv::Mat image{};
        if (p_frameDrawer->drawFrame(image, trackedImageScale) !=
            FrameDrawerStatus::FRAME_DRAWER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: drawFrame returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        if (shouldDrawBothImages)
        {
            cv::Mat imageRight{};
            if (p_frameDrawer->drawRightFrame(imageRight, trackedImageScale) !=
                FrameDrawerStatus::FRAME_DRAWER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: drawRightFrame returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            cv::hconcat(image, imageRight, toShow);
        }
        else
        {
            toShow = image;
        }

        if (imageViewerScale != 1.f)
        {
            int width  = toShow.cols * imageViewerScale;
            int height = toShow.rows * imageViewerScale;
            cv::resize(toShow, toShow, cv::Size(width, height));
        }

        cv::imshow("ORB-SLAM3: Current Frame", toShow);
        cv::waitKey(framePeriod);

        if (menuReset)
        {
            menuShowGraph         = true;
            menuShowInertialGraph = true;
            menuShowKeyFrames     = true;
            menuShowPoints        = true;
            menuLocalizationMode  = false;
            if (isLocalizationMode)
            {
                if (p_system->deactivateLocalizationMode() !=
                    SystemStatus::SYSTEM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: deactivateLocalizationMode returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }
            }
            isLocalizationMode = false;
            isFollowing        = true;
            menuFollowCamera   = true;
            if (p_system->requestResetActiveMapWithCause(
                    ResetCause::VIEWER_REQUEST) !=
                SystemStatus::SYSTEM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: requestResetActiveMapWithCause returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
            menuReset = false;
        }

        if (menuStop)
        {
            if (isLocalizationMode)
            {
                if (p_system->deactivateLocalizationMode() !=
                    SystemStatus::SYSTEM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: deactivateLocalizationMode returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }
            }

            // Stop all threads
            if (p_system->shutdown() != SystemStatus::SYSTEM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: shutdown returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }

            // Save camera trajectory
            if (p_system->saveTrajectoryEuRoC("CameraTrajectory.txt") !=
                SystemStatus::SYSTEM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: saveTrajectoryEuRoC returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (p_system->saveKeyFrameTrajectoryEuRoC(
                    "KeyFrameTrajectory.txt") !=
                SystemStatus::SYSTEM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: saveKeyFrameTrajectoryEuRoC returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            menuStop = false;
        }

        bool isStopped2{};
        if (stop(isStopped2) != ViewerStatus::VIEWER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: stop returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (isStopped2)
        {
            for (;;)
            {
                bool isStopped3{};
                if (isStopped(isStopped3) !=
                    ViewerStatus::VIEWER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: isStopped returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (!isStopped3)
                {
                    break;
                }
                usleep(3000);
            }
        }

        bool shouldFinish{};
        if (checkFinish(shouldFinish) != ViewerStatus::VIEWER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: checkFinish returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (shouldFinish)
            break;
    }

    if (setFinish() != ViewerStatus::VIEWER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setFinish returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
}

} // namespace core
} // namespace vs_graphs

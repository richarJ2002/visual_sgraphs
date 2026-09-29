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

#include "FrameDrawer.h"
#include "ResetCause.h"
#include "System.h"
#include "Tracking.h"
#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

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

    pangolin::OpenGlMatrix Twc, Twr;
    Twc.SetIdentity();
    pangolin::OpenGlMatrix Ow; // Oriented with g in the z axis
    Ow.SetIdentity();
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

    float trackedImageScale = p_tracker->getImageScale();

    cout << "Starting the Viewer" << endl;
    while (1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        p_mapDrawer->getCurrentOpenGLCameraMatrix(Twc, Ow);

        if (isTrackingStopRequested)
        {
            menuStepByStep          = true;
            isTrackingStopRequested = false;
        }

        if (menuFollowCamera && isFollowing)
        {
            if (isCameraView)
                camera.Follow(Twc);
            else
                camera.Follow(Ow);
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
                camera.Follow(Twc);
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
                camera.Follow(Ow);
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
            camera.Follow(Twc);
        }

        if (menuTopView && p_mapDrawer->p_atlas->isImuInitialized())
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
            camera.Follow(Ow);
        }

        if (menuLocalizationMode && !isLocalizationMode)
        {
            p_system->activateLocalizationMode();
            isLocalizationMode = true;
        }
        else if (!menuLocalizationMode && isLocalizationMode)
        {
            p_system->deactivateLocalizationMode();
            isLocalizationMode = false;
        }

        if (menuStepByStep && !stepByStep)
        {
            // cout << "Viewer: step by step" << endl;
            p_tracker->setStepByStep(true);
            stepByStep = true;
        }
        else if (!menuStepByStep && stepByStep)
        {
            p_tracker->setStepByStep(false);
            stepByStep = false;
        }

        if (menuStep)
        {
            p_tracker->isStepRequested = true;
            menuStep                   = false;
        }

        cameraView.Activate(camera);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        p_mapDrawer->drawCurrentCamera(Twc);
        if (menuShowKeyFrames || menuShowGraph || menuShowInertialGraph ||
            menuShowOptLba)
            p_mapDrawer->drawKeyFrames(menuShowKeyFrames,
                                       menuShowGraph,
                                       menuShowInertialGraph,
                                       menuShowOptLba);
        if (menuShowPoints)
            p_mapDrawer->drawMapPoints();

        // Draw world frame
        pangolin::glDrawAxis(10.0);

        pangolin::FinishFrame();

        cv::Mat toShow;
        cv::Mat image = p_frameDrawer->drawFrame(trackedImageScale);

        if (shouldDrawBothImages)
        {
            cv::Mat imageRight =
                p_frameDrawer->drawRightFrame(trackedImageScale);
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
                p_system->deactivateLocalizationMode();
            isLocalizationMode = false;
            isFollowing        = true;
            menuFollowCamera   = true;
            p_system->requestResetActiveMapWithCause(
                ResetCause::VIEWER_REQUEST);
            menuReset = false;
        }

        if (menuStop)
        {
            if (isLocalizationMode)
                p_system->deactivateLocalizationMode();

            // Stop all threads
            p_system->shutdown();

            // Save camera trajectory
            p_system->saveTrajectoryEuRoC("CameraTrajectory.txt");
            p_system->saveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
            menuStop = false;
        }

        if (stop())
        {
            while (isStopped())
            {
                usleep(3000);
            }
        }

        if (checkFinish())
            break;
    }

    setFinish();
}

} // namespace core
} // namespace vs_graphs

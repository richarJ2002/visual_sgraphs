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
 * @file         Viewer.cc
 *
 * @brief        Implements Viewer declared in Viewer.h.
 */

#include "ResetCause.h"
#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace vs_graphs
{
namespace core
{

Viewer::Viewer(System                    *pSystem,
               FrameDrawer               *pFrameDrawer,
               MapDrawer                 *pMapDrawer,
               Tracking                  *pTracking,
               const string              &strSettingPath,
               utils::settings::Settings *settings) :
    both(false),
    p_system(pSystem),
    p_frameDrawer(pFrameDrawer),
    p_mapDrawer(pMapDrawer),
    p_tracker(pTracking),
    finishRequested(false),
    finished(true),
    stopped(true),
    stopRequestedFlag(false)
{
    if (settings)
    {
        newParameterLoader(settings);
    }
    else
    {

        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        bool is_correct = parseViewerParamFile(fSettings);

        if (!is_correct)
        {
            std::cerr
                << "**ERROR in the config file, the format is not correct**"
                << std::endl;
            try
            {
                throw -1;
            }
            catch (exception &e)
            {}
        }
    }

    stopTrack = false;
}

void Viewer::newParameterLoader(utils::settings::Settings *settings)
{
    imageViewerScale = 1.f;

    float fps = settings->getFramesPerSecond();
    if (fps < 1)
        fps = 30;
    framePeriod = 1e3 / fps;

    cv::Size imSize = settings->newImSize();
    imageHeight     = imSize.height;
    imageWidth      = imSize.width;

    imageViewerScale = settings->imageViewerScale();
    viewpointX       = settings->viewPointX();
    viewpointY       = settings->viewPointY();
    viewpointZ       = settings->viewPointZ();
    viewpointF       = settings->viewPointF();
}

bool Viewer::parseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    imageViewerScale   = 1.f;

    float fps = fSettings["Camera.fps"];
    if (fps < 1)
        fps = 30;
    framePeriod = 1e3 / fps;

    cv::FileNode node = fSettings["Camera.width"];
    if (!node.empty())
    {
        imageWidth = node.real();
    }
    else
    {
        std::cerr
            << "*Camera.width parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Camera.height"];
    if (!node.empty())
    {
        imageHeight = node.real();
    }
    else
    {
        std::cerr
            << "*Camera.height parameter doesn't exist or is not a real number*"
            << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.imageViewScale"];
    if (!node.empty())
    {
        imageViewerScale = node.real();
    }

    node = fSettings["Viewer.ViewpointX"];
    if (!node.empty())
    {
        viewpointX = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointX parameter doesn't exist or is not a "
                     "real number*"
                  << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointY"];
    if (!node.empty())
    {
        viewpointY = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointY parameter doesn't exist or is not a "
                     "real number*"
                  << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointZ"];
    if (!node.empty())
    {
        viewpointZ = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointZ parameter doesn't exist or is not a "
                     "real number*"
                  << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointF"];
    if (!node.empty())
    {
        viewpointF = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointF parameter doesn't exist or is not a "
                     "real number*"
                  << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}

void Viewer::run()
{
    finished = false;
    stopped  = false;

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
    pangolin::Var<bool> menuCamView("menu.Camera View", false, false);
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
    pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(1024,
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
    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0,
                                           1.0,
                                           pangolin::Attach::Pix(175),
                                           1.0,
                                           -1024.0f / 768.0f)
                                .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc, Twr;
    Twc.SetIdentity();
    pangolin::OpenGlMatrix Ow; // Oriented with g in the z axis
    Ow.SetIdentity();
    cv::namedWindow("ORB-SLAM3: Current Frame");

    bool bFollow           = true;
    bool bLocalizationMode = false;
    bool stepByStep        = false;
    bool bCameraView       = true;

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

        if (stopTrack)
        {
            menuStepByStep = true;
            stopTrack      = false;
        }

        if (menuFollowCamera && bFollow)
        {
            if (bCameraView)
                s_cam.Follow(Twc);
            else
                s_cam.Follow(Ow);
        }
        else if (menuFollowCamera && !bFollow)
        {
            if (bCameraView)
            {
                s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,
                                                                     768,
                                                                     viewpointF,
                                                                     viewpointF,
                                                                     512,
                                                                     389,
                                                                     0.1,
                                                                     1000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(viewpointX,
                                                                   viewpointY,
                                                                   viewpointZ,
                                                                   0,
                                                                   0,
                                                                   0,
                                                                   0.0,
                                                                   -1.0,
                                                                   0.0));
                s_cam.Follow(Twc);
            }
            else
            {
                s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,
                                                                     768,
                                                                     3000,
                                                                     3000,
                                                                     512,
                                                                     389,
                                                                     0.1,
                                                                     1000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,
                                                                   0.01,
                                                                   10,
                                                                   0,
                                                                   0,
                                                                   0,
                                                                   0.0,
                                                                   0.0,
                                                                   1.0));
                s_cam.Follow(Ow);
            }
            bFollow = true;
        }
        else if (!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if (menuCamView)
        {
            menuCamView = false;
            bCameraView = true;
            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,
                                                                 768,
                                                                 viewpointF,
                                                                 viewpointF,
                                                                 512,
                                                                 389,
                                                                 0.1,
                                                                 10000));
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(viewpointX,
                                                               viewpointY,
                                                               viewpointZ,
                                                               0,
                                                               0,
                                                               0,
                                                               0.0,
                                                               -1.0,
                                                               0.0));
            s_cam.Follow(Twc);
        }

        if (menuTopView && p_mapDrawer->p_atlas->isImuInitialized())
        {
            menuTopView = false;
            bCameraView = false;
            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,
                                                                 768,
                                                                 3000,
                                                                 3000,
                                                                 512,
                                                                 389,
                                                                 0.1,
                                                                 10000));
            s_cam.SetModelViewMatrix(
                pangolin::ModelViewLookAt(0, 0.01, 50, 0, 0, 0, 0.0, 0.0, 1.0));
            s_cam.Follow(Ow);
        }

        if (menuLocalizationMode && !bLocalizationMode)
        {
            p_system->activateLocalizationMode();
            bLocalizationMode = true;
        }
        else if (!menuLocalizationMode && bLocalizationMode)
        {
            p_system->deactivateLocalizationMode();
            bLocalizationMode = false;
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
            p_tracker->step = true;
            menuStep        = false;
        }

        d_cam.Activate(s_cam);
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
        cv::Mat im = p_frameDrawer->drawFrame(trackedImageScale);

        if (both)
        {
            cv::Mat imRight = p_frameDrawer->drawRightFrame(trackedImageScale);
            cv::hconcat(im, imRight, toShow);
        }
        else
        {
            toShow = im;
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
            if (bLocalizationMode)
                p_system->deactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow           = true;
            menuFollowCamera  = true;
            p_system->requestResetActiveMapWithCause(
                ResetCause::VIEWER_REQUEST);
            menuReset = false;
        }

        if (menuStop)
        {
            if (bLocalizationMode)
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

void Viewer::requestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    finishRequested = true;
}

bool Viewer::checkFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return finishRequested;
}

void Viewer::setFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    finished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return finished;
}

void Viewer::requestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if (!stopped)
        stopRequestedFlag = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return stopped;
}

bool Viewer::stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if (finishRequested)
        return false;
    else if (stopRequestedFlag)
    {
        stopped           = true;
        stopRequestedFlag = false;
        return true;
    }

    return false;
}

void Viewer::release()
{
    unique_lock<mutex> lock(mMutexStop);
    stopped = false;
}

/*void Viewer::SetTrackingPause()
{
    mbStopTrack = true;
}*/

} // namespace core
} // namespace vs_graphs

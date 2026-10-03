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
 * @file            Viewer.h
 *
 * @brief           Declares Viewer, the window that draws the map, the camera
 *                  and the current frame.
 */

#ifndef VIEWER_H
#define VIEWER_H

#include "MapDrawer.h"
#include "Utils/Settings/objects/Settings.h"
#include "ViewerStatus.h"

#include <iostream>
#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;

/*!
 * @brief        Pangolin map viewer and OpenCV frame window, run on its own
 *               thread. It can be asked to stop (pause) or to finish.
 */
class Viewer
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief        Creates a viewer in the finished and stopped state and
     *               loads its display settings. Call run() on a thread to open
     *               the windows.
     *
     * @param[in]    p_system_in
     *               System that owns this viewer. Borrowed.
     * @param[in]    p_frameDrawer_in
     *               Source of the annotated camera image. Borrowed.
     * @param[in]    p_mapDrawer_in
     *               Source of the 3D map drawing and camera pose. Borrowed.
     * @param[in]    p_tracking_in
     *               Tracker that is controlled from the viewer menu. Borrowed.
     * @param[in]    settingsFilePath_in
     *               Settings file read only when p_settings_in is null.
     * @param[in]    p_settings_in
     *               Parsed settings, or null to read the settings file.
     */
    Viewer(System                    *p_system_in,
           FrameDrawer               *p_frameDrawer_in,
           MapDrawer                 *p_mapDrawer_in,
           Tracking                  *p_tracking_in,
           const std::string         &settingsFilePath_in,
           utils::settings::Settings *p_settings_in) :
        shouldDrawBothImages(false),
        p_system(p_system_in),
        p_frameDrawer(p_frameDrawer_in),
        p_mapDrawer(p_mapDrawer_in),
        p_tracker(p_tracking_in),
        isFinishRequested(false),
        hasFinished(true),
        hasStopped(true),
        isStopRequested(false)
    {
        if (p_settings_in)
        {
            if (newParameterLoader(p_settings_in) !=
                ViewerStatus::VIEWER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: newParameterLoader returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
        else
        {

            cv::FileStorage settingsFileStorage(settingsFilePath_in,
                                                cv::FileStorage::READ);

            bool isConfigValid{};
            if (parseViewerParamFile(settingsFileStorage, isConfigValid) !=
                ViewerStatus::VIEWER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: parseViewerParamFile returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            if (!isConfigValid)
            {
                std::cerr
                    << "**ERROR in the config file, the format is not correct**"
                    << std::endl;
                try
                {
                    throw -1;
                }
                catch (std::exception &parseError)
                {}
            }
        }

        isTrackingStopRequested = false;
    }

    /*!
     * @brief        Loads the frame rate, image size, image scale and 3D
     *               viewpoint from the settings object.
     *
     * @param[in,out] p_settings_inout
     *               Settings to read from. Must not be null.
     *
     * @return       VIEWER_STATUS_SUCCESS always.
     */
    [[nodiscard]] ViewerStatus
        newParameterLoader(utils::settings::Settings *p_settings_inout);

    /*!
     * @brief        Viewer thread function: draws map points, key frames, the
     *               current camera pose and the last processed frame until
     *               requestFinish() is called. Drawing is refreshed at the
     *               camera frame rate.
     */
    void run();

    /*!
     * @brief        Asks run() to leave its loop and return.
     *
     * @return       VIEWER_STATUS_SUCCESS always.
     */
    [[nodiscard]] ViewerStatus requestFinish();

    /*!
     * @brief        Asks run() to pause until release() is called. Ignored
     *               when the viewer is already stopped.
     *
     * @return       VIEWER_STATUS_SUCCESS always.
     */
    [[nodiscard]] ViewerStatus requestStop();

    /*!
     * @brief        Tells whether run() has returned (or has not started yet).
     *
     * @param[out]   isFinished_out
     *               True when the viewer thread is not running its loop.
     *
     * @return       VIEWER_STATUS_SUCCESS always.
     */
    [[nodiscard]] ViewerStatus isFinished(bool &isFinished_out);

    /*!
     * @brief        Tells whether the viewer is paused.
     *
     * @param[out]   isStopped_out
     *               True when the viewer is stopped (also before run()
     *               starts).
     *
     * @return       VIEWER_STATUS_SUCCESS always.
     */
    [[nodiscard]] ViewerStatus isStopped(bool &isStopped_out);

    /*!
     * @brief        Lets a paused viewer continue drawing.
     *
     * @return       VIEWER_STATUS_SUCCESS always.
     */
    [[nodiscard]] ViewerStatus release();

    // void SetTrackingPause();

    /*!
     * @brief        True when the camera has a right image that run() shows
     *               next to the left one. Set by the system from the frame
     *               drawer.
     */
    bool shouldDrawBothImages;

  private:
    /*!
     * @brief        Reads the frame rate, image size, image scale and 3D
     *               viewpoint from the "Viewer.*" entries of a settings file.
     *
     * @param[in]    settings_in
     *               Opened settings file.
     * @param[out]   isParsed_out
     *               False when any viewpoint entry is missing or not a real
     *               number.
     *
     * @return       VIEWER_STATUS_SUCCESS always.
     */
    [[nodiscard]] ViewerStatus
        parseViewerParamFile(cv::FileStorage &settings_in, bool &isParsed_out);

    /*!
     * @brief        Turns a pending stop request into the stopped state,
     *               unless a finish was requested.
     *
     * @param[out]   isStopped_out
     *               True when the viewer has just become stopped.
     *
     * @return       VIEWER_STATUS_SUCCESS always.
     */
    [[nodiscard]] ViewerStatus stop(bool &isStopped_out);

    /*!
     * @brief        System that owns this viewer. Borrowed.
     */
    System *p_system;

    /*!
     * @brief        Source of the annotated camera images. Borrowed.
     */
    FrameDrawer *p_frameDrawer;

    /*!
     * @brief        Source of the 3D map drawing and camera pose. Borrowed.
     */
    MapDrawer *p_mapDrawer;

    /*!
     * @brief        Tracker controlled from the viewer menu. Borrowed.
     */
    Tracking *p_tracker;

    /*!
     * @brief        Time between two redraws of the frame window, in
     *               milliseconds (1000 divided by the camera frame rate).
     */
    double framePeriod;

    /*!
     * @brief        Width of the camera image, in pixels.
     */
    float imageWidth;

    /*!
     * @brief        Height of the camera image, in pixels.
     */
    float imageHeight;

    /*!
     * @brief        Factor by which the frame window is enlarged (1 keeps the
     *               image size).
     */
    float imageViewerScale;

    /*!
     * @brief        X position of the initial 3D view camera, in the map
     *               frame.
     */
    float viewpointX;

    /*!
     * @brief        Y position of the initial 3D view camera, in the map
     *               frame.
     */
    float viewpointY;

    /*!
     * @brief        Z position of the initial 3D view camera, in the map
     *               frame.
     */
    float viewpointZ;

    /*!
     * @brief        Focal length of the 3D view camera, in pixels.
     */
    float viewpointF;

    /*!
     * @brief        Tells whether requestFinish() was called.
     *
     * @param[out]   isFinishRequested_out
     *               True when run() should leave its loop.
     *
     * @return       VIEWER_STATUS_SUCCESS always.
     */
    [[nodiscard]] ViewerStatus checkFinish(bool &isFinishRequested_out);

    /*!
     * @brief        Marks run() as finished; called when its loop ends.
     *
     * @return       VIEWER_STATUS_SUCCESS always.
     */
    [[nodiscard]] ViewerStatus setFinish();

    /*!
     * @brief        True once requestFinish() was called. Guarded by
     *               finishMutex.
     */
    bool isFinishRequested;

    /*!
     * @brief        True while run() is not looping: before it starts and after
     *               it returns. Guarded by finishMutex.
     */
    bool hasFinished;

    /*!
     * @brief        Guards isFinishRequested and hasFinished.
     */
    std::mutex finishMutex;

    /*!
     * @brief        True while the viewer is paused, and before run() starts.
     *               Guarded by stopMutex.
     */
    bool hasStopped;

    /*!
     * @brief        True after requestStop() until the viewer pauses. Guarded
     *               by stopMutex.
     */
    bool isStopRequested;

    /*!
     * @brief        Guards hasStopped and isStopRequested.
     */
    std::mutex stopMutex;

    /*!
     * @brief        When true, run() switches the menu to step-by-step mode and
     *               clears it. Only read in run(); nothing sets it to true.
     */
    bool isTrackingStopRequested;
};

} // namespace core
} // namespace vs_graphs
#endif // VIEWER_H

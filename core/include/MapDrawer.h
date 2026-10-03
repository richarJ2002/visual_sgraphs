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
 * @file            MapDrawer.h
 *
 * @brief           Declares MapDrawer, which draws the map points, key frames
 *                  and camera in the viewer.
 */

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "Atlas.h"
#include "KeyFrame.h"
#include "MapDrawerStatus.h"
#include "MapPoint.h"
#include "Utils/Settings/objects/Settings.h"
#include <pangolin/pangolin.h>

#include <iostream>
#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief        Draws the SLAM map (map points, key frames, covisibility
 *               graph and the current camera) with OpenGL for the viewer.
 */
class MapDrawer
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief        Creates the drawer and loads its drawing sizes, either from
     *               the settings object or from the settings file.
     *
     * @param[in]    p_atlas_in
     *               Atlas that holds the maps to draw. Borrowed, not deleted
     *               by the drawer.
     * @param[in]    settingsFilePath_in
     *               Settings file read only when p_settings_in is null.
     * @param[in]    p_settings_in
     *               Parsed settings, or null to read the settings file.
     */
    MapDrawer(Atlas                     *p_atlas_in,
              const std::string         &settingsFilePath_in,
              utils::settings::Settings *p_settings_in) :
        p_atlas(p_atlas_in)
    {
        if (p_settings_in)
        {
            if (newParameterLoader(p_settings_in) !=
                MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS)
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
            bool            isViewerConfigValid{};
            if (parseViewerParamFile(settingsFileStorage,
                                     isViewerConfigValid) !=
                MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: parseViewerParamFile returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            if (!isViewerConfigValid)
            {
                std::cerr
                    << "**ERROR in the config file, the format is not correct**"
                    << std::endl;
                try
                {
                    throw -1;
                }
                catch (std::exception &caughtException)
                {}
            }
        }
    }

    /*!
     * @brief        Loads the key frame, graph, point and camera drawing sizes
     *               from the settings object.
     *
     * @param[in,out] p_settings_inout
     *               Settings to read from. Must not be null.
     *
     * @return       MAP_DRAWER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapDrawerStatus
        newParameterLoader(utils::settings::Settings *p_settings_inout);

    /*!
     * @brief        Atlas whose maps are drawn. Borrowed from the caller of the
     *               constructor; also read by the viewer.
     */
    Atlas *p_atlas;

    /*!
     * @brief        Draws the points of the active map: black for ordinary map
     *               points, red for the tracker's reference (local map)
     *               points. Bad points are skipped.
     *
     * @return       MAP_DRAWER_STATUS_SUCCESS always, also when there is no
     *               active map or it has no points.
     */
    [[nodiscard]] MapDrawerStatus drawMapPoints() const;
    /*!
     * @brief        Draws key frames and graph edges of the maps in the atlas.
     *               Must run on the thread that owns the OpenGL context.
     *
     * @param[in]    shouldDrawKeyFrames_in
     *               Draw the key frame frusta; those of the active map are
     *               blue and those of other maps use a colour per origin map.
     * @param[in]    shouldDrawGraph_in
     *               Draw the covisibility, spanning tree and loop edges
     *               between key frames of the active map.
     * @param[in]    shouldDrawInertialGraph_in
     *               Draw the inertial edges, only when the active map has its
     *               IMU initialised.
     * @param[in]    shouldDrawOptimizedLba_in
     *               Colour active-map key frames of the last local bundle
     *               adjustment: green when optimised, red when fixed.
     *
     * @return       MAP_DRAWER_STATUS_SUCCESS always, also when there is no
     *               active map.
     */
    [[nodiscard]] MapDrawerStatus
        drawKeyFrames(const bool shouldDrawKeyFrames_in,
                      const bool shouldDrawGraph_in,
                      const bool shouldDrawInertialGraph_in,
                      const bool shouldDrawOptimizedLba_in);
    /*!
     * @brief        Draws the camera frustum in green at the given pose. Must
     *               run on the thread that owns the OpenGL context.
     *
     * @param[in]    cameraPose_cameraToWorld_in
     *               Camera pose as a column-major OpenGL matrix mapping
     *               camera-frame points into the world frame.
     *
     * @return       MAP_DRAWER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapDrawerStatus drawCurrentCamera(
        pangolin::OpenGlMatrix &cameraPose_cameraToWorld_in) const;
    /*!
     * @brief        Stores the latest camera pose for the viewer to draw and
     *               follow. Safe to call from the tracking thread.
     *
     * @param[in]    cameraPose_worldToCamera_in
     *               Camera pose that maps world-frame points into the camera
     *               frame; stored inverted as camera to world.
     *
     * @return       MAP_DRAWER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapDrawerStatus
        setCurrentCameraPose(const Sophus::SE3f &cameraPose_worldToCamera_in);
    /*!
     * @brief        Converts the stored camera pose into OpenGL matrices.
     *
     * @param[out]   M_in
     *               Camera pose mapping camera-frame points into the world
     *               frame, as a column-major OpenGL matrix.
     * @param[out]   MOw_inout
     *               Identity rotation with the camera centre in the world
     *               frame as translation.
     *
     * @return       MAP_DRAWER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapDrawerStatus
        getCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M_in,
                                     pangolin::OpenGlMatrix &MOw_inout);

  private:
    /*!
     * @brief        Reads the key frame, graph, point and camera drawing sizes
     *               from the "Viewer.*" entries of a settings file.
     *
     * @param[in]    settings_in
     *               Opened settings file.
     * @param[out]   isParsed_out
     *               False when any of the entries is missing or not a real
     *               number; the missing entry's member keeps its old value.
     *
     * @return       MAP_DRAWER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapDrawerStatus
        parseViewerParamFile(cv::FileStorage &settings_in, bool &isParsed_out);

    /*!
     * @brief        Width of a key frame frustum, in OpenGL scene units (map
     *               units).
     */
    float keyFrameSize;

    /*!
     * @brief        Line width of a key frame frustum, in pixels.
     */
    float keyFrameLineWidth;

    /*!
     * @brief        Line width of the graph edges, in pixels.
     */
    float graphLineWidth;

    /*!
     * @brief        Size of a map point, in pixels.
     */
    float pointSize;

    /*!
     * @brief        Width of the current camera frustum, in OpenGL scene units
     *               (map units).
     */
    float cameraSize;

    /*!
     * @brief        Line width of the current camera frustum, in pixels.
     */
    float cameraLineWidth;

    /*!
     * @brief        Latest camera pose mapping camera-frame points into the
     *               world frame. Guarded by cameraMutex.
     */
    Sophus::SE3f cameraPose;

    /*!
     * @brief        Guards cameraPose between the tracking thread that sets it
     *               and the viewer thread that reads it.
     */
    std::mutex cameraMutex;

    /*!
     * @brief        RGB colours (0 to 1) of key frames from other maps, indexed
     *               by the key frame's origin map id.
     */
    float frameColors[6][3] = {{0.0f, 0.0f, 1.0f},
                               {0.8f, 0.4f, 1.0f},
                               {1.0f, 0.2f, 0.4f},
                               {0.6f, 0.0f, 1.0f},
                               {1.0f, 1.0f, 0.0f},
                               {0.0f, 1.0f, 1.0f}};
};

} // namespace core
} // namespace vs_graphs

#endif // MAPDRAWER_H

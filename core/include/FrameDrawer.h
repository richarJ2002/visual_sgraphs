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
 * @file            FrameDrawer.h
 *
 * @brief           Declares FrameDrawer, which draws the current image with its
 *                  tracked features for the viewer.
 */

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Atlas.h"
#include "Frame.h"
#include "FrameDrawerStatus.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <mutex>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{
class MapPoint;
} // namespace core
} // namespace vs_graphs

namespace vs_graphs
{
namespace core
{

class Tracking;
class Viewer;

/*!
 * @brief        Keeps a copy of the tracker's last processed frame and draws it
 *               as an annotated camera image for the viewer window.
 */
class FrameDrawer
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief        Creates a drawer that shows a black 640x480 image until the
     *               first frame is passed to update().
     *
     * @param[in]    p_atlas_in
     *               Atlas whose map, key frame and map point counts are shown
     *               in the status text. Borrowed, not deleted by the drawer.
     */
    FrameDrawer(Atlas *p_atlas_in);

    /*!
     * @brief        Copies the tracker's last processed frame (image, key
     *               points, map point matches, tracking state) so that
     *               drawFrame() can draw it from the viewer thread.
     *
     * @param[in]    p_tracker_in
     *               Tracker to copy from. Must not be null.
     *
     * @return       FRAME_DRAWER_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameDrawerStatus update(Tracking *p_tracker_in);

    /*!
     * @brief        Draws the last copied left image with its key points and
     *               a status line underneath.
     *
     * @param[out]   frameImage_out
     *               Annotated image.
     * @param[in]    imageScale_in
     *               Divisor applied to the image size and key point pixels;
     *               1 keeps the original size.
     *
     * @return       FRAME_DRAWER_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameDrawerStatus drawFrame(cv::Mat &frameImage_out,
                                              float    imageScale_in = 1.f);

    /*!
     * @brief        Draws the last copied right image of a stereo pair, like
     *               drawFrame() does for the left one.
     *
     * @param[out]   frameImage_out
     *               Annotated image.
     * @param[in]    imageScale_in
     *               Divisor applied to the image size and key point pixels;
     *               1 keeps the original size.
     *
     * @return       FRAME_DRAWER_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameDrawerStatus drawRightFrame(cv::Mat &frameImage_out,
                                                   float imageScale_in = 1.f);

    /*!
     * @brief        True when the camera has a right image that update() must
     *               copy as well. Set by the tracker from the camera settings.
     */
    bool shouldDrawBothImages;

  protected:
    /*!
     * @brief        Appends a black strip with the tracking state text (state,
     *               map, key frame and map point counts, match counts) under
     *               an image.
     *
     * @param[in]    sourceImage_in
     *               Image to annotate.
     * @param[in]    trackingState_in
     *               Tracker state as a Tracking state value.
     * @param[out]   annotatedImage_out
     *               Copy of the source image with the text strip below it.
     *
     * @return       FRAME_DRAWER_STATUS_SUCCESS always.
     */
    [[nodiscard]] FrameDrawerStatus drawTextInfo(cv::Mat &sourceImage_in,
                                                 int      trackingState_in,
                                                 cv::Mat &annotatedImage_out);

    // Info of the frame to be drawn

    /*!
     * @brief        Left grayscale image of the last processed frame.
     */
    cv::Mat image;

    /*!
     * @brief        Right grayscale image of the last processed frame; only
     *               filled when shouldDrawBothImages is true.
     */
    cv::Mat imageRight;

    /*!
     * @brief        Number of key points of the last frame (left plus right
     *               when both images are drawn).
     */
    int keyPointCount;

    /*!
     * @brief        Left key points of the last frame, in pixels.
     */
    std::vector<cv::KeyPoint> currentKeys;

    /*!
     * @brief        Right key points of the last frame, in pixels.
     */
    std::vector<cv::KeyPoint> currentKeysRight;

    /*!
     * @brief        Per key point: true when it is matched to a map point that
     *               other key frames observe.
     */
    std::vector<bool> isTrackedMapPoint;

    /*!
     * @brief        Per key point: true when it is matched to a map point that
     *               no key frame observes yet (created by visual odometry in
     *               the previous frame).
     */
    std::vector<bool> isVisualOdometryPoint;

    /*!
     * @brief        True when the tracker only localises and does not extend
     *               the map.
     */
    bool isTrackingOnlyMode;

    /*!
     * @brief        Number of key points drawn as matched to map points;
     *               counted by drawFrame() and shown in the status text.
     */
    int trackedCount;

    /*!
     * @brief        Number of key points drawn as matched to visual odometry
     *               map points; counted by drawFrame() and shown in the status
     *               text.
     */
    int trackedVOCount;

    /*!
     * @brief        Key points of the initial reference frame, in pixels; used
     *               while the tracker is not initialised yet.
     */
    std::vector<cv::KeyPoint> iniKeys;

    /*!
     * @brief        For each key point in iniKeys, the index of its match in
     *               currentKeys, or a negative value when it has no match.
     */
    std::vector<int> iniMatches;

    /*!
     * @brief        Tracking state of the last frame, as a Tracking state
     *               value.
     */
    int state;

    /*!
     * @brief        Depth of each key point of the last frame, in metres.
     */
    std::vector<float> currentDepths;

    /*!
     * @brief        Depth that separates close from far points of the last
     *               frame, in metres.
     */
    float depthThreshold;

    /*!
     * @brief        Atlas used for the counts in the status text. Borrowed from
     *               the caller of the constructor.
     */
    Atlas *p_atlas;

    /*!
     * @brief        Guards the copied frame data above and below, which
     *               update() writes and drawFrame() and drawRightFrame() read
     *               from different threads.
     */
    std::mutex frameStateMutex;

    /*!
     * @brief        Pairs of previous and current pixel positions that
     *               drawFrame() draws as thick lines while the tracker is not
     *               initialised. Nothing in the code fills it.
     */
    std::vector<std::pair<cv::Point2f, cv::Point2f>> tracks;

    /*!
     * @brief        Copy of the tracker's last processed frame.
     */
    Frame currentFrame;

    /*!
     * @brief        Map points of the tracker's local map. Borrowed from the
     *               map, not deleted here.
     */
    std::vector<MapPoint *> localMap;

    /*!
     * @brief        Key points matched to map points. Emptied by update() and
     *               never filled.
     */
    std::vector<cv::KeyPoint> matchedKeys;

    /*!
     * @brief        Map points matched to key points. Emptied by update() and
     *               never filled.
     */
    std::vector<MapPoint *> matchedMPs;

    /*!
     * @brief        Key points whose map point match was rejected as an
     *               outlier, in pixels.
     */
    std::vector<cv::KeyPoint> outlierKeys;

    /*!
     * @brief        Map points whose match to a key point was rejected as an
     *               outlier. Borrowed from the map, not deleted here.
     */
    std::vector<MapPoint *> outlierMPs;

    /*!
     * @brief        Pixel position of each map point projected into the last
     *               frame, keyed by map point id.
     */
    std::map<long unsigned int, cv::Point2f> projectPoints;

    /*!
     * @brief        Pixel position of the key point matched to each inlier map
     *               point, keyed by map point id.
     */
    std::map<long unsigned int, cv::Point2f> matchedInImage;
};

} // namespace core
} // namespace vs_graphs

#endif // FRAMEDRAWER_H

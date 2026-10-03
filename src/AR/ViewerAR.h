/*!
 * This file is part of ORB-SLAM3.
 * Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details:
 * https://www.gnu.org/licenses/
 */

#ifndef VIEWERAR_H
#define VIEWERAR_H

#include "System.h"
#include <mutex>
#include <opencv2/core/core.hpp>
#include <pangolin/pangolin.h>
#include <string>

namespace ORB_SLAM3
{

/*!
 * @brief        A virtual plane fitted to map points, on which the demo draws
 *               its cubes and grids.
 */
class Plane
{
  public:
    /*!
     * @brief        Fits a plane to map points and orients it towards the
     *               camera that first saw them.
     *
     * @param[in]    vMPs
     *               Map points that define the plane; borrowed, and none may
     *               be null.
     * @param[in]    Tcw
     *               Camera pose at creation time, mapping world-frame points
     *               into the camera frame (4x4 float).
     */
    Plane(const std::vector<MapPoint *> &vMPs, const cv::Mat &Tcw);

    /*!
     * @brief        Builds a plane directly from a normal and an origin, with
     *               no map points behind it.
     *
     * @param[in]    nx
     *               X component of the unit normal, world frame.
     * @param[in]    ny
     *               Y component of the unit normal, world frame.
     * @param[in]    nz
     *               Z component of the unit normal, world frame.
     * @param[in]    ox
     *               X coordinate of the plane origin, world frame.
     * @param[in]    oy
     *               Y coordinate of the plane origin, world frame.
     * @param[in]    oz
     *               Z coordinate of the plane origin, world frame.
     */
    Plane(const float &nx,
          const float &ny,
          const float &nz,
          const float &ox,
          const float &oy,
          const float &oz);

    /*!
     * @brief        Refits the plane to the current positions of its map
     *               points, for use after a loop closure or global bundle
     *               adjustment moved them.
     *
     *               Updates the normal, origin and drawing pose; the yaw angle
     *               is kept.
     */
    void Recompute();

    /*!
     * @brief        Unit normal of the plane, world frame (3x1 float).
     */
    cv::Mat n;

    /*!
     * @brief        Origin of the plane (mean of its map points), world frame
     *               (3x1 float).
     */
    cv::Mat o;

    /*!
     * @brief        Yaw of the drawn cube and grid about the plane normal,
     *               radians; chosen at random once when the plane is created.
     */
    float rang;

    /*!
     * @brief        Pose of the plane in the world: maps points given in plane
     *               coordinates into the world frame (4x4 float).
     */
    cv::Mat Tpw;

    /*!
     * @brief        Tpw in the column-major layout OpenGL expects.
     */
    pangolin::OpenGlMatrix glTpw;

    /*!
     * @brief        Map points that define the plane; borrowed from the map.
     */
    std::vector<MapPoint *> mvMPs;

    /*!
     * @brief        Camera pose when the plane was first observed, mapping
     *               world-frame points into the camera frame (4x4 float).
     */
    cv::Mat mTcw;

    /*!
     * @brief        Vector from the plane origin to that first camera centre,
     *               world frame (3x1 float); set once, used to keep the normal
     *               pointing away from the camera.
     */
    cv::Mat XC;
};

/*!
 * @brief        Augmented-reality demo viewer: shows the SLAM camera image in
 *               a Pangolin window and draws virtual cubes on planes the user
 *               detects.
 *
 *               Run() executes on the viewer thread; the ROS image callback
 *               feeds it through SetImagePose().
 */
class ViewerAR
{
  public:
    ViewerAR();

    /*!
     * @brief        Sets the camera frame rate, which also sets the window
     *               refresh period.
     *
     * @param[in]    fps
     *               Camera frame rate, hertz; must be positive.
     */
    void SetFPS(const float fps)
    {
        mFPS = fps;
        mT   = 1e3 / fps;
    }

    /*!
     * @brief        Tells the viewer which SLAM system to query and to switch
     *               between mapping and localization.
     *
     * @param[in]    pSystem
     *               SLAM system; borrowed, must outlive Run().
     */
    void SetSLAM(ORB_SLAM3::System *pSystem)
    {
        mpSystem = pSystem;
    }

    /*!
     * @brief        Viewer thread body: waits for the first image, opens the
     *               window and draws frames until the process ends.
     *
     *               Call SetSLAM(), SetFPS() and SetCameraCalibration() first.
     *               It never returns.
     */
    void Run();

    /*!
     * @brief        Sets the pinhole intrinsics used for the OpenGL projection.
     *
     * @param[in]    fx_
     *               Focal length along x, pixels.
     * @param[in]    fy_
     *               Focal length along y, pixels.
     * @param[in]    cx_
     *               Principal point x, pixels.
     * @param[in]    cy_
     *               Principal point y, pixels.
     */
    void SetCameraCalibration(const float &fx_,
                              const float &fy_,
                              const float &cx_,
                              const float &cy_)
    {
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
    }

    /*!
     * @brief        Stores the latest SLAM result for the viewer thread to
     *               draw; takes mMutexPoseImage and copies the image.
     *
     * @param[in]    im
     *               Undistorted camera image.
     * @param[in]    Tcw
     *               Camera pose, mapping world-frame points into the camera
     *               frame (4x4 float); empty while there is no pose.
     * @param[in]    status
     *               SLAM tracking state (1 not initialised, 2 tracking,
     *               3 lost).
     * @param[in]    vKeys
     *               Undistorted keypoints of the image.
     * @param[in]    vMPs
     *               Map point tracked by each keypoint, null where none;
     *               borrowed, same size as vKeys.
     */
    void SetImagePose(const cv::Mat                   &im,
                      const cv::Mat                   &Tcw,
                      const int                       &status,
                      const std::vector<cv::KeyPoint> &vKeys,
                      const std::vector<MapPoint *>   &vMPs);

    /*!
     * @brief        Copies out the latest SLAM result stored by
     *               SetImagePose(); takes mMutexPoseImage.
     *
     * @param[out]   im
     *               Copy of the camera image; empty before the first image.
     * @param[out]   Tcw
     *               Copy of the camera pose, world frame to camera frame.
     * @param[out]   status
     *               SLAM tracking state as given to SetImagePose().
     * @param[out]   vKeys
     *               Keypoints of the image.
     * @param[out]   vMPs
     *               Map point tracked by each keypoint, null where none;
     *               borrowed.
     */
    void GetImagePose(cv::Mat                   &im,
                      cv::Mat                   &Tcw,
                      int                       &status,
                      std::vector<cv::KeyPoint> &vKeys,
                      std::vector<MapPoint *>   &vMPs);

  private:
    /*!
     * @brief        SLAM system the viewer queries; borrowed, set by SetSLAM().
     */
    ORB_SLAM3::System *mpSystem;

    /*!
     * @brief        Writes the tracking status as text on the image.
     *
     *               Draws nothing for a status other than 1, 2 or 3.
     *
     * @param[in]    status
     *               SLAM tracking state (1 not initialised, 2 tracking,
     *               3 lost).
     * @param[in]    bLocMode
     *               True in localization mode, which changes the wording.
     * @param[in,out] im
     *               Image drawn on.
     */
    void PrintStatus(const int &status, const bool &bLocMode, cv::Mat &im);

    /*!
     * @brief        Writes text near the bottom-left corner of the image, with
     *               a white outline so it stays readable.
     *
     * @param[in]    s
     *               Text to write.
     * @param[in,out] im
     *               Image drawn on.
     * @param[in]    r
     *               First colour channel of the text, 0 to 255.
     * @param[in]    g
     *               Second colour channel of the text, 0 to 255.
     * @param[in]    b
     *               Third colour channel of the text, 0 to 255.
     */
    void AddTextToImage(const std::string &s,
                        cv::Mat           &im,
                        const int          r = 0,
                        const int          g = 0,
                        const int          b = 0);

    /*!
     * @brief        Loads the camera pose as the OpenGL model-view matrix, so
     *               world-frame drawing lands in the camera view.
     *
     *               Does nothing when the pose is empty.
     *
     * @param[in]    Tcw
     *               Camera pose, mapping world-frame points into the camera
     *               frame (4x4 float).
     */
    void LoadCameraPose(const cv::Mat &Tcw);

    /*!
     * @brief        Draws the camera image as the window background.
     *
     *               Does nothing when the image is empty.
     *
     * @param[in,out] imageTexture
     *               Texture the image is uploaded to.
     * @param[in,out] im
     *               RGB image to draw; its pixel data is read.
     */
    void DrawImageTexture(pangolin::GlTexture &imageTexture, cv::Mat &im);

    /*!
     * @brief        Draws a coloured cube resting on the x-z plane of the
     *               current OpenGL frame.
     *
     * @param[in]    size
     *               Half the cube side length, in map units.
     * @param[in]    x
     *               Shift of the cube by -x along the frame's x axis.
     * @param[in]    y
     *               Shift of the cube by -y along the frame's y axis.
     * @param[in]    z
     *               Shift of the cube by -z along the frame's z axis.
     */
    void DrawCube(const float &size,
                  const float  x = 0,
                  const float  y = 0,
                  const float  z = 0);

    /*!
     * @brief        Draws a square grid on the x-z plane (y = 0) of the
     *               current OpenGL frame.
     *
     * @param[in]    ndivs
     *               Number of grid cells from the centre to each edge.
     * @param[in]    ndivsize
     *               Side length of one cell, in map units.
     */
    void DrawPlane(int ndivs, float ndivsize);

    /*!
     * @brief        Draws the grid in the frame of a detected plane.
     *
     * @param[in]    pPlane
     *               Plane to draw in; borrowed, must not be null.
     * @param[in]    ndivs
     *               Number of grid cells from the centre to each edge.
     * @param[in]    ndivsize
     *               Side length of one cell, in map units.
     */
    void DrawPlane(Plane *pPlane, int ndivs, float ndivsize);

    /*!
     * @brief        Marks every keypoint that has a tracked map point with a
     *               green dot.
     *
     * @param[in]    vKeys
     *               Keypoints of the image, pixel coordinates.
     * @param[in]    vMPs
     *               Map point tracked by each keypoint, null where none; at
     *               least as long as vKeys.
     * @param[in,out] im
     *               Image drawn on.
     */
    void DrawTrackedPoints(const std::vector<cv::KeyPoint> &vKeys,
                           const std::vector<MapPoint *>   &vMPs,
                           cv::Mat                         &im);

    /*!
     * @brief        Finds the dominant plane among the tracked map points
     *               with a RANSAC fit.
     *
     *               Only map points seen more than five times are used.
     *
     * @param[in]    Tcw
     *               Current camera pose, mapping world-frame points into the
     *               camera frame (4x4 float).
     * @param[in]    vMPs
     *               Tracked map points, null where a keypoint has none.
     * @param[in]    iterations
     *               Number of random three-point samples to try.
     *
     * @return       A newly allocated plane that the caller must delete, or
     *               null when fewer than 50 usable map points exist.
     */
    Plane *DetectPlane(const cv::Mat                  Tcw,
                       const std::vector<MapPoint *> &vMPs,
                       const int                      iterations = 50);

    /*!
     * @brief        Camera frame rate, hertz.
     */
    float mFPS;

    /*!
     * @brief        Window refresh period, milliseconds (1000 / mFPS).
     */
    float mT;

    /*!
     * @brief        Focal length along x, pixels.
     */
    float fx;

    /*!
     * @brief        Focal length along y, pixels.
     */
    float fy;

    /*!
     * @brief        Principal point x, pixels.
     */
    float cx;

    /*!
     * @brief        Principal point y, pixels.
     */
    float cy;

    /*!
     * @brief        Guards the five last-result members below; the ROS callback
     *               thread writes them and the viewer thread reads them.
     */
    std::mutex mMutexPoseImage;

    /*!
     * @brief        Latest camera pose, mapping world-frame points into the
     *               camera frame (4x4 float).
     */
    cv::Mat mTcw;

    /*!
     * @brief        Latest camera image.
     */
    cv::Mat mImage;

    /*!
     * @brief        Latest SLAM tracking state (1 not initialised, 2 tracking,
     *               3 lost).
     */
    int mStatus;

    /*!
     * @brief        Undistorted keypoints of the latest image.
     */
    std::vector<cv::KeyPoint> mvKeys;

    /*!
     * @brief        Map point tracked by each keypoint of the latest image,
     *               null where none; borrowed from the map.
     */
    std::vector<MapPoint *> mvMPs;
};

} // namespace ORB_SLAM3

#endif // VIEWERAR_H

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

#include "LocalMapping.h"

#include "GeometricTools.h"
#include "ORBmatcher.h"

namespace vs_graphs
{
namespace core
{

void LocalMapping::createNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 10;
    // For stereo inertial case
    if (monocular)
        nn = 30;
    vector<KeyFrame *> vpNeighKFs =
        p_currentKeyFrame->getBestCovisibilityKeyFrames(nn);

    if (inertial)
    {
        KeyFrame *pKF   = p_currentKeyFrame;
        int       count = 0;
        // nn is the fixed covisibility budget set above (10, or 30 when
        // monocular), so it is always positive here.
        while ((vpNeighKFs.size() <= static_cast<std::size_t>(nn)) &&
               (pKF->p_prevKF) && (count++ < nn))
        {
            vector<KeyFrame *>::iterator it =
                std::find(vpNeighKFs.begin(), vpNeighKFs.end(), pKF->p_prevKF);
            if (it == vpNeighKFs.end())
                vpNeighKFs.push_back(pKF->p_prevKF);
            pKF = pKF->p_prevKF;
        }
    }

    float th = 0.6f;

    ORBmatcher matcher(th, false);

    Sophus::SE3<float>         sophTcw1 = p_currentKeyFrame->getPose();
    Eigen::Matrix<float, 3, 4> eigTcw1  = sophTcw1.matrix3x4();
    Eigen::Matrix<float, 3, 3> Rcw1     = eigTcw1.block<3, 3>(0, 0);
    Eigen::Matrix<float, 3, 3> Rwc1     = Rcw1.transpose();
    Eigen::Vector3f            tcw1     = sophTcw1.translation();
    Eigen::Vector3f            Ow1      = p_currentKeyFrame->getCameraCenter();

    const float &fx1 = p_currentKeyFrame->fx;
    const float &fy1 = p_currentKeyFrame->fy;
    const float &cx1 = p_currentKeyFrame->cx;
    const float &cy1 = p_currentKeyFrame->cy;

    const float ratioFactor         = 1.5f * p_currentKeyFrame->scaleFactor;
    int         countStereo         = 0;
    int         countStereoGoodProj = 0;
    int         countStereoAttempt  = 0;
    int         totalStereoPts      = 0;
    // Search matches with epipolar restriction and triangulate
    for (size_t i = 0; i < vpNeighKFs.size(); i++)
    {
        if (i > 0 && checkNewKeyFrames())
            return;

        KeyFrame *pKF2 = vpNeighKFs[i];

        camera_models::geometriccamera::GeometricCamera
            *pCamera1 = p_currentKeyFrame->p_camera,
            *pCamera2 = pKF2->p_camera;

        // Check first that baseline is not too short
        Eigen::Vector3f Ow2       = pKF2->getCameraCenter();
        Eigen::Vector3f vBaseline = Ow2 - Ow1;
        const float     baseline  = vBaseline.norm();

        if (!monocular)
        {
            if (baseline < pKF2->mb)
                continue;
        }
        else
        {
            const float medianDepthKF2     = pKF2->computeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline / medianDepthKF2;

            if (ratioBaselineDepth < 0.01)
                continue;
        }

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t, size_t>> vMatchedIndices;
        bool                         bCoarse = inertial &&
                       p_tracker->state == Tracking::RECENTLY_LOST &&
                       p_currentKeyFrame->getMap()->getInertialBA2();

        matcher.searchForTriangulation(p_currentKeyFrame,
                                       pKF2,
                                       vMatchedIndices,
                                       false,
                                       bCoarse);

        Sophus::SE3<float>         sophTcw2 = pKF2->getPose();
        Eigen::Matrix<float, 3, 4> eigTcw2  = sophTcw2.matrix3x4();
        Eigen::Matrix<float, 3, 3> Rcw2     = eigTcw2.block<3, 3>(0, 0);
        Eigen::Matrix<float, 3, 3> Rwc2     = Rcw2.transpose();
        Eigen::Vector3f            tcw2     = sophTcw2.translation();

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for (int ikp = 0; ikp < nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 =
                (p_currentKeyFrame->Nleft == -1)
                    ? p_currentKeyFrame->keyPointsUndistorted[idx1]
                : (idx1 < p_currentKeyFrame->Nleft)
                    ? p_currentKeyFrame->keyPoints[idx1]
                    : p_currentKeyFrame
                          ->keyPointsRight[idx1 - p_currentKeyFrame->Nleft];
            const float kp1_ur = p_currentKeyFrame->uRight[idx1];
            bool bStereo1      = (!p_currentKeyFrame->p_camera2 && kp1_ur >= 0);
            const bool bRight1 = (p_currentKeyFrame->Nleft == -1 ||
                                  idx1 < p_currentKeyFrame->Nleft)
                                     ? false
                                     : true;

            const cv::KeyPoint &kp2 =
                (pKF2->Nleft == -1) ? pKF2->keyPointsUndistorted[idx2]
                : (idx2 < pKF2->Nleft)
                    ? pKF2->keyPoints[idx2]
                    : pKF2->keyPointsRight[idx2 - pKF2->Nleft];

            const float kp2_ur   = pKF2->uRight[idx2];
            bool        bStereo2 = (!pKF2->p_camera2 && kp2_ur >= 0);
            const bool  bRight2 =
                (pKF2->Nleft == -1 || idx2 < pKF2->Nleft) ? false : true;

            if (p_currentKeyFrame->p_camera2 && pKF2->p_camera2)
            {
                if (bRight1 && bRight2)
                {
                    sophTcw1 = p_currentKeyFrame->getRightPose();
                    Ow1      = p_currentKeyFrame->getRightCameraCenter();

                    sophTcw2 = pKF2->getRightPose();
                    Ow2      = pKF2->getRightCameraCenter();

                    pCamera1 = p_currentKeyFrame->p_camera2;
                    pCamera2 = pKF2->p_camera2;
                }
                else if (bRight1 && !bRight2)
                {
                    sophTcw1 = p_currentKeyFrame->getRightPose();
                    Ow1      = p_currentKeyFrame->getRightCameraCenter();

                    sophTcw2 = pKF2->getPose();
                    Ow2      = pKF2->getCameraCenter();

                    pCamera1 = p_currentKeyFrame->p_camera2;
                    pCamera2 = pKF2->p_camera;
                }
                else if (!bRight1 && bRight2)
                {
                    sophTcw1 = p_currentKeyFrame->getPose();
                    Ow1      = p_currentKeyFrame->getCameraCenter();

                    sophTcw2 = pKF2->getRightPose();
                    Ow2      = pKF2->getRightCameraCenter();

                    pCamera1 = p_currentKeyFrame->p_camera;
                    pCamera2 = pKF2->p_camera2;
                }
                else
                {
                    sophTcw1 = p_currentKeyFrame->getPose();
                    Ow1      = p_currentKeyFrame->getCameraCenter();

                    sophTcw2 = pKF2->getPose();
                    Ow2      = pKF2->getCameraCenter();

                    pCamera1 = p_currentKeyFrame->p_camera;
                    pCamera2 = pKF2->p_camera;
                }
                eigTcw1 = sophTcw1.matrix3x4();
                Rcw1    = eigTcw1.block<3, 3>(0, 0);
                Rwc1    = Rcw1.transpose();
                tcw1    = sophTcw1.translation();

                eigTcw2 = sophTcw2.matrix3x4();
                Rcw2    = eigTcw2.block<3, 3>(0, 0);
                Rwc2    = Rcw2.transpose();
                tcw2    = sophTcw2.translation();
            }

            // Check parallax between rays
            Eigen::Vector3f xn1 = pCamera1->unprojectEig(kp1.pt);
            Eigen::Vector3f xn2 = pCamera2->unprojectEig(kp2.pt);

            Eigen::Vector3f ray1 = Rwc1 * xn1;
            Eigen::Vector3f ray2 = Rwc2 * xn2;
            const float     cosParallaxRays =
                ray1.dot(ray2) / (ray1.norm() * ray2.norm());

            float cosParallaxStereo  = cosParallaxRays + 1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if (bStereo1)
                cosParallaxStereo1 =
                    cos(2 * atan2(p_currentKeyFrame->mb / 2,
                                  p_currentKeyFrame->depths[idx1]));
            else if (bStereo2)
                cosParallaxStereo2 =
                    cos(2 * atan2(pKF2->mb / 2, pKF2->depths[idx2]));

            if (bStereo1 || bStereo2)
                totalStereoPts++;

            cosParallaxStereo = min(cosParallaxStereo1, cosParallaxStereo2);

            Eigen::Vector3f x3D;

            bool goodProj     = false;
            bool bPointStereo = false;
            if (cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0 &&
                (bStereo1 || bStereo2 ||
                 (cosParallaxRays < 0.9996 && inertial) ||
                 (cosParallaxRays < 0.9998 && !inertial)))
            {
                goodProj = GeometricTools::triangulate(xn1,
                                                       xn2,
                                                       eigTcw1,
                                                       eigTcw2,
                                                       x3D);
                if (!goodProj)
                    continue;
            }
            else if (bStereo1 && cosParallaxStereo1 < cosParallaxStereo2)
            {
                countStereoAttempt++;
                bPointStereo = true;
                goodProj     = p_currentKeyFrame->unprojectStereo(idx1, x3D);
            }
            else if (bStereo2 && cosParallaxStereo2 < cosParallaxStereo1)
            {
                countStereoAttempt++;
                bPointStereo = true;
                goodProj     = pKF2->unprojectStereo(idx2, x3D);
            }
            else
            {
                continue; // No stereo and very low parallax
            }

            if (goodProj && bPointStereo)
                countStereoGoodProj++;

            if (!goodProj)
                continue;

            // Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3D) + tcw1(2);
            if (z1 <= 0)
                continue;

            float z2 = Rcw2.row(2).dot(x3D) + tcw2(2);
            if (z2 <= 0)
                continue;

            // Check reprojection error in first keyframe
            const float &sigmaSquare1 =
                p_currentKeyFrame->levelSigmaSquared[kp1.octave];
            const float x1    = Rcw1.row(0).dot(x3D) + tcw1(0);
            const float y1    = Rcw1.row(1).dot(x3D) + tcw1(1);
            const float invz1 = 1.0 / z1;

            if (!bStereo1)
            {
                cv::Point2f uv1   = pCamera1->project(cv::Point3f(x1, y1, z1));
                float       errX1 = uv1.x - kp1.pt.x;
                float       errY1 = uv1.y - kp1.pt.y;

                if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaSquare1)
                    continue;
            }
            else
            {
                float u1      = fx1 * x1 * invz1 + cx1;
                float u1_r    = u1 - p_currentKeyFrame->mbf * invz1;
                float v1      = fy1 * y1 * invz1 + cy1;
                float errX1   = u1 - kp1.pt.x;
                float errY1   = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if ((errX1 * errX1 + errY1 * errY1 + errX1_r * errX1_r) >
                    7.8 * sigmaSquare1)
                    continue;
            }

            // Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->levelSigmaSquared[kp2.octave];
            const float x2           = Rcw2.row(0).dot(x3D) + tcw2(0);
            const float y2           = Rcw2.row(1).dot(x3D) + tcw2(1);
            const float invz2        = 1.0 / z2;
            if (!bStereo2)
            {
                cv::Point2f uv2   = pCamera2->project(cv::Point3f(x2, y2, z2));
                float       errX2 = uv2.x - kp2.pt.x;
                float       errY2 = uv2.y - kp2.pt.y;
                if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaSquare2)
                    continue;
            }
            else
            {
                float u2      = fx2 * x2 * invz2 + cx2;
                float u2_r    = u2 - p_currentKeyFrame->mbf * invz2;
                float v2      = fy2 * y2 * invz2 + cy2;
                float errX2   = u2 - kp2.pt.x;
                float errY2   = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if ((errX2 * errX2 + errY2 * errY2 + errX2_r * errX2_r) >
                    7.8 * sigmaSquare2)
                    continue;
            }

            // Check scale consistency
            Eigen::Vector3f normal1 = x3D - Ow1;
            float           dist1   = normal1.norm();

            Eigen::Vector3f normal2 = x3D - Ow2;
            float           dist2   = normal2.norm();

            if (dist1 == 0 || dist2 == 0)
                continue;

            if (farPoints && (dist1 >= farPointsThreshold ||
                              dist2 >= farPointsThreshold)) // MODIFICATION
                continue;

            const float ratioDist = dist2 / dist1;
            const float ratioOctave =
                p_currentKeyFrame->scaleFactors[kp1.octave] /
                pKF2->scaleFactors[kp2.octave];

            if (ratioDist * ratioFactor < ratioOctave ||
                ratioDist > ratioOctave * ratioFactor)
                continue;

            // Triangulation is succesfull
            MapPoint *pMP =
                new MapPoint(x3D, p_currentKeyFrame, p_atlas->getCurrentMap());
            if (bPointStereo)
                countStereo++;

            pMP->addObservation(p_currentKeyFrame, idx1);
            pMP->addObservation(pKF2, idx2);

            p_currentKeyFrame->addMapPoint(pMP, idx1);
            pKF2->addMapPoint(pMP, idx2);

            pMP->computeDistinctiveDescriptors();

            pMP->updateNormalAndDepth();

            p_atlas->addMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);
        }
    }
}

} // namespace core
} // namespace vs_graphs

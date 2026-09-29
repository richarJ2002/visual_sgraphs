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

#include "Frame.h"

#include "CameraModels/GeometricCamera/objects/GeometricCamera.h"
#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"
#include "CameraModels/Pinhole/objects/Pinhole.h"
#include "G2oTypes.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "StereoMatchOutlierRejection.h"
#include "Utils/Converter/objects/Converter.h"

#include <rclcpp/logging.hpp>
#include <thread>

namespace vs_graphs
{
namespace core
{

FrameStatus Frame::isInFrustum(MapPoint *p_mapPoint_inout,
                               float     viewingCosLimit_in,
                               bool     &isInFrustum_out)
{
    if (leftKeyPointCount == -1)
    {
        p_mapPoint_inout->isTrackedInView = false;
        p_mapPoint_inout->trackProjX      = -1;
        p_mapPoint_inout->trackProjY      = -1;

        // 3D in absolute coordinates
        Eigen::Matrix<float, 3, 1> P{};
        if (p_mapPoint_inout->getWorldPos(P) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        // 3D in camera coordinates
        const Eigen::Matrix<float, 3, 1> Pc = rotationRcw * P + translationTcw;
        const float                      Pc_dist = Pc.norm();

        // Check positive depth
        const float &PcZ  = Pc(2);
        const float  invz = 1.0f / PcZ;
        if (PcZ < 0.0f)
        {
            isInFrustum_out = false;
            return FrameStatus::FRAME_STATUS_SUCCESS;
        }

        const Eigen::Vector2f uv = p_camera->project(Pc);

        if (uv(0) < gridMinX || uv(0) > gridMaxX)
        {
            isInFrustum_out = false;
            return FrameStatus::FRAME_STATUS_SUCCESS;
        }
        if (uv(1) < gridMinY || uv(1) > gridMaxY)
        {
            isInFrustum_out = false;
            return FrameStatus::FRAME_STATUS_SUCCESS;
        }

        p_mapPoint_inout->trackProjX = uv(0);
        p_mapPoint_inout->trackProjY = uv(1);

        // Check distance is in the scale invariance region of the MapPoint
        float maximumDistance{};
        if (p_mapPoint_inout->getMaxDistanceInvariance(maximumDistance) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getMaxDistanceInvariance returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        float minimumDistance{};
        if (p_mapPoint_inout->getMinDistanceInvariance(minimumDistance) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getMinDistanceInvariance returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        const Eigen::Vector3f PO       = P - centerOw;
        const float           distance = PO.norm();

        if (distance < minimumDistance || distance > maximumDistance)
        {
            isInFrustum_out = false;
            return FrameStatus::FRAME_STATUS_SUCCESS;
        }

        // Check viewing angle
        Eigen::Vector3f Pn{};
        if (p_mapPoint_inout->getNormal(Pn) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getNormal returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        const float viewCos = PO.dot(Pn) / distance;

        if (viewCos < viewingCosLimit_in)
        {
            isInFrustum_out = false;
            return FrameStatus::FRAME_STATUS_SUCCESS;
        }

        // Predict scale in the image
        int predictedLevelCount{};
        if (p_mapPoint_inout->predictScale(distance,
                                           this,
                                           predictedLevelCount) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: predictScale returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        // Data used by the tracking
        p_mapPoint_inout->isTrackedInView = true;
        p_mapPoint_inout->trackProjX      = uv(0);
        p_mapPoint_inout->trackProjXR     = uv(0) - mbf * invz;

        p_mapPoint_inout->trackDepth = Pc_dist;

        p_mapPoint_inout->trackProjY      = uv(1);
        p_mapPoint_inout->trackScaleLevel = predictedLevelCount;
        p_mapPoint_inout->trackViewCos    = viewCos;

        isInFrustum_out = true;
        return FrameStatus::FRAME_STATUS_SUCCESS;
    }
    else
    {
        p_mapPoint_inout->isTrackedInView      = false;
        p_mapPoint_inout->isTrackedInRightView = false;
        p_mapPoint_inout->trackScaleLevel      = -1;
        p_mapPoint_inout->trackScaleLevelR     = -1;

        bool isInFrustumChecks2{};
        if (isInFrustumChecks(p_mapPoint_inout,
                              viewingCosLimit_in,
                              isInFrustumChecks2) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isInFrustumChecks returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        p_mapPoint_inout->isTrackedInView = isInFrustumChecks2;
        bool isInFrustumChecks3{};
        if (isInFrustumChecks(p_mapPoint_inout,
                              viewingCosLimit_in,
                              isInFrustumChecks3,
                              true) != FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isInFrustumChecks returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        p_mapPoint_inout->isTrackedInRightView = isInFrustumChecks3;

        isInFrustum_out = p_mapPoint_inout->isTrackedInView ||
                          p_mapPoint_inout->isTrackedInRightView;
        return FrameStatus::FRAME_STATUS_SUCCESS;
    }
}

} // namespace core
} // namespace vs_graphs

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

#include <thread>

namespace vs_graphs
{
namespace core
{

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    if (Nleft == -1)
    {
        pMP->trackInView = false;
        pMP->trackProjX  = -1;
        pMP->trackProjY  = -1;

        // 3D in absolute coordinates
        Eigen::Matrix<float, 3, 1> P = pMP->getWorldPos();

        // 3D in camera coordinates
        const Eigen::Matrix<float, 3, 1> Pc = rotationRcw * P + translationTcw;
        const float                      Pc_dist = Pc.norm();

        // Check positive depth
        const float &PcZ  = Pc(2);
        const float  invz = 1.0f / PcZ;
        if (PcZ < 0.0f)
            return false;

        const Eigen::Vector2f uv = p_camera->project(Pc);

        if (uv(0) < gridMinX || uv(0) > gridMaxX)
            return false;
        if (uv(1) < gridMinY || uv(1) > gridMaxY)
            return false;

        pMP->trackProjX = uv(0);
        pMP->trackProjY = uv(1);

        // Check distance is in the scale invariance region of the MapPoint
        const float           maxDistance = pMP->getMaxDistanceInvariance();
        const float           minDistance = pMP->getMinDistanceInvariance();
        const Eigen::Vector3f PO          = P - centerOw;
        const float           dist        = PO.norm();

        if (dist < minDistance || dist > maxDistance)
            return false;

        // Check viewing angle
        Eigen::Vector3f Pn = pMP->getNormal();

        const float viewCos = PO.dot(Pn) / dist;

        if (viewCos < viewingCosLimit)
            return false;

        // Predict scale in the image
        const int nPredictedLevel = pMP->predictScale(dist, this);

        // Data used by the tracking
        pMP->trackInView = true;
        pMP->trackProjX  = uv(0);
        pMP->trackProjXR = uv(0) - mbf * invz;

        pMP->trackDepth = Pc_dist;

        pMP->trackProjY      = uv(1);
        pMP->trackScaleLevel = nPredictedLevel;
        pMP->trackViewCos    = viewCos;

        return true;
    }
    else
    {
        pMP->trackInView      = false;
        pMP->trackInViewR     = false;
        pMP->trackScaleLevel  = -1;
        pMP->trackScaleLevelR = -1;

        pMP->trackInView  = isInFrustumChecks(pMP, viewingCosLimit);
        pMP->trackInViewR = isInFrustumChecks(pMP, viewingCosLimit, true);

        return pMP->trackInView || pMP->trackInViewR;
    }
}

} // namespace core
} // namespace vs_graphs

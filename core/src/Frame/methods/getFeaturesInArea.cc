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

FrameStatus Frame::getFeaturesInArea(const float         &x_in,
                                     const float         &y_in,
                                     const float         &r_in,
                                     std::vector<size_t> &featuresInArea_out,
                                     const int            minimumLevel_in,
                                     const int            maximumLevel_in,
                                     const bool isRightCamera_in) const
{
    vector<size_t> indices;
    indices.reserve(keyPointCount);

    float factorX = r_in;
    float factorY = r_in;

    const int minimumCellXCount =
        max(0,
            (int)floor((x_in - gridMinX - factorX) * gridElementWidthInverse));
    if (minimumCellXCount >= FRAME_GRID_COLS)
    {
        featuresInArea_out = indices;
        return FrameStatus::FRAME_STATUS_SUCCESS;
    }

    const int maximumCellXCount =
        min((int)FRAME_GRID_COLS - 1,
            (int)ceil((x_in - gridMinX + factorX) * gridElementWidthInverse));
    if (maximumCellXCount < 0)
    {
        featuresInArea_out = indices;
        return FrameStatus::FRAME_STATUS_SUCCESS;
    }

    const int minimumCellYCount =
        max(0,
            (int)floor((y_in - gridMinY - factorY) * gridElementHeightInverse));
    if (minimumCellYCount >= FRAME_GRID_ROWS)
    {
        featuresInArea_out = indices;
        return FrameStatus::FRAME_STATUS_SUCCESS;
    }

    const int maximumCellYCount =
        min((int)FRAME_GRID_ROWS - 1,
            (int)ceil((y_in - gridMinY + factorY) * gridElementHeightInverse));
    if (maximumCellYCount < 0)
    {
        featuresInArea_out = indices;
        return FrameStatus::FRAME_STATUS_SUCCESS;
    }

    const bool shouldCheckLevels =
        (minimumLevel_in > 0) || (maximumLevel_in >= 0);

    for (int ix = minimumCellXCount; ix <= maximumCellXCount; ix++)
    {
        for (int iy = minimumCellYCount; iy <= maximumCellYCount; iy++)
        {
            const vector<size_t> cells =
                (!isRightCamera_in) ? grid[ix][iy] : gridRight[ix][iy];
            if (cells.empty())
                continue;

            for (size_t cellFeatureIndex = 0, jend = cells.size();
                 cellFeatureIndex < jend;
                 cellFeatureIndex++)
            {
                const cv::KeyPoint &keyPointUn =
                    (leftKeyPointCount == -1)
                        ? keyPointsUndistorted[cells[cellFeatureIndex]]
                    : (!isRightCamera_in)
                        ? keyPoints[cells[cellFeatureIndex]]
                        : keyPointsRight[cells[cellFeatureIndex]];
                if (shouldCheckLevels)
                {
                    if (keyPointUn.octave < minimumLevel_in)
                        continue;
                    if (maximumLevel_in >= 0)
                        if (keyPointUn.octave > maximumLevel_in)
                            continue;
                }

                const float distx = keyPointUn.pt.x - x_in;
                const float disty = keyPointUn.pt.y - y_in;

                if (fabs(distx) < factorX && fabs(disty) < factorY)
                    indices.push_back(cells[cellFeatureIndex]);
            }
        }
    }

    featuresInArea_out = indices;
    return FrameStatus::FRAME_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

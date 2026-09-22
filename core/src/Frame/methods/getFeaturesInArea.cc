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

vector<size_t> Frame::getFeaturesInArea(const float &x,
                                        const float &y,
                                        const float &r,
                                        const int    minLevel,
                                        const int    maxLevel,
                                        const bool   bRight) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    float factorX = r;
    float factorY = r;

    const int nMinCellX =
        max(0, (int)floor((x - gridMinX - factorX) * gridElementWidthInverse));
    if (nMinCellX >= FRAME_GRID_COLS)
    {
        return vIndices;
    }

    const int nMaxCellX =
        min((int)FRAME_GRID_COLS - 1,
            (int)ceil((x - gridMinX + factorX) * gridElementWidthInverse));
    if (nMaxCellX < 0)
    {
        return vIndices;
    }

    const int nMinCellY =
        max(0, (int)floor((y - gridMinY - factorY) * gridElementHeightInverse));
    if (nMinCellY >= FRAME_GRID_ROWS)
    {
        return vIndices;
    }

    const int nMaxCellY =
        min((int)FRAME_GRID_ROWS - 1,
            (int)ceil((y - gridMinY + factorY) * gridElementHeightInverse));
    if (nMaxCellY < 0)
    {
        return vIndices;
    }

    const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

    for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
    {
        for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
        {
            const vector<size_t> vCell =
                (!bRight) ? grid[ix][iy] : gridRight[ix][iy];
            if (vCell.empty())
                continue;

            for (size_t j = 0, jend = vCell.size(); j < jend; j++)
            {
                const cv::KeyPoint &kpUn =
                    (Nleft == -1) ? keyPointsUndistorted[vCell[j]]
                    : (!bRight)   ? keyPoints[vCell[j]]
                                  : keyPointsRight[vCell[j]];
                if (bCheckLevels)
                {
                    if (kpUn.octave < minLevel)
                        continue;
                    if (maxLevel >= 0)
                        if (kpUn.octave > maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x - x;
                const float disty = kpUn.pt.y - y;

                if (fabs(distx) < factorX && fabs(disty) < factorY)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

} // namespace core
} // namespace vs_graphs

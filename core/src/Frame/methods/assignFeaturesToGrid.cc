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

void Frame::assignFeaturesToGrid()
{
    // Fill matrix with points
    const int nCells = FRAME_GRID_COLS * FRAME_GRID_ROWS;

    int nReserve = 0.5f * N / (nCells);

    for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
        for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
        {
            grid[i][j].reserve(nReserve);
            if (Nleft != -1)
            {
                gridRight[i][j].reserve(nReserve);
            }
        }

    for (int i = 0; i < N; i++)
    {
        const cv::KeyPoint &kp = (Nleft == -1) ? keyPointsUndistorted[i]
                                 : (i < Nleft) ? keyPoints[i]
                                               : keyPointsRight[i - Nleft];

        int nGridPosX, nGridPosY;
        if (isPositionInGrid(kp, nGridPosX, nGridPosY))
        {
            if (Nleft == -1 || i < Nleft)
                grid[nGridPosX][nGridPosY].push_back(i);
            else
                gridRight[nGridPosX][nGridPosY].push_back(i - Nleft);
        }
    }
}

} // namespace core
} // namespace vs_graphs

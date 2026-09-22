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

#include "KeyFrame.h"

#include "ImuTypes.h"
#include "Utils/Converter/objects/Converter.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

vector<size_t> KeyFrame::getFeaturesInArea(const float &x,
                                           const float &y,
                                           const float &r,
                                           const bool   bRight) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    float factorX = r;
    float factorY = r;

    const int nMinCellX =
        max(0, (int)floor((x - gridMinX - factorX) * gridElementWidthInverse));
    if (nMinCellX >= gridCols)
        return vIndices;

    const int nMaxCellX =
        min((int)gridCols - 1,
            (int)ceil((x - gridMinX + factorX) * gridElementWidthInverse));
    if (nMaxCellX < 0)
        return vIndices;

    const int nMinCellY =
        max(0, (int)floor((y - gridMinY - factorY) * gridElementHeightInverse));
    if (nMinCellY >= gridRows)
        return vIndices;

    const int nMaxCellY =
        min((int)gridRows - 1,
            (int)ceil((y - gridMinY + factorY) * gridElementHeightInverse));
    if (nMaxCellY < 0)
        return vIndices;

    for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
    {
        for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
        {
            const vector<size_t> vCell =
                (!bRight) ? grid[ix][iy] : gridRight[ix][iy];
            for (size_t j = 0, jend = vCell.size(); j < jend; j++)
            {
                const cv::KeyPoint &kpUn =
                    (Nleft == -1) ? keyPointsUndistorted[vCell[j]]
                    : (!bRight)   ? keyPoints[vCell[j]]
                                  : keyPointsRight[vCell[j]];
                const float distx = kpUn.pt.x - x;
                const float disty = kpUn.pt.y - y;

                if (fabs(distx) < r && fabs(disty) < r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

} // namespace core
} // namespace vs_graphs

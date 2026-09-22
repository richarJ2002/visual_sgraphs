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

#include "MapPoint.h"

#include "ORBmatcher.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

void MapPoint::computeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame *, tuple<int, int>> observedKeyFrames;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if (mbBad)
            return;
        observedKeyFrames = observations;
    }

    if (observedKeyFrames.empty())
        return;

    vDescriptors.reserve(observedKeyFrames.size());

    for (map<KeyFrame *, tuple<int, int>>::iterator
             mit  = observedKeyFrames.begin(),
             mend = observedKeyFrames.end();
         mit != mend;
         mit++)
    {
        KeyFrame *pKF = mit->first;

        if (!pKF->isBad())
        {
            tuple<int, int> indexes = mit->second;
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            if (leftIndex != -1)
            {
                vDescriptors.push_back(pKF->descriptors.row(leftIndex));
            }
            if (rightIndex != -1)
            {
                vDescriptors.push_back(pKF->descriptors.row(rightIndex));
            }
        }
    }

    if (vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    // Symmetric N x N distance matrix held row-major, so row i occupies the
    // contiguous range [i * N, i * N + N).
    std::vector<float> Distances(N * N);
    for (size_t i = 0; i < N; i++)
    {
        Distances[i * N + i] = 0;
        for (size_t j = i + 1; j < N; j++)
        {
            int distij = ORBmatcher::computeDescriptorDistance(vDescriptors[i],
                                                               vDescriptors[j]);
            Distances[i * N + j] = distij;
            Distances[j * N + i] = distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx    = 0;
    for (size_t i = 0; i < N; i++)
    {
        const float *p_row = &Distances[i * N];
        vector<int>  vDists(p_row, p_row + N);
        sort(vDists.begin(), vDists.end());
        int median = vDists[0.5 * (N - 1)];

        if (median < BestMedian)
        {
            BestMedian = median;
            BestIdx    = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        descriptor = vDescriptors[BestIdx].clone();
    }
}

} // namespace core
} // namespace vs_graphs

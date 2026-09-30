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
 * @file            computeDistinctiveDescriptors.cc
 *
 * @brief           Implements MapPoint::computeDistinctiveDescriptors(),
 *                  declared in MapPoint.h.
 */

#include "MapPoint.h"

#include "ORBmatcher.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

MapPointStatus MapPoint::computeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    std::vector<cv::Mat> descriptors;

    std::map<KeyFrame *, std::tuple<int, int>> observedKeyFrames;

    {
        std::unique_lock<std::mutex> lock1(featuresMutex);
        if (isFlaggedBad)
            return MapPointStatus::MAP_POINT_STATUS_SUCCESS;
        observedKeyFrames = observations;
    }

    if (observedKeyFrames.empty())
        return MapPointStatus::MAP_POINT_STATUS_SUCCESS;

    descriptors.reserve(observedKeyFrames.size());

    for (std::map<KeyFrame *, std::tuple<int, int>>::iterator
             mit  = observedKeyFrames.begin(),
             mend = observedKeyFrames.end();
         mit != mend;
         mit++)
    {
        KeyFrame *p_keyFrame = mit->first;

        bool keyFrameIsBad{};
        if (p_keyFrame->isBad(keyFrameIsBad) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!keyFrameIsBad)
        {
            std::tuple<int, int> indexes   = mit->second;
            int                  leftIndex = std::get<0>(indexes),
                rightIndex                 = std::get<1>(indexes);

            if (leftIndex != -1)
            {
                descriptors.push_back(p_keyFrame->descriptors.row(leftIndex));
            }
            if (rightIndex != -1)
            {
                descriptors.push_back(p_keyFrame->descriptors.row(rightIndex));
            }
        }
    }

    if (descriptors.empty())
        return MapPointStatus::MAP_POINT_STATUS_SUCCESS;

    // Compute distances between them
    const size_t N = descriptors.size();

    // Symmetric N x N distance matrix held row-major, so row i occupies the
    // contiguous range [i * N, i * N + N).
    std::vector<float> distances(N * N);
    for (size_t keyPointIndex = 0; keyPointIndex < N; keyPointIndex++)
    {
        distances[keyPointIndex * N + keyPointIndex] = 0;
        for (size_t secondDescriptorIndex = keyPointIndex + 1;
             secondDescriptorIndex < N;
             secondDescriptorIndex++)
        {
            int distij{};
            if (ORBmatcher::computeDescriptorDistance(
                    descriptors[keyPointIndex],
                    descriptors[secondDescriptorIndex],
                    distij) != ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: computeDescriptorDistance returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            distances[keyPointIndex * N + secondDescriptorIndex] = distij;
            distances[secondDescriptorIndex * N + keyPointIndex] = distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int bestMedian = INT_MAX;
    int bestIndex  = 0;
    for (size_t keyPointIndex = 0; keyPointIndex < N; keyPointIndex++)
    {
        const float     *p_row = &distances[keyPointIndex * N];
        std::vector<int> dists(p_row, p_row + N);
        std::sort(dists.begin(), dists.end());
        int median = dists[0.5 * (N - 1)];

        if (median < bestMedian)
        {
            bestMedian = median;
            bestIndex  = keyPointIndex;
        }
    }

    {
        std::unique_lock<std::mutex> lock(featuresMutex);
        descriptor = descriptors[bestIndex].clone();
    }

    return MapPointStatus::MAP_POINT_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

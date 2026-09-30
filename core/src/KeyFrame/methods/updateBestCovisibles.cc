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
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

KeyFrameStatus KeyFrame::updateBestCovisibles()
{
    std::unique_lock<std::mutex>            lock(connectionsMutex);
    std::vector<std::pair<int, KeyFrame *>> pairs;
    pairs.reserve(connectedKeyFrameWeights.size());
    for (std::map<KeyFrame *, int>::iterator
             mit  = connectedKeyFrameWeights.begin(),
             mend = connectedKeyFrameWeights.end();
         mit != mend;
         mit++)
        pairs.push_back(std::make_pair(mit->second, mit->first));

    std::sort(pairs.begin(), pairs.end());
    std::list<KeyFrame *> keyFrames;
    std::list<int>        weights;
    for (size_t pairIndex = 0, iend = pairs.size(); pairIndex < iend;
         pairIndex++)
    {
        if (pairs[pairIndex].second != nullptr)
        {
            bool isBad2{};
            if (pairs[pairIndex].second->isBad(isBad2) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!isBad2)
            {
                keyFrames.push_front(pairs[pairIndex].second);
                weights.push_front(pairs[pairIndex].first);
            }
        }
    }

    orderedConnectedKeyFrames =
        std::vector<KeyFrame *>(keyFrames.begin(), keyFrames.end());
    orderedWeights = std::vector<int>(weights.begin(), weights.end());

    return KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

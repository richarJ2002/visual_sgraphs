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
 * @file            release.cc
 *
 * @brief           Implements LocalMapping::release(), declared in
 *                  LocalMapping.h.
 */

#include "LocalMapping.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

LocalMappingStatus LocalMapping::release()
{
    std::unique_lock<std::mutex> stopLock(stopMutex);
    std::unique_lock<std::mutex> finishLock(finishMutex);
    if (hasFinished)
        return LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS;
    hasStopped      = false;
    isStopRequested = false;
    for (std::list<KeyFrame *>::iterator newKeyFrameIt  = newKeyFrames.begin(),
                                         newKeyFrameEnd = newKeyFrames.end();
         newKeyFrameIt != newKeyFrameEnd;
         newKeyFrameIt++)
        delete *newKeyFrameIt;
    newKeyFrames.clear();

    return LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

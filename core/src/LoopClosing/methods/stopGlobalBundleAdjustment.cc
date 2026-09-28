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

#include "LoopClosing.h"

#include <mutex>
#include <thread>

namespace vs_graphs
{
namespace core
{

bool LoopClosing::stopGlobalBundleAdjustment()
{
    std::thread *p_globalBundleAdjustmentThread = nullptr;
    bool         optimizationWasRunning         = false;

    {
        std::unique_lock<std::mutex> globalBundleAdjustmentLock(gbaMutex);

        optimizationWasRunning = isGbaRunning;

        if (optimizationWasRunning)
        {
            /* Invalidate the result before waiting for the worker to finish. */
            ++fullBundleAdjustmentIndex;
            isGlobalBundleAdjustmentStopRequested.store(
                true,
                std::memory_order_release);
        }

        /*
         * Move ownership out while holding the state mutex. The mutex must be
         * released before join() because the worker takes it while finishing.
         */
        p_globalBundleAdjustmentThread = p_threadGBA;
        p_threadGBA                    = nullptr;
    }

    if (p_globalBundleAdjustmentThread != nullptr)
    {
        if (p_globalBundleAdjustmentThread->joinable())
        {
            p_globalBundleAdjustmentThread->join();
        }

        delete p_globalBundleAdjustmentThread;
    }

    {
        std::unique_lock<std::mutex> globalBundleAdjustmentLock(gbaMutex);
        isGbaRunning   = false;
        hasGbaFinished = true;
    }

    return optimizationWasRunning;
}

} // namespace core
} // namespace vs_graphs

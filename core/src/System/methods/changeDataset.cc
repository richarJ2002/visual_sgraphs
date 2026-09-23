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
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "System.h"

namespace vs_graphs
{
namespace core
{

void System::changeDataset()
{
    if (p_atlas->getCurrentMap()->getKeyFrameCount() < 12)
    {
        reportResetAttribution(ResetCause::DATASET_CHANGE_SMALL_MAP,
                               ResetAction::RESET_ACTIVE_MAP_EXECUTION);
        p_tracker->resetActiveMap();
        resetCount.fetch_add(1U, std::memory_order_relaxed);
    }
    else
    {
        reportResetAttribution(ResetCause::DATASET_CHANGE_NEW_MAP,
                               ResetAction::CREATE_MAP_EXECUTION);
        p_tracker->createMapInAtlas();
    }

    p_tracker->newDataset();
}

} // namespace core
} // namespace vs_graphs

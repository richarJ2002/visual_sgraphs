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
 * @file            copyRoomContextForMapChecked.cc
 *
 * @brief           Implements Atlas::copyRoomContextForMapChecked().
 */

#include "Atlas.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

AtlasStatus Atlas::copyRoomContextForMapChecked(
    Map                       *p_map_in,
    const bool                 callerOwnsSemanticLock_in,
    Atlas::SnapshotCopyResult &copyResult_out)
{
    SnapshotCopyResult result;
    if (!callerOwnsSemanticLock_in)
    {
        copyResult_out = result;
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }

    std::vector<semantic::RoomContextSnapshot> roomContexts{};
    if (copyRoomContextForMap(p_map_in, roomContexts) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: copyRoomContextForMap returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    result.snapshots = roomContexts;
    result.status    = SnapshotCopyStatus::COMPLETE;
    copyResult_out   = result;
    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

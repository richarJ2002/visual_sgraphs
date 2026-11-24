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
 * @file            recordAcceptedMerge.cc
 *
 * @brief           Implements recordAcceptedMerge(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"

#include <iostream>
#include <rclcpp/logging.hpp>
#include <string>

namespace vs_graphs
{
namespace core
{

void recordAcceptedMerge(map::Map          *p_currentMap_inout,
                         map::Map          *p_mergeMap_in,
                         const std::string &floorDecision_in,
                         const g2o::Sim3 &mergeTransform_absorbedToSurviving_in,
                         double           mergeTime_s_in)
{
    unsigned long currentMapId2{};
    if (p_currentMap_inout->getId(currentMapId2) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    unsigned long mergeMapId2{};
    if (p_mergeMap_in->getId(mergeMapId2) != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    map::MapMergeRecord mergeRecord{};
    mergeRecord.absorbedMapId  = mergeMapId2;
    mergeRecord.survivingMapId = currentMapId2;
    mergeRecord.mergeTime_s    = mergeTime_s_in;
    mergeRecord.mergeRotation_absorbedToSurviving =
        mergeTransform_absorbedToSurviving_in.rotation();
    mergeRecord.mergeTranslation_absorbedToSurviving_m =
        mergeTransform_absorbedToSurviving_in.translation();
    mergeRecord.mergeScale_absorbedToSurviving =
        mergeTransform_absorbedToSurviving_in.scale();
    p_currentMap_inout->addMergeRecord(mergeRecord);

    std::cout << "[SemanticMergeGate] surviving_map=" << currentMapId2
              << " absorbed_map=" << mergeMapId2
              << " decision=ACCEPT reason=ALIGNED"
              << " floor=" << floorDecision_in << " committed=1"
              << formatMergeTransform(mergeTransform_absorbedToSurviving_in)
              << std::endl;
}

} /* namespace core */
} /* namespace vs_graphs */

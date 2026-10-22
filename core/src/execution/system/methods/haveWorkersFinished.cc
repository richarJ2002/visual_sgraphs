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
 * @file            haveWorkersFinished.cc
 *
 * @brief           Implements System::haveWorkersFinished(), declared in
 *                  System.h.
 */

#include "scene_understanding/semantic_segmentation/semantic_segmentation.h"
#include "mapping/local_mapping/objects/LocalMappingStatusEnum.h"
#include "place_recognition_and_loop_closure/loop_closing/objects/LoopClosingStatusEnum.h"
#include "scene_understanding/semantic_segmentation/objects/SemanticSegmentationStatusEnum.h"
#include "scene_understanding/semantics_manager/objects/SemanticsManagerStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"
#include "mapping/local_mapping/local_mapping.h"
#include "place_recognition_and_loop_closure/loop_closing/loop_closing.h"
#include "execution/system/objects/SystemClass.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace system
{

SystemStatus System::haveWorkersFinished(bool &haveFinished_out)
{
    /* Every worker is asked each time: the checks only read each worker's
     * finished flag, so asking all of them has no side effects. */
    bool localMapperFinished{};
    if (p_localMapper->isFinished(localMapperFinished) !=
        local_mapping::LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isFinished returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool loopCloserFinished{};
    if (p_loopCloser->isFinished(loopCloserFinished) !=
        loop_closing::LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isFinished returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool semanticSegmentationFinished{};
    if (p_semanticSegmentation->isFinished(semanticSegmentationFinished) !=
        semantic_segmentation::SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isFinished returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool semanticsManagerFinished{};
    if (p_semanticsManager->isFinished(semanticsManagerFinished) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isFinished returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    haveFinished_out = localMapperFinished && loopCloserFinished &&
                       semanticSegmentationFinished && semanticsManagerFinished;
    return SystemStatus::SYSTEM_STATUS_SUCCESS;
}

} /* namespace system */
} /* namespace core */
} /* namespace vs_graphs */

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

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "SemanticSegmentation.h"
#include "SemanticsManager.h"
#include "System.h"
#include "Tracking.h"
#include "Viewer.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SystemStatus System::shutdown()
{
    {
        std::unique_lock<std::mutex> lock(resetMutex);
        isShutdownRequested = true;
    }

    std::cout << "Shutdown" << std::endl;

    if (p_localMapper->requestFinish() !=
        LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: requestFinish returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_loopCloser->requestFinish() !=
        LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: requestFinish returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_semanticSegmentation->requestFinish() !=
        SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: requestFinish returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_semanticsManager->requestFinish() !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: requestFinish returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_viewer != static_cast<Viewer *>(nullptr))
    {
        if (p_viewer->requestFinish() != ViewerStatus::VIEWER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: requestFinish returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /*
     * Workers report finished only after their owned work has drained. Waiting
     * here prevents Atlas serialization from racing final worker updates.
     */
    std::size_t shutdownPollCount = 0U;
    for (;;)
    {
        bool haveFinished{};
        if (haveWorkersFinished(haveFinished) !=
            SystemStatus::SYSTEM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: haveWorkersFinished returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (haveFinished)
        {
            break;
        }
        usleep(1000);
        ++shutdownPollCount;
        if (shutdownPollCount % 1000U == 0U)
        {
            bool localMappingFinished{};
            if (p_localMapper->isFinished(localMappingFinished) !=
                LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isFinished returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            bool loopClosingFinished{};
            if (p_loopCloser->isFinished(loopClosingFinished) !=
                LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isFinished returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            bool semanticSegmentationFinished{};
            if (p_semanticSegmentation->isFinished(
                    semanticSegmentationFinished) !=
                SemanticSegmentationStatus::
                    SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isFinished returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            bool semanticsManagerFinished{};
            if (p_semanticsManager->isFinished(semanticsManagerFinished) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isFinished returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            bool viewerIsFinished{};
            if (!(p_viewer == static_cast<Viewer *>(nullptr)) &&
                p_viewer->isFinished(viewerIsFinished) !=
                    ViewerStatus::VIEWER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isFinished returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            const bool viewerFinished =
                p_viewer == static_cast<Viewer *>(nullptr) || viewerIsFinished;
            std::cout << "[System::Shutdown] local_mapping="
                      << localMappingFinished
                      << " loop_closing=" << loopClosingFinished
                      << " semantic_segmentation="
                      << semanticSegmentationFinished
                      << " semantics_manager=" << semanticsManagerFinished
                      << " viewer=" << viewerFinished << std::endl;
        }
    }

    std::cout << "[System::Shutdown] all workers completed [flushed]"
              << std::endl;

    if (!saveAtlasFile.empty())
    {
        if (Verbose::printMess("Atlas saving to file " + saveAtlasFile,
                               Verbose::VERBOSITY_NORMAL) !=
            VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        std::unique_lock<std::mutex> semanticUpdateLock{};
        if (p_atlas->acquireSemanticUpdateLock(semanticUpdateLock) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: acquireSemanticUpdateLock returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        bool isSaved{};
        if (saveAtlas(FileType::BINARY_FILE, isSaved) !=
            SystemStatus::SYSTEM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: saveAtlas returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

#ifdef REGISTER_TIMES
    if (p_tracker->printTimeStats() != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printTimeStats returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
#endif

    return SystemStatus::SYSTEM_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

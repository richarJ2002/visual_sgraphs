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
#include "Viewer.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

System::~System()
{
    {
        unique_lock<mutex> lock(resetMutex);
        isShutdownRequested = true;
    }

    /* Request a graceful stop on every running worker thread. */
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

    /* Wait for each worker to report finished before joining. Shutdown() may
     * have already stopped Local Mapping / Loop Closing; isFinished() is
     * idempotent, join() below is the only join in the process. */
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
    }
    /* Join and free the thread objects (first and only join). */
    p_localMappingThread->join();
    p_loopClosingThread->join();
    p_semanticSegmentationThread->join();
    p_semanticsManagerThread->join();
    if (p_viewer != static_cast<Viewer *>(nullptr))
    {
        p_viewerThread->join();
    }
    if (clearResetCause(this) != ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: clearResetCause returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    delete p_localMappingThread;
    delete p_loopClosingThread;
    delete p_semanticSegmentationThread;
    delete p_semanticsManagerThread;
    if (p_viewer != static_cast<Viewer *>(nullptr))
    {
        delete p_viewerThread;
    }
    delete p_geometricSegmentationThread;
}

} // namespace core
} // namespace vs_graphs

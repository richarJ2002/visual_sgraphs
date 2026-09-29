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

namespace vs_graphs
{
namespace core
{

void System::shutdown()
{
    {
        unique_lock<mutex> lock(resetMutex);
        isShutdownRequested = true;
    }

    cout << "Shutdown" << endl;

    p_localMapper->requestFinish();
    p_loopCloser->requestFinish();
    p_semanticSegmentation->requestFinish();
    p_semanticsManager->requestFinish();
    if (p_viewer != static_cast<Viewer *>(nullptr))
    {
        p_viewer->requestFinish();
    }

    /*
     * Workers report finished only after their owned work has drained. Waiting
     * here prevents Atlas serialization from racing final worker updates.
     */
    std::size_t shutdownPollCount = 0U;
    while (
        !p_localMapper->isFinished() || !p_loopCloser->isFinished() ||
        !p_semanticSegmentation->isFinished() ||
        !p_semanticsManager->isFinished() ||
        (p_viewer != static_cast<Viewer *>(nullptr) && !p_viewer->isFinished()))
    {
        usleep(1000);
        ++shutdownPollCount;
        if (shutdownPollCount % 1000U == 0U)
        {
            const bool localMappingFinished = p_localMapper->isFinished();
            const bool loopClosingFinished  = p_loopCloser->isFinished();
            const bool semanticSegmentationFinished =
                p_semanticSegmentation->isFinished();
            const bool semanticsManagerFinished =
                p_semanticsManager->isFinished();
            const bool viewerFinished =
                p_viewer == static_cast<Viewer *>(nullptr) ||
                p_viewer->isFinished();
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
        Verbose::printMess("Atlas saving to file " + saveAtlasFile,
                           Verbose::VERBOSITY_NORMAL);

        std::unique_lock<std::mutex> semanticUpdateLock =
            p_atlas->acquireSemanticUpdateLock();
        saveAtlas(FileType::BINARY_FILE);
    }

#ifdef REGISTER_TIMES
    p_tracker->printTimeStats();
#endif
}

} // namespace core
} // namespace vs_graphs

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

namespace vs_graphs
{
namespace core
{

LoopClosing::LoopClosing(Atlas            *p_atlas_in,
                         KeyFrameDatabase *p_database_in,
                         ORBVocabulary    *p_vocabulary_in,
                         const bool        isScaleFixed_in,
                         const bool        isActiveLc_in) :
    isResetRequested(false),
    isResetActiveMapRequested(false),
    isFinishRequested(false),
    hasFinished(true),
    p_atlas(p_atlas_in),
    p_keyFrameDatabase(p_database_in),
    p_orbVocabulary(p_vocabulary_in),
    p_matchedKF(nullptr),
    isLoopDetected(false),
    loopNumCoincidences(0),
    loopNumNotFound(0),
    isMergeDetected(false),
    hasMergeInProgress(false),
    mergeNumCoincidences(0),
    mergeNumNotFound(0),
    lastLoopKeyFrameId(0),
    isGbaRunning(false),
    hasGbaFinished(true),
    p_threadGBA(nullptr),
    isScaleFixed(isScaleFixed_in),
    fullBundleAdjustmentIndex(0),
    isLoopClosingActive(isActiveLc_in)
{
    covisibilityConsistencyThreshold = 3;
    p_lastCurrentKF                  = static_cast<KeyFrame *>(nullptr);

#ifdef REGISTER_TIMES

    dataQueryTimes_ms.clear();
    sim3EstimationTimes_ms.clear();
    placeRecognitionTotalTimes_ms.clear();

    mergeMapsTimes_ms.clear();
    weldingBaTimes_ms.clear();
    mergeEssentialGraphTimes_ms.clear();
    mergeTotalTimes_ms.clear();
    mergeKeyFrameCounts.clear();
    mergeMapPointCounts.clear();
    mergeCount = 0;

    loopFusionTimes_ms.clear();
    loopEssentialGraphTimes_ms.clear();
    loopTotalTimes_ms.clear();
    loopKeyFrameCounts.clear();
    loopCount = 0;

    gbaTimes_ms.clear();
    updateMapTimes_ms.clear();
    fullGbaTotalTimes_ms.clear();
    gbaKeyFrameCounts.clear();
    gbaMapPointCounts.clear();
    fullGbaExecutionCount = 0;
    fullGbaAbortCount     = 0;

#endif

    mstrFolderSubTraj = "SubTrajectories/";
    numCorrection     = 0;
    correctionGBA     = 0;
}

} // namespace core
} // namespace vs_graphs

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
 * @file            default.cc
 *
 * @brief           Implements the KeyFrame constructor (default), declared in
 *                  KeyFrame.h.
 */

#include "KeyFrame.h"

#include "Frame.h"
#include "ImuTypes.h"
#include "Utils/Converter/objects/Converter.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

/* NOTE: out-of-line to break the KeyFrame<->MapPoint/Map include cycle. */

KeyFrame::KeyFrame() :
    frameId(0),
    timeStamp(0),
    gridCols(FRAME_GRID_COLS),
    gridRows(FRAME_GRID_ROWS),
    gridElementWidthInverse(0),
    gridElementHeightInverse(0),
    trackReferenceFrameId(0),
    fuseTargetKeyFrameId(0),
    baLocalKeyFrameId(0),
    baFixedKeyFrameId(0),
    loopQuery(0),
    loopWords(0),
    relocQuery(0),
    relocWords(0),
    mergeQuery(0),
    mergeWords(0),
    placeRecognitionQuery(0),
    placeRecognitionWords(0),
    placeRecognitionScore(0),
    isInCurrentPlaceRecognition(false),
    baGlobalKeyFrameId(0),
    mergeCorrectedKeyFrameId(0),
    baLocalMergeId(0),
    fx(0),
    fy(0),
    cx(0),
    cy(0),
    invfx(0),
    invfy(0),
    mbf(0),
    mb(0),
    depthThreshold(0),
    keyPointCount(0),
    keyPoints(),
    keyPointsUndistorted(),
    uRight(),
    depths(),
    scaleLevelCount(0),
    scaleFactor(0),
    logScaleFactor(0),
    scaleFactors(0),
    levelSigmaSquared(0),
    invLevelSigmaSquared(0),
    gridMinX(0),
    gridMinY(0),
    gridMaxX(0),
    gridMaxY(0),
    p_prevKF(static_cast<KeyFrame *>(nullptr)),
    p_nextKF(static_cast<KeyFrame *>(nullptr)),
    isVelocityAvailable(false),
    isFirstConnection(true),
    p_parent(nullptr),
    isEraseProtected(false),
    isPendingErase(false),
    isFlaggedBad(false),
    halfBaseline(0),
    leftKeyPointCount(0),
    rightKeyPointCount(0)
{}

} // namespace core
} // namespace vs_graphs

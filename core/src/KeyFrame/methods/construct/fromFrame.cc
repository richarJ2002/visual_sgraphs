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

#include "KeyFrame.h"

#include "Frame.h"
#include "ImuTypes.h"
#include "Map.h"
#include "Utils/Converter/objects/Converter.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

/* NOTE: out-of-line to break the KeyFrame<->MapPoint/Map include cycle. */

KeyFrame::KeyFrame(Frame            &F_inout,
                   Map              *p_map_in,
                   KeyFrameDatabase *p_keyFrameDatabase_in) :
    isImu(p_map_in->isImuInitialized()),
    frameId(F_inout.id),
    timeStamp(F_inout.timeStamp),
    gridCols(FRAME_GRID_COLS),
    gridRows(FRAME_GRID_ROWS),
    gridElementWidthInverse(F_inout.gridElementWidthInverse),
    gridElementHeightInverse(F_inout.gridElementHeightInverse),
    trackReferenceFrameId(0),
    fuseTargetKeyFrameId(0),
    baLocalKeyFrameId(0),
    baFixedKeyFrameId(0),
    optimizationCount(0),
    loopQuery(0),
    loopWords(0),
    relocQuery(0),
    relocWords(0),
    placeRecognitionQuery(0),
    placeRecognitionWords(0),
    placeRecognitionScore(0),
    isInCurrentPlaceRecognition(false),
    baGlobalKeyFrameId(0),
    mergeCorrectedKeyFrameId(0),
    baLocalMergeId(0),
    fx(F_inout.fx),
    fy(F_inout.fy),
    cx(F_inout.cx),
    cy(F_inout.cy),
    invfx(F_inout.invfx),
    invfy(F_inout.invfy),
    mbf(F_inout.mbf),
    mb(F_inout.mb),
    depthThreshold(F_inout.depthThreshold),
    distortionCoefficients(F_inout.distortionCoefficients),
    keyPointCount(F_inout.keyPointCount),
    keyPoints(F_inout.keyPoints),
    keyPointsUndistorted(F_inout.keyPointsUndistorted),
    uRight(F_inout.uRight),
    depths(F_inout.depths),
    descriptors(F_inout.descriptors.clone()),
    bowVector(F_inout.bowVector),
    featureVector(F_inout.featureVector),
    scaleLevelCount(F_inout.scaleLevelCount),
    scaleFactor(F_inout.scaleFactor),
    logScaleFactor(F_inout.logScaleFactor),
    scaleFactors(F_inout.scaleFactors),
    levelSigmaSquared(F_inout.levelSigmaSquared),
    invLevelSigmaSquared(F_inout.invLevelSigmaSquared),
    gridMinX(F_inout.gridMinX),
    gridMinY(F_inout.gridMinY),
    gridMaxX(F_inout.gridMaxX),
    gridMaxY(F_inout.gridMaxY),
    p_prevKF(nullptr),
    p_nextKF(nullptr),
    p_imuPreintegrated(F_inout.p_imuPreintegrated),
    imuCalibration(F_inout.imuCalibration),
    fileName(F_inout.fileName),
    datasetId(F_inout.datasetId),
    colorImg(F_inout.colorImg),
    isPublished(false),
    isVelocityAvailable(false),
    poseTlr(F_inout.getRelativePoseTlr()),
    poseTrl(F_inout.getRelativePoseTrl()),
    mapPoints(F_inout.mapPoints),
    p_keyFrameDatabase(p_keyFrameDatabase_in),
    p_orbVocabulary(F_inout.p_orbVocabulary),
    isFirstConnection(true),
    p_parent(nullptr),
    isEraseProtected(false),
    isPendingErase(false),
    isFlaggedBad(false),
    halfBaseline(F_inout.mb / 2),
    currentFrameMarkers(F_inout.mapMarkers),
    currentFrameMapPoints(F_inout.mapPoints),
    currentFramePointClouds(F_inout.pointClouds),
    p_map(p_map_in),
    calibrationMatrixEigen(F_inout.calibrationMatrixEigen),
    p_camera(F_inout.p_camera),
    p_camera2(F_inout.p_camera2),
    leftToRightMatches(F_inout.leftToRightMatches),
    rightToLeftMatches(F_inout.rightToLeftMatches),
    keyPointsRight(F_inout.keyPointsRight),
    leftKeyPointCount(F_inout.leftKeyPointCount),
    rightKeyPointCount(F_inout.rightKeyPointCount)
{
    id = nextId++;

    grid.resize(gridCols);
    if (F_inout.leftKeyPointCount != -1)
        gridRight.resize(gridCols);
    for (int columnIndex = 0; columnIndex < gridCols; columnIndex++)
    {
        grid[columnIndex].resize(gridRows);
        if (F_inout.leftKeyPointCount != -1)
            gridRight[columnIndex].resize(gridRows);
        for (int rowIndex = 0; rowIndex < gridRows; rowIndex++)
        {
            grid[columnIndex][rowIndex] = F_inout.grid[columnIndex][rowIndex];
            if (F_inout.leftKeyPointCount != -1)
            {
                gridRight[columnIndex][rowIndex] =
                    F_inout.gridRight[columnIndex][rowIndex];
            }
        }
    }

    if (!F_inout.hasVelocity())
    {
        velocityVw.setZero();
        isVelocityAvailable = false;
    }
    else
    {
        velocityVw          = F_inout.getVelocity();
        isVelocityAvailable = true;
    }

    imuBias = F_inout.imuBias;
    setPose(F_inout.getPose());

    originMapId = p_map_in->getId();
}

} // namespace core
} // namespace vs_graphs

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

#include "ImuTypes.h"
#include "Utils/Converter/objects/Converter.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

/* NOTE: out-of-line per the WP-02 cycle-break policy (KeyFrame<->MapPoint/Map SCC). */

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB) :
    isImu(pMap->isImuInitialized()),
    frameId(F.mnId),
    timeStamp(F.timeStamp),
    gridCols(FRAME_GRID_COLS),
    gridRows(FRAME_GRID_ROWS),
    gridElementWidthInverse(F.gridElementWidthInverse),
    gridElementHeightInverse(F.gridElementHeightInverse),
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
    currentPlaceRecognition(false),
    baGlobalKeyFrameId(0),
    mergeCorrectedKeyFrameId(0),
    baLocalMergeId(0),
    fx(F.fx),
    fy(F.fy),
    cx(F.cx),
    cy(F.cy),
    invfx(F.invfx),
    invfy(F.invfy),
    mbf(F.mbf),
    mb(F.mb),
    depthThreshold(F.depthThreshold),
    distortionCoefficients(F.distortionCoefficients),
    N(F.N),
    keyPoints(F.keyPoints),
    keyPointsUndistorted(F.keyPointsUndistorted),
    uRight(F.uRight),
    depths(F.depths),
    descriptors(F.descriptors.clone()),
    bowVector(F.bowVector),
    featureVector(F.featureVector),
    scaleLevelCount(F.scaleLevelCount),
    scaleFactor(F.scaleFactor),
    logScaleFactor(F.logScaleFactor),
    scaleFactors(F.scaleFactors),
    levelSigmaSquared(F.levelSigmaSquared),
    invLevelSigmaSquared(F.invLevelSigmaSquared),
    gridMinX(F.gridMinX),
    gridMinY(F.gridMinY),
    gridMaxX(F.gridMaxX),
    gridMaxY(F.gridMaxY),
    p_prevKF(nullptr),
    p_nextKF(nullptr),
    p_imuPreintegrated(F.p_imuPreintegrated),
    imuCalibration(F.imuCalibration),
    fileName(F.fileName),
    datasetId(F.datasetId),
    colorImg(F.colorImg),
    isPublished(false),
    velocityAvailable(false),
    poseTlr(F.getRelativePoseTlr()),
    poseTrl(F.getRelativePoseTrl()),
    mapPoints(F.mapPoints),
    p_keyFrameDatabase(pKFDB),
    p_orbVocabulary(F.p_orbVocabulary),
    firstConnection(true),
    p_parent(nullptr),
    notErase(false),
    toBeErased(false),
    mbBad(false),
    halfBaseline(F.mb / 2),
    currentFrameMarkers(F.mapMarkers),
    currentFrameMapPoints(F.mapPoints),
    currentFramePointClouds(F.pointClouds),
    p_map(pMap),
    calibrationMatrixEigen(F.calibrationMatrixEigen),
    p_camera(F.p_camera),
    p_camera2(F.p_camera2),
    leftToRightMatches(F.leftToRightMatches),
    rightToLeftMatches(F.rightToLeftMatches),
    keyPointsRight(F.keyPointsRight),
    Nleft(F.Nleft),
    Nright(F.Nright)
{
    mnId = nNextId++;

    grid.resize(gridCols);
    if (F.Nleft != -1)
        gridRight.resize(gridCols);
    for (int i = 0; i < gridCols; i++)
    {
        grid[i].resize(gridRows);
        if (F.Nleft != -1)
            gridRight[i].resize(gridRows);
        for (int j = 0; j < gridRows; j++)
        {
            grid[i][j] = F.grid[i][j];
            if (F.Nleft != -1)
            {
                gridRight[i][j] = F.gridRight[i][j];
            }
        }
    }

    if (!F.hasVelocity())
    {
        velocityVw.setZero();
        velocityAvailable = false;
    }
    else
    {
        velocityVw        = F.getVelocity();
        velocityAvailable = true;
    }

    imuBias = F.imuBias;
    setPose(F.getPose());

    originMapId = pMap->getId();
}

} // namespace core
} // namespace vs_graphs

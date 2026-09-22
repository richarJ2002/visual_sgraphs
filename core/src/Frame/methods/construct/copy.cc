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

#include "Frame.h"

#include "CameraModels/GeometricCamera/objects/GeometricCamera.h"
#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"
#include "CameraModels/Pinhole/objects/Pinhole.h"
#include "G2oTypes.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "StereoMatchOutlierRejection.h"
#include "Utils/Converter/objects/Converter.h"

#include <thread>

namespace vs_graphs
{
namespace core
{

/* NOTE: out-of-line per the WP-02 cycle-break policy (Frame<->KeyFrame/MapPoint SCC). */

Frame::Frame(const Frame &frame) :
    p_poseImuConstraint(frame.p_poseImuConstraint),
    poseTcw(frame.poseTcw),
    poseAvailable(false),
    poseTlr(frame.poseTlr),
    poseTrl(frame.poseTrl),
    rotationRlr(frame.rotationRlr),
    translationTlr(frame.translationTlr),
    velocityAvailable(false),
    p_orbVocabulary(frame.p_orbVocabulary),
    p_orbExtractorLeft(frame.p_orbExtractorLeft),
    p_orbExtractorRight(frame.p_orbExtractorRight),
    timeStamp(frame.timeStamp),
    calibrationMatrix(frame.calibrationMatrix.clone()),
    calibrationMatrixEigen(
        utils::converter::Converter::toMatrix3f(frame.calibrationMatrix)),
    distortionCoefficients(frame.distortionCoefficients.clone()),
    mbf(frame.mbf),
    mb(frame.mb),
    depthThreshold(frame.depthThreshold),
    N(frame.N),
    keyPoints(frame.keyPoints),
    keyPointsRight(frame.keyPointsRight),
    keyPointsUndistorted(frame.keyPointsUndistorted),
    mapPoints(frame.mapPoints),
    uRight(frame.uRight),
    depths(frame.depths),
    bowVector(frame.bowVector),
    featureVector(frame.featureVector),
    descriptors(frame.descriptors.clone()),
    descriptorsRight(frame.descriptorsRight.clone()),
    outlierFlags(frame.outlierFlags),
    closeMapPointCount(frame.closeMapPointCount),
    imuBias(frame.imuBias),
    imuCalibration(frame.imuCalibration),
    p_imuPreintegrated(frame.p_imuPreintegrated),
    p_lastKeyFrame(frame.p_lastKeyFrame),
    p_previousFrame(frame.p_previousFrame),
    p_imuPreintegratedFrame(frame.p_imuPreintegratedFrame),
    mnId(frame.mnId),
    p_referenceKeyFrame(frame.p_referenceKeyFrame),
    scaleLevelCount(frame.scaleLevelCount),
    scaleFactor(frame.scaleFactor),
    logScaleFactor(frame.logScaleFactor),
    scaleFactors(frame.scaleFactors),
    invScaleFactors(frame.invScaleFactors),
    levelSigmaSquared(frame.levelSigmaSquared),
    invLevelSigmaSquared(frame.invLevelSigmaSquared),
    fileName(frame.fileName),
    datasetId(frame.datasetId),
    isFrameSet(frame.isFrameSet),
    imuPreintegrated(frame.imuPreintegrated),
    p_imuMutex(frame.p_imuMutex),
    p_camera(frame.p_camera),
    p_camera2(frame.p_camera2),
    Nleft(frame.Nleft),
    Nright(frame.Nright),
    monoLeft(frame.monoLeft),
    monoRight(frame.monoRight),
    leftToRightMatches(frame.leftToRightMatches),
    rightToLeftMatches(frame.rightToLeftMatches),
    stereoPoints3D(frame.stereoPoints3D)
{
    for (int i = 0; i < FRAME_GRID_COLS; i++)
        for (int j = 0; j < FRAME_GRID_ROWS; j++)
        {
            grid[i][j] = frame.grid[i][j];
            if (frame.Nleft > 0)
            {
                gridRight[i][j] = frame.gridRight[i][j];
            }
        }

    if (frame.poseAvailable)
        setPose(frame.getPose());

    if (frame.hasVelocity())
    {
        setVelocity(frame.getVelocity());
    }

    projectedPoints = frame.projectedPoints;
    matchedPoints   = frame.matchedPoints;

#ifdef REGISTER_TIMES
    stereoMatchTime   = frame.stereoMatchTime;
    orbExtractionTime = frame.orbExtractionTime;
#endif
}

} // namespace core
} // namespace vs_graphs

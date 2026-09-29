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

#include <rclcpp/logging.hpp>
#include <thread>

namespace vs_graphs
{
namespace core
{

/* NOTE: out-of-line to break the Frame<->KeyFrame/MapPoint include cycle. */

Frame::Frame(const Frame &frame_in) :
    p_poseImuConstraint(frame_in.p_poseImuConstraint),
    poseTcw(frame_in.poseTcw),
    isPoseAvailable(false),
    poseTlr(frame_in.poseTlr),
    poseTrl(frame_in.poseTrl),
    rotationRlr(frame_in.rotationRlr),
    translationTlr(frame_in.translationTlr),
    isVelocityAvailable(false),
    p_orbVocabulary(frame_in.p_orbVocabulary),
    p_orbExtractorLeft(frame_in.p_orbExtractorLeft),
    p_orbExtractorRight(frame_in.p_orbExtractorRight),
    timeStamp(frame_in.timeStamp),
    calibrationMatrix(frame_in.calibrationMatrix.clone()),
    distortionCoefficients(frame_in.distortionCoefficients.clone()),
    mbf(frame_in.mbf),
    mb(frame_in.mb),
    depthThreshold(frame_in.depthThreshold),
    keyPointCount(frame_in.keyPointCount),
    keyPoints(frame_in.keyPoints),
    keyPointsRight(frame_in.keyPointsRight),
    keyPointsUndistorted(frame_in.keyPointsUndistorted),
    mapPoints(frame_in.mapPoints),
    uRight(frame_in.uRight),
    depths(frame_in.depths),
    bowVector(frame_in.bowVector),
    featureVector(frame_in.featureVector),
    descriptors(frame_in.descriptors.clone()),
    descriptorsRight(frame_in.descriptorsRight.clone()),
    outlierFlags(frame_in.outlierFlags),
    closeMapPointCount(frame_in.closeMapPointCount),
    imuBias(frame_in.imuBias),
    imuCalibration(frame_in.imuCalibration),
    p_imuPreintegrated(frame_in.p_imuPreintegrated),
    p_lastKeyFrame(frame_in.p_lastKeyFrame),
    p_previousFrame(frame_in.p_previousFrame),
    p_imuPreintegratedFrame(frame_in.p_imuPreintegratedFrame),
    id(frame_in.id),
    p_referenceKeyFrame(frame_in.p_referenceKeyFrame),
    scaleLevelCount(frame_in.scaleLevelCount),
    scaleFactor(frame_in.scaleFactor),
    logScaleFactor(frame_in.logScaleFactor),
    scaleFactors(frame_in.scaleFactors),
    invScaleFactors(frame_in.invScaleFactors),
    levelSigmaSquared(frame_in.levelSigmaSquared),
    invLevelSigmaSquared(frame_in.invLevelSigmaSquared),
    fileName(frame_in.fileName),
    datasetId(frame_in.datasetId),
    isFrameSet(frame_in.isFrameSet),
    hasImuPreintegration(frame_in.hasImuPreintegration),
    p_imuMutex(frame_in.p_imuMutex),
    p_camera(frame_in.p_camera),
    p_camera2(frame_in.p_camera2),
    leftKeyPointCount(frame_in.leftKeyPointCount),
    rightKeyPointCount(frame_in.rightKeyPointCount),
    monoLeft(frame_in.monoLeft),
    monoRight(frame_in.monoRight),
    leftToRightMatches(frame_in.leftToRightMatches),
    rightToLeftMatches(frame_in.rightToLeftMatches),
    stereoPoints3D(frame_in.stereoPoints3D)
{
    if (utils::converter::Converter::toMatrix3f(frame_in.calibrationMatrix,
                                                calibrationMatrixEigen) !=
        utils::converter::ConverterStatus::CONVERTER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: toMatrix3f returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    for (int columnIndex = 0; columnIndex < FRAME_GRID_COLS; columnIndex++)
        for (int rowIndex = 0; rowIndex < FRAME_GRID_ROWS; rowIndex++)
        {
            grid[columnIndex][rowIndex] = frame_in.grid[columnIndex][rowIndex];
            if (frame_in.leftKeyPointCount > 0)
            {
                gridRight[columnIndex][rowIndex] =
                    frame_in.gridRight[columnIndex][rowIndex];
            }
        }

    if (frame_in.isPoseAvailable)
        setPose(frame_in.getPose());

    if (frame_in.hasVelocity())
    {
        setVelocity(frame_in.getVelocity());
    }

    projectedPoints = frame_in.projectedPoints;
    matchedPoints   = frame_in.matchedPoints;

#ifdef REGISTER_TIMES
    stereoMatchTime   = frame_in.stereoMatchTime;
    orbExtractionTime = frame_in.orbExtractionTime;
#endif
}

} // namespace core
} // namespace vs_graphs

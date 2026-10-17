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
 * @file            serialize.cc
 *
 * @brief           Implements KeyFrame::serialize(), declared in
 * keyframe/objects/KeyFrameClass.h.
 */

#include "keyframe/objects/KeyFrameClass.h"

#include "utils/utils/objects/SerializationUtils.h"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

namespace vs_graphs
{
namespace core
{
namespace keyframe
{

/* Instantiated below for every archive used, so only this file compiles it */
template <class Archive>
void KeyFrame::serialize(Archive &ar, const unsigned int version)
{
    ar & id;
    ar &const_cast<long unsigned int &>(frameId);
    ar &const_cast<double &>(timeStamp);
    /* Grid */
    ar &const_cast<int &>(gridCols);
    ar &const_cast<int &>(gridRows);
    ar &const_cast<float &>(gridElementWidthInverse);
    ar &const_cast<float &>(gridElementHeightInverse);

    /* Scale */
    ar & correctedScale;
    /* Calibration parameters */
    ar &const_cast<float &>(fx);
    ar &const_cast<float &>(fy);
    ar &const_cast<float &>(invfx);
    ar &const_cast<float &>(invfy);
    ar &const_cast<float &>(cx);
    ar &const_cast<float &>(cy);
    ar &const_cast<float &>(mbf);
    ar &const_cast<float &>(mb);
    ar &const_cast<float &>(depthThreshold);
    serializeMatrix(ar, distortionCoefficients, version);
    /* Number of Keypoints */
    ar &const_cast<int &>(keyPointCount);
    /* KeyPoints */
    serializeVectorKeyPoints<Archive>(ar, keyPoints, version);
    serializeVectorKeyPoints<Archive>(ar, keyPointsUndistorted, version);
    ar &const_cast<std::vector<float> &>(uRight);
    ar &const_cast<std::vector<float> &>(depths);
    serializeMatrix<Archive>(ar, descriptors, version);
    /* BOW */
    ar & bowVector;
    ar & featureVector;
    /* Pose relative to parent */
    serializeSophusSE3<Archive>(ar, tcp, version);
    /* Scale */
    ar &const_cast<int &>(scaleLevelCount);
    ar &const_cast<float &>(scaleFactor);
    ar &const_cast<float &>(logScaleFactor);
    ar &const_cast<std::vector<float> &>(scaleFactors);
    ar &const_cast<std::vector<float> &>(levelSigmaSquared);
    ar &const_cast<std::vector<float> &>(invLevelSigmaSquared);
    /* Image bounds and calibration */
    ar &const_cast<int &>(gridMinX);
    ar &const_cast<int &>(gridMinY);
    ar &const_cast<int &>(gridMaxX);
    ar &const_cast<int &>(gridMaxY);
    ar &boost::serialization::make_array(calibrationMatrixEigen.data(),
                                         calibrationMatrixEigen.size());
    /* Pose */
    serializeSophusSE3<Archive>(ar, poseTcw, version);
    /* MapPointsId associated to keypoints */
    ar & backupMapPointsId;
    /* Grid */
    ar & grid;
    /* Connected KeyFrameWeight */
    ar & backupConnectedKeyFrameIdWeights;
    /* Spanning Tree and Loop Edges */
    ar & isFirstConnection;
    ar & backupParentId;
    ar & backupChildrensId;
    ar & backupLoopEdgesId;
    ar & backupMergeEdgesId;
    /* Bad flags */
    ar & isEraseProtected;
    ar & isPendingErase;
    ar & isFlaggedBad;

    ar & halfBaseline;

    ar & originMapId;

    /* Camera variables */
    ar & backupCameraId;
    ar & backupCamera2Id;

    /* Fisheye variables */
    ar & leftToRightMatches;
    ar & rightToLeftMatches;
    ar &const_cast<int &>(leftKeyPointCount);
    ar &const_cast<int &>(rightKeyPointCount);
    serializeSophusSE3<Archive>(ar, poseTlr, version);
    serializeVectorKeyPoints<Archive>(ar, keyPointsRight, version);
    ar & gridRight;

    /* Inertial variables */
    ar & imuBias;
    ar & backupImuPreintegrated;
    ar & imuCalibration;
    ar & backupPrevKFId;
    ar & backupNextKFId;
    ar & isImu;
    ar &boost::serialization::make_array(velocityVw.data(), velocityVw.size());
    ar &boost::serialization::make_array(owb.data(), owb.size());
    ar & isVelocityAvailable;
}

/*!
 * @brief           Instantiates serialize for loading from a binary archive.
 */
template void KeyFrame::serialize<boost::archive::binary_iarchive>(
    boost::archive::binary_iarchive &,
    const unsigned int);
/*!
 * @brief           Instantiates serialize for saving to a binary archive.
 */
template void KeyFrame::serialize<boost::archive::binary_oarchive>(
    boost::archive::binary_oarchive &,
    const unsigned int);
/*!
 * @brief           Instantiates serialize for loading from a text archive.
 */
template void KeyFrame::serialize<boost::archive::text_iarchive>(
    boost::archive::text_iarchive &,
    const unsigned int);
/*!
 * @brief           Instantiates serialize for saving to a text archive.
 */
template void KeyFrame::serialize<boost::archive::text_oarchive>(
    boost::archive::text_oarchive &,
    const unsigned int);

} /* namespace keyframe */
} /* namespace core */
} /* namespace vs_graphs */

/*!
 * This file is part of ORB-SLAM3.
 * Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details:
 * https://www.gnu.org/licenses/
 */

#ifndef SERIALIZATION_UTILS_H
#define SERIALIZATION_UTILS_H

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <vector>

namespace vs_graphs
{
namespace core
{

template <class Archive>
void serializeSophusSE3(Archive                            &ar,
                        Sophus::SE3f                       &T,
                        [[maybe_unused]] const unsigned int version)
{
    Eigen::Vector4f quaternionCoefficients;
    Eigen::Vector3f translation;

    if (Archive::is_saving::value)
    {
        Eigen::Quaternionf quaternion = T.unit_quaternion();
        quaternionCoefficients << quaternion.w(), quaternion.x(),
            quaternion.y(), quaternion.z();
        translation = T.translation();
    }

    ar &boost::serialization::make_array(quaternionCoefficients.data(),
                                         quaternionCoefficients.size());
    ar &boost::serialization::make_array(translation.data(),
                                         translation.size());

    if (Archive::is_loading::value)
    {
        Eigen::Quaternionf quaternion(quaternionCoefficients[0],
                                      quaternionCoefficients[1],
                                      quaternionCoefficients[2],
                                      quaternionCoefficients[3]);
        T = Sophus::SE3f(quaternion, translation);
    }
}

/*template <class Archive, size_t dim>
void serializeDiagonalMatrix(Archive &ar, Eigen::DiagonalMatrix<float, dim> &D,
const unsigned int version)
{
    Eigen::Matrix<float,dim,dim> dense;
    if(Archive::is_saving::value)
    {
        dense = D.toDenseMatrix();
    }

    ar & boost::serialization::make_array(dense.data(), dense.size());

    if (Archive::is_loading::value)
    {
        D = dense.diagonal().asDiagonal();
    }
}*/

template <class Archive>
void serializeMatrix(Archive                            &ar,
                     cv::Mat                            &mat,
                     [[maybe_unused]] const unsigned int version)
{
    int  columnCount, rowCount, matType;
    bool isContinuous;

    if (Archive::is_saving::value)
    {
        columnCount  = mat.cols;
        rowCount     = mat.rows;
        matType      = mat.type();
        isContinuous = mat.isContinuous();
    }

    ar & columnCount & rowCount & matType & isContinuous;

    if (Archive::is_loading::value)
        mat.create(rowCount, columnCount, matType);

    if (isContinuous)
    {
        const unsigned int dataByteCount =
            rowCount * columnCount * mat.elemSize();
        ar &boost::serialization::make_array(mat.ptr(), dataByteCount);
    }
    else
    {
        const unsigned int rowByteCount = columnCount * mat.elemSize();
        for (int rowIndex = 0; rowIndex < rowCount; rowIndex++)
        {
            ar &boost::serialization::make_array(mat.ptr(rowIndex),
                                                 rowByteCount);
        }
    }
}

template <class Archive>
void serializeMatrix(Archive           &ar,
                     const cv::Mat     &mat,
                     const unsigned int version)
{
    cv::Mat mutableMatrixCopy = mat;

    serializeMatrix(ar, mutableMatrixCopy, version);

    if (Archive::is_loading::value)
    {
        cv::Mat *p_matrixWriteback;
        p_matrixWriteback  = const_cast<cv::Mat *>(&mat);
        *p_matrixWriteback = mutableMatrixCopy;
    }
}

template <class Archive>
void serializeVectorKeyPoints(Archive                            &ar,
                              const std::vector<cv::KeyPoint>    &vKP,
                              [[maybe_unused]] const unsigned int version)
{
    int keyPointCount;

    if (Archive::is_saving::value)
    {
        keyPointCount = vKP.size();
    }

    ar & keyPointCount;

    std::vector<cv::KeyPoint> keyPointsCopy = vKP;
    if (Archive::is_loading::value)
        keyPointsCopy.reserve(keyPointCount);

    for (int keyPointIndex = 0; keyPointIndex < keyPointCount; ++keyPointIndex)
    {
        cv::KeyPoint keyPoint;

        if (Archive::is_loading::value)
            keyPoint = cv::KeyPoint();

        if (Archive::is_saving::value)
            keyPoint = keyPointsCopy[keyPointIndex];

        ar & keyPoint.angle;
        ar & keyPoint.response;
        ar & keyPoint.size;
        ar & keyPoint.pt.x;
        ar & keyPoint.pt.y;
        ar & keyPoint.class_id;
        ar & keyPoint.octave;

        if (Archive::is_loading::value)
            keyPointsCopy.push_back(keyPoint);
    }

    if (Archive::is_loading::value)
    {
        std::vector<cv::KeyPoint> *p_keyPointsWriteback;
        p_keyPointsWriteback  = const_cast<std::vector<cv::KeyPoint> *>(&vKP);
        *p_keyPointsWriteback = keyPointsCopy;
    }
}

} // namespace core
} // namespace vs_graphs

#endif // SERIALIZATION_UTILS_H

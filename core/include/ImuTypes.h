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

/*!
 * @file         ImuTypes.h
 *
 * @brief        Declares IMU measurements, biases and preintegration.
 */

#ifndef IMUTYPES_H
#define IMUTYPES_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>
#include <utility>
#include <vector>

#include "SerializationUtils.h"

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

namespace vs_graphs
{
namespace core
{

namespace IMU
{
/*!
 * @brief        Gravity magnitude in metres per second
 *               squared.
 */
const float GRAVITY_VALUE = 9.81;

/*!
 * @brief        Singularity threshold shared by the SO(3)
 *               Jacobian helpers and IntegratedRotation.
 */
const float eps = 1e-4;

/*!
 * @brief        Single IMU sample with accelerometer,
 *               gyroscope and timestamp.
 */
class Point
{
  public:
    /*!
     * @brief        Creates a sample from raw components.
     *
     * @param[in]    accelerometerX_in
     *               Accelerometer x in metres per second
     *               squared.
     * @param[in]    accelerometerY_in
     *               Accelerometer y in metres per second
     *               squared.
     * @param[in]    accelerometerZ_in
     *               Accelerometer z in metres per second
     *               squared.
     * @param[in]    gyroscopeX_in
     *               Gyroscope x in radians per second.
     * @param[in]    gyroscopeY_in
     *               Gyroscope y in radians per second.
     * @param[in]    gyroscopeZ_in
     *               Gyroscope z in radians per second.
     * @param[in]    timestamp_in
     *               Sample timestamp in seconds.
     */
    Point(const float  &accelerometerX_in,
          const float  &accelerometerY_in,
          const float  &accelerometerZ_in,
          const float  &gyroscopeX_in,
          const float  &gyroscopeY_in,
          const float  &gyroscopeZ_in,
          const double &timestamp_in) :
        a(accelerometerX_in, accelerometerY_in, accelerometerZ_in),
        w(gyroscopeX_in, gyroscopeY_in, gyroscopeZ_in),
        t(timestamp_in)
    {}
    /*!
     * @brief        Creates a sample from OpenCV vectors.
     *
     * @param[in]    accelerometer_in
     *               Accelerometer sample in metres per second
     *               squared.
     * @param[in]    gyroscope_in
     *               Gyroscope sample in radians per second.
     * @param[in]    timestamp_in
     *               Sample timestamp in seconds.
     */
    Point(const cv::Point3f accelerometer_in,
          const cv::Point3f gyroscope_in,
          const double     &timestamp_in) :
        a(accelerometer_in.x, accelerometer_in.y, accelerometer_in.z),
        w(gyroscope_in.x, gyroscope_in.y, gyroscope_in.z),
        t(timestamp_in)
    {}

  public:
    /*!
     * @brief        Accelerometer sample in metres per second
     *               squared.
     */
    Eigen::Vector3f a;
    /*!
     * @brief        Gyroscope sample in radians per second.
     */
    Eigen::Vector3f w;
    /*!
     * @brief        Sample timestamp in seconds.
     */
    double          t;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/*!
 * @brief        IMU biases for the gyroscope and the
 *               accelerometer.
 */
class Bias
{
    friend class boost::serialization::access;
    /*!
     * @brief        Serializes the six bias components.
     *
     * @param[in,out] ar
     *                Archive receiving the stored fields.
     * @param[in]    version
     *               Archive version; currently unused.
     */
    template <class Archive>
    void serialize(Archive &ar, [[maybe_unused]] const unsigned int version)
    {
        ar & bax;
        ar & bay;
        ar & baz;

        ar & bwx;
        ar & bwy;
        ar & bwz;
    }

  public:
    /*!
     * @brief        Creates zero biases.
     */
    Bias() :
        bax(0),
        bay(0),
        baz(0),
        bwx(0),
        bwy(0),
        bwz(0)
    {}
    /*!
     * @brief        Creates biases from raw components.
     *
     * @param[in]    accelerometerBiasX_in
     *               Accelerometer x bias.
     * @param[in]    accelerometerBiasY_in
     *               Accelerometer y bias.
     * @param[in]    accelerometerBiasZ_in
     *               Accelerometer z bias.
     * @param[in]    gyroscopeBiasX_in
     *               Gyroscope x bias.
     * @param[in]    gyroscopeBiasY_in
     *               Gyroscope y bias.
     * @param[in]    gyroscopeBiasZ_in
     *               Gyroscope z bias.
     */
    Bias(const float &accelerometerBiasX_in,
         const float &accelerometerBiasY_in,
         const float &accelerometerBiasZ_in,
         const float &gyroscopeBiasX_in,
         const float &gyroscopeBiasY_in,
         const float &gyroscopeBiasZ_in) :
        bax(accelerometerBiasX_in),
        bay(accelerometerBiasY_in),
        baz(accelerometerBiasZ_in),
        bwx(gyroscopeBiasX_in),
        bwy(gyroscopeBiasY_in),
        bwz(gyroscopeBiasZ_in)
    {}
    /*!
     * @brief        Copies every component from another bias.
     *
     * @param[in]    b_in
     *               Source bias; shall be non-null.
     */
    void                 copyFrom(Bias &b_in);
    /*!
     * @brief        Appends the six bias components to the
     *               stream.
     *
     * @param[in,out] out_inout
     *                Stream receiving the components.
     * @param[in]    b_in
     *               Bias whose components are written.
     *
     * @return       The output stream.
     */
    friend std::ostream &operator<<(std::ostream &out_inout, const Bias &b_in);

  public:
    /*!
     * @brief        Accelerometer bias components.
     */
    float bax, bay, baz;
    /*!
     * @brief        Gyroscope bias components.
     */
    float bwx, bwy, bwz;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/*!
 * @brief        IMU calibration holding transforms and noise
 *               models.
 */
class Calib
{
    friend class boost::serialization::access;
    /*!
     * @brief        Serializes the transforms, covariances
     *               and validity flag.
     *
     * @param[in,out] ar
     *                Archive receiving the stored fields.
     * @param[in]    version
     *               Archive version; currently unused.
     */
    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        serializeSophusSE3(ar, mTcb, version);
        serializeSophusSE3(ar, mTbc, version);

        ar &boost::serialization::make_array(Cov.diagonal().data(),
                                             Cov.diagonal().size());
        ar &boost::serialization::make_array(CovWalk.diagonal().data(),
                                             CovWalk.diagonal().size());

        ar & isCalibrationSet;
    }

  public:
    /*!
     * @brief        Creates a calibration and marks it valid.
     *
     * @param[in]    Tbc_in
     *               Body-to-camera transform.
     * @param[in]    gyroscopeNoiseDensity_in
     *               Gyroscope noise density.
     * @param[in]    accelerometerNoiseDensity_in
     *               Accelerometer noise density.
     * @param[in]    gyroscopeRandomWalkDensity_in
     *               Gyroscope random-walk density.
     * @param[in]    accelerometerRandomWalkDensity_in
     *               Accelerometer random-walk density.
     */
    Calib(const Sophus::SE3<float> &Tbc_in,
          const float              &gyroscopeNoiseDensity_in,
          const float              &accelerometerNoiseDensity_in,
          const float              &gyroscopeRandomWalkDensity_in,
          const float              &accelerometerRandomWalkDensity_in)
    {
        setCalibration(Tbc_in,
                       gyroscopeNoiseDensity_in,
                       accelerometerNoiseDensity_in,
                       gyroscopeRandomWalkDensity_in,
                       accelerometerRandomWalkDensity_in);
    }

    /*!
     * @brief        Copies another calibration.
     *
     * @param[in]    calib_in
     *               Source calibration.
     */
    Calib(const Calib &calib_in)
    {
        isCalibrationSet = calib_in.isCalibrationSet;
        // Sophus/Eigen parameters
        mTbc    = calib_in.mTbc;
        mTcb    = calib_in.mTcb;
        Cov     = calib_in.Cov;
        CovWalk = calib_in.CovWalk;
    }
    /*!
     * @brief        Copies every member from another
     *               calibration.
     *
     *               Declared explicitly because the
     *               user-provided copy constructor suppresses
     *               the implicit declaration; the copy
     *               constructor is memberwise, so the
     *               defaulted assignment matches it.
     *
     * @param[in]    calib_in
     *               Source calibration.
     *
     * @return       Reference to this calibration.
     */
    Calib &operator=(const Calib &calib_in) = default;
    /*!
     * @brief        Creates an unset calibration.
     */
    Calib()
    {
        isCalibrationSet = false;
    }

    // void Set(const cv::Mat &cvTbc, const float &ng, const float &na, const
    // float &ngw, const float &naw);
    /*!
     * @brief        Stores the transform and noise densities
     *               and marks the calibration valid.
     *
     * @param[in]    sophTbc_in
     *               Body-to-camera transform.
     * @param[in]    ng_in
     *               Gyroscope noise density.
     * @param[in]    na_in
     *               Accelerometer noise density.
     * @param[in]    ngw_in
     *               Gyroscope random-walk density.
     * @param[in]    naw_in
     *               Accelerometer random-walk density.
     */
    void setCalibration(const Sophus::SE3<float> &sophTbc_in,
                        const float              &ng_in,
                        const float              &na_in,
                        const float              &ngw_in,
                        const float              &naw_in);

  public:
    // Sophus/Eigen implementation
    /*!
     * @brief        Camera-to-body transform.
     */
    Sophus::SE3<float>              mTcb;
    /*!
     * @brief        Body-to-camera transform.
     */
    Sophus::SE3<float>              mTbc;
    /*!
     * @brief        Noise and random-walk covariances.
     */
    Eigen::DiagonalMatrix<float, 6> Cov, CovWalk;
    /*!
     * @brief        True once Set has stored a calibration.
     */
    bool                            isCalibrationSet;
};

/*!
 * @brief        Rotation integrated from one gyroscope
 *               measurement.
 */
class IntegratedRotation
{
  public:
    /*!
     * @brief        Creates an empty integrated rotation.
     */
    IntegratedRotation() {}
    /*!
     * @brief        Integrates one angular-rate measurement.
     *
     * @param[in]    angularVelocity_in
     *               Angular rate in radians per second.
     * @param[in]    imuBias_in
     *               Bias subtracted before integration.
     * @param[in]    integrationInterval_in
     *               Integration interval in seconds.
     */
    IntegratedRotation(const Eigen::Vector3f &angularVelocity_in,
                       const Bias            &imuBias_in,
                       const float           &integrationInterval_in)
    {
        const float rotationVectorX =
            (angularVelocity_in(0) - imuBias_in.bwx) * integrationInterval_in;
        const float rotationVectorY =
            (angularVelocity_in(1) - imuBias_in.bwy) * integrationInterval_in;
        const float rotationVectorZ =
            (angularVelocity_in(2) - imuBias_in.bwz) * integrationInterval_in;

        const float rotationAngleSquared = rotationVectorX * rotationVectorX +
                                           rotationVectorY * rotationVectorY +
                                           rotationVectorZ * rotationVectorZ;
        const float rotationAngle = sqrt(rotationAngleSquared);

        Eigen::Vector3f rotationVector;
        rotationVector << rotationVectorX, rotationVectorY, rotationVectorZ;
        Eigen::Matrix3f skewMatrix = Sophus::SO3f::hat(rotationVector);
        if (rotationAngle < eps)
        {
            deltaR = Eigen::Matrix3f::Identity() + skewMatrix;
            rightJ = Eigen::Matrix3f::Identity();
        }
        else
        {
            deltaR = Eigen::Matrix3f::Identity() +
                     skewMatrix * sin(rotationAngle) / rotationAngle +
                     skewMatrix * skewMatrix * (1.0f - cos(rotationAngle)) /
                         rotationAngleSquared;
            rightJ = Eigen::Matrix3f::Identity() -
                     skewMatrix * (1.0f - cos(rotationAngle)) /
                         rotationAngleSquared +
                     skewMatrix * skewMatrix *
                         (rotationAngle - sin(rotationAngle)) /
                         (rotationAngleSquared * rotationAngle);
        }
    }

  public:
    /*!
     * @brief        Integration interval in seconds.
     */
    float           deltaT; // integration time
    /*!
     * @brief        Integrated rotation increment.
     */
    Eigen::Matrix3f deltaR;
    /*!
     * @brief        Right Jacobian of the increment.
     */
    Eigen::Matrix3f rightJ; // right jacobian
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/*!
 * @brief        Preintegration of IMU measurements between
 *               keyframes.
 *
 *              Bias-dependent getters take the preintegration
 *              lock; bulk updates run under the same lock
 *              through Reintegrate and MergePrevious.
 */
class Preintegrated
{
    friend class boost::serialization::access;
    /*!
     * @brief        Serializes the preintegrated state and
     *               the stored measurements.
     *
     * @param[in,out] ar
     *                Archive receiving the stored fields.
     * @param[in]    version
     *               Archive version; currently unused.
     */
    template <class Archive>
    void serialize(Archive &ar, [[maybe_unused]] const unsigned int version)
    {
        ar & dT;
        ar &boost::serialization::make_array(C.data(), C.size());
        ar &boost::serialization::make_array(Info.data(), Info.size());
        ar &boost::serialization::make_array(Nga.diagonal().data(),
                                             Nga.diagonal().size());
        ar &boost::serialization::make_array(NgaWalk.diagonal().data(),
                                             NgaWalk.diagonal().size());
        ar & b;
        ar &boost::serialization::make_array(dR.data(), dR.size());
        ar &boost::serialization::make_array(dV.data(), dV.size());
        ar &boost::serialization::make_array(dP.data(), dP.size());
        ar &boost::serialization::make_array(JRg.data(), JRg.size());
        ar &boost::serialization::make_array(JVg.data(), JVg.size());
        ar &boost::serialization::make_array(JVa.data(), JVa.size());
        ar &boost::serialization::make_array(JPg.data(), JPg.size());
        ar &boost::serialization::make_array(JPa.data(), JPa.size());
        ar &boost::serialization::make_array(avgA.data(), avgA.size());
        ar &boost::serialization::make_array(avgW.data(), avgW.size());

        ar & bu;
        ar &boost::serialization::make_array(db.data(), db.size());
        ar & measurements;
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief        Creates a preintegration from a bias and
     *               a calibration.
     *
     * @param[in]    bias_in
     *               Bias integrated against.
     * @param[in]    calib_in
     *               Calibration supplying the noise models.
     */
    Preintegrated(const Bias &bias_in, const Calib &calib_in)
    {
        Nga     = calib_in.Cov;
        NgaWalk = calib_in.CovWalk;
        initialize(bias_in);
    }
    /*!
     * @brief        Copies another preintegration.
     *
     * @param[in]    p_sourcePreintegration_in
     *               Non-owning source; shall be non-null.
     */
    // Copy constructor
    Preintegrated(Preintegrated *p_sourcePreintegration_in) :
        dT(p_sourcePreintegration_in->dT),
        C(p_sourcePreintegration_in->C),
        Info(p_sourcePreintegration_in->Info),
        Nga(p_sourcePreintegration_in->Nga),
        NgaWalk(p_sourcePreintegration_in->NgaWalk),
        b(p_sourcePreintegration_in->b),
        dR(p_sourcePreintegration_in->dR),
        dV(p_sourcePreintegration_in->dV),
        dP(p_sourcePreintegration_in->dP),
        JRg(p_sourcePreintegration_in->JRg),
        JVg(p_sourcePreintegration_in->JVg),
        JVa(p_sourcePreintegration_in->JVa),
        JPg(p_sourcePreintegration_in->JPg),
        JPa(p_sourcePreintegration_in->JPa),
        avgA(p_sourcePreintegration_in->avgA),
        avgW(p_sourcePreintegration_in->avgW),
        bu(p_sourcePreintegration_in->bu),
        db(p_sourcePreintegration_in->db),
        measurements(p_sourcePreintegration_in->measurements)
    {}
    /*!
     * @brief        Creates an empty preintegration.
     */
    Preintegrated() {}
    /*!
     * @brief        Destroys the preintegration.
     */
    ~Preintegrated() {}
    /*!
     * @brief        Copies the state of another
     *               preintegration.
     *
     * @param[in]    p_sourcePreintegrated_in
     *               Non-owning source; shall be non-null.
     */
    void      copyFrom(Preintegrated *p_sourcePreintegrated_in);
    /*!
     * @brief        Resets the deltas for a new bias.
     *
     * @param[in]    referenceBias_in
     *               Bias integrated against.
     */
    void      initialize(const Bias &referenceBias_in);
    /*!
     * @brief        Folds one measurement into the deltas.
     *
     * @param[in]    acceleration_in
     *               Accelerometer sample in metres per second
     *               squared.
     * @param[in]    angularVelocity_in
     *               Gyroscope sample in radians per second.
     * @param[in]    deltaTime_in
     *               Interval since the previous sample, in
     *               seconds.
     */
    void      integrateNewMeasurement(const Eigen::Vector3f &acceleration_in,
                                      const Eigen::Vector3f &angularVelocity_in,
                                      const float           &deltaTime_in);
    /*!
     * @brief        Rebuilds the deltas from the stored
     *               measurements under the updated bias.
     *
     *              Takes the preintegration lock.
     */
    void      reintegrate();
    /*!
     * @brief        Prepends the state of a previous
     *               preintegration.
     *
     *              Takes both preintegration locks; a
     *              self-merge is ignored.
     *
     * @param[in]    p_previousPreintegrated_in
     *               Non-owning previous preintegration; shall
     *               be non-null.
     */
    void      mergePrevious(Preintegrated *p_previousPreintegrated_in);
    /*!
     * @brief        Stores the updated bias and refreshes the
     *               bias difference.
     *
     *              Takes the preintegration lock.
     *
     * @param[in]    updatedBias_in
     *               Updated bias estimate.
     */
    void      setNewBias(const Bias &updatedBias_in);
    /*!
     * @brief        Returns the bias change relative to the
     *               original bias.
     *
     *              Takes the preintegration lock.
     *
     * @param[in]    referenceBias_in
     *               Bias to compare against.
     *
     * @return       Difference between the given bias and the
     *               original bias.
     */
    IMU::Bias getDeltaBias(const Bias &referenceBias_in);

    /*!
     * @brief        Returns the delta rotation corrected for
     *               the given bias.
     *
     *              Takes the preintegration lock.
     *
     * @param[in]    referenceBias_in
     *               Bias to correct for.
     *
     * @return       Bias-corrected delta rotation.
     */
    Eigen::Matrix3f getDeltaRotation(const Bias &referenceBias_in);
    /*!
     * @brief        Returns the delta velocity corrected for
     *               the given bias.
     *
     *              Takes the preintegration lock.
     *
     * @param[in]    referenceBias_in
     *               Bias to correct for.
     *
     * @return       Bias-corrected delta velocity in metres
     *               per second.
     */
    Eigen::Vector3f getDeltaVelocity(const Bias &referenceBias_in);
    /*!
     * @brief        Returns the delta position corrected for
     *               the given bias.
     *
     *              Takes the preintegration lock.
     *
     * @param[in]    referenceBias_in
     *               Bias to correct for.
     *
     * @return       Bias-corrected delta position in metres.
     */
    Eigen::Vector3f getDeltaPosition(const Bias &referenceBias_in);

    /*!
     * @brief        Returns the delta rotation under the
     *               updated bias.
     *
     *              Takes the preintegration lock.
     *
     * @return       Updated delta rotation.
     */
    Eigen::Matrix3f getUpdatedDeltaRotation();
    /*!
     * @brief        Returns the delta velocity under the
     *               updated bias.
     *
     *              Takes the preintegration lock.
     *
     * @return       Updated delta velocity in metres per
     *               second.
     */
    Eigen::Vector3f getUpdatedDeltaVelocity();
    /*!
     * @brief        Returns the delta position under the
     *               updated bias.
     *
     *              Takes the preintegration lock.
     *
     * @return       Updated delta position in metres.
     */
    Eigen::Vector3f getUpdatedDeltaPosition();

    /*!
     * @brief        Returns the delta rotation under the
     *               original bias.
     *
     *              Takes the preintegration lock.
     *
     * @return       Original delta rotation.
     */
    Eigen::Matrix3f getOriginalDeltaRotation();
    /*!
     * @brief        Returns the delta velocity under the
     *               original bias.
     *
     *              Takes the preintegration lock.
     *
     * @return       Original delta velocity in metres per
     *               second.
     */
    Eigen::Vector3f getOriginalDeltaVelocity();
    /*!
     * @brief        Returns the delta position under the
     *               original bias.
     *
     *              Takes the preintegration lock.
     *
     * @return       Original delta position in metres.
     */
    Eigen::Vector3f getOriginalDeltaPosition();

    /*!
     * @brief        Returns the stored bias difference.
     *
     *              Takes the preintegration lock.
     *
     * @return       Six-element difference between the
     *               updated and original biases.
     */
    Eigen::Matrix<float, 6, 1> getDeltaBias();

    /*!
     * @brief        Returns the original integration bias.
     *
     *              Takes the preintegration lock.
     *
     * @return       Bias integrated against.
     */
    Bias getOriginalBias();
    /*!
     * @brief        Returns the latest updated bias.
     *
     *              Takes the preintegration lock.
     *
     * @return       Updated bias estimate.
     */
    Bias getUpdatedBias();

    /*!
     * @brief        Prints the stored measurement timestamps
     *               to standard output.
     */
    void printMeasurements() const
    {
        std::cout << "\nIMU measures: \n";
        for (long unsigned int measurementIndex = 0;
             measurementIndex < measurements.size();
             measurementIndex++)
            std::cout << "- Measurement " << measurements[measurementIndex].t
                      << std::endl;
        std::cout << "Finished printing IMU measures ...\n";
    }

  public:
    /*!
     * @brief        Accumulated integration time in seconds.
     */
    float                           dT;
    /*!
     * @brief        Fifteen-by-fifteen covariance.
     */
    Eigen::Matrix<float, 15, 15>    C;
    /*!
     * @brief        Fifteen-by-fifteen information matrix.
     */
    Eigen::Matrix<float, 15, 15>    Info;
    /*!
     * @brief        Measurement and random-walk noise models.
     */
    Eigen::DiagonalMatrix<float, 6> Nga, NgaWalk;

    // Values for the original bias (when integration was computed)
    /*!
     * @brief        Bias the deltas were integrated against.
     */
    Bias            b;
    /*!
     * @brief        Integrated rotation increment.
     */
    Eigen::Matrix3f dR;
    /*!
     * @brief        Integrated velocity and position deltas.
     */
    Eigen::Vector3f dV, dP;
    /*!
     * @brief        Jacobians of the deltas with respect to
     *               the bias.
     */
    Eigen::Matrix3f JRg, JVg, JVa, JPg, JPa;
    /*!
     * @brief        Mean acceleration and angular rate.
     */
    Eigen::Vector3f avgA, avgW;

  private:
    // Updated bias
    /*!
     * @brief        Latest updated bias estimate.
     */
    Bias                       bu;
    // Dif between original and updated bias
    // This is used to compute the updated values of the preintegration
    /*!
     * @brief        Difference between the updated and the
     *               original bias.
     */
    Eigen::Matrix<float, 6, 1> db;

    /*!
     * @brief        One stored accelerometer and gyroscope
     *               reading.
     */
    struct Integrable
    {
        /*!
         * @brief        Serializes the stored reading.
         *
         * @param[in,out] ar
         *                Archive receiving the stored fields.
         * @param[in]    version
         *               Archive version; currently unused.
         */
        template <class Archive>
        void serialize(Archive &ar, [[maybe_unused]] const unsigned int version)
        {
            ar &boost::serialization::make_array(a.data(), a.size());
            ar &boost::serialization::make_array(w.data(), w.size());
            ar & t;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /*!
         * @brief        Creates an empty stored reading.
         */
        Integrable() {}
        /*!
         * @brief        Creates a stored reading.
         *
         * @param[in]    acceleration_in
         *               Accelerometer sample in metres per
         *               second squared.
         * @param[in]    angularVelocity_in
         *               Gyroscope sample in radians per
         *               second.
         * @param[in]    sampleInterval_in
         *               Interval since the previous sample,
         *               in seconds.
         */
        Integrable(const Eigen::Vector3f &acceleration_in,
                   const Eigen::Vector3f &angularVelocity_in,
                   const float           &sampleInterval_in) :
            a(acceleration_in),
            w(angularVelocity_in),
            t(sampleInterval_in)
        {}
        /*!
         * @brief        Stored accelerometer and gyroscope
         *               samples.
         */
        Eigen::Vector3f a, w;
        /*!
         * @brief        Interval since the previous sample,
         *               in seconds.
         */
        float           t;
    };

    /*!
     * @brief        Stored readings backing reintegration.
     */
    std::vector<Integrable> measurements;

    /*!
     * @brief        Serializes concurrent preintegration
     *               updates.
     */
    std::mutex preintegrationMutex;
};

/*!
 * @brief        Returns the right Jacobian of SO3 at the
 *               given rotation vector.
 *
 * @param[in]    rotationVectorX_in
 *               Rotation x in radians.
 * @param[in]    rotationVectorY_in
 *               Rotation y in radians.
 * @param[in]    rotationVectorZ_in
 *               Rotation z in radians.
 *
 * @return       Three-by-three right Jacobian.
 */
Eigen::Matrix3f rightJacobianSO3(const float &rotationVectorX_in,
                                 const float &rotationVectorY_in,
                                 const float &rotationVectorZ_in);
/*!
 * @brief        Returns the right Jacobian of SO3 at the
 *               given rotation vector.
 *
 * @param[in]    rotationVector_in
 *               Rotation vector in radians.
 *
 * @return       Three-by-three right Jacobian.
 */
Eigen::Matrix3f rightJacobianSO3(const Eigen::Vector3f &rotationVector_in);

/*!
 * @brief        Returns the inverse right Jacobian of SO3 at
 *               the given rotation vector.
 *
 * @param[in]    angleAxisX_in
 *               Rotation x in radians.
 * @param[in]    angleAxisY_in
 *               Rotation y in radians.
 * @param[in]    angleAxisZ_in
 *               Rotation z in radians.
 *
 * @return       Three-by-three inverse right Jacobian.
 */
Eigen::Matrix3f inverseRightJacobianSO3(const float &angleAxisX_in,
                                        const float &angleAxisY_in,
                                        const float &angleAxisZ_in);
/*!
 * @brief        Returns the inverse right Jacobian of SO3 at
 *               the given rotation vector.
 *
 * @param[in]    angleAxisVector_in
 *               Rotation vector in radians.
 *
 * @return       Three-by-three inverse right Jacobian.
 */
Eigen::Matrix3f
    inverseRightJacobianSO3(const Eigen::Vector3f &angleAxisVector_in);

/*!
 * @brief        Re-orthonormalizes a rotation estimate.
 *
 * @param[in]    rotationMatrix_in
 *               Rotation to normalize.
 *
 * @return       Closest rotation in the Frobenius sense.
 */
Eigen::Matrix3f normalizeRotation(const Eigen::Matrix3f &rotationMatrix_in);

} // namespace IMU

} // namespace core
} // namespace vs_graphs

#endif // IMUTYPES_H

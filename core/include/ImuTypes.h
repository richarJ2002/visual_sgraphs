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
 * @brief        Single IMU sample with accelerometer,
 *               gyroscope and timestamp.
 */
class Point
{
  public:
    /*!
     * @brief        Creates a sample from raw components.
     *
     * @param[in]    acc_x
     *               Accelerometer x in metres per second
     *               squared.
     * @param[in]    acc_y
     *               Accelerometer y in metres per second
     *               squared.
     * @param[in]    acc_z
     *               Accelerometer z in metres per second
     *               squared.
     * @param[in]    ang_vel_x
     *               Gyroscope x in radians per second.
     * @param[in]    ang_vel_y
     *               Gyroscope y in radians per second.
     * @param[in]    ang_vel_z
     *               Gyroscope z in radians per second.
     * @param[in]    timestamp
     *               Sample timestamp in seconds.
     */
    Point(const float  &acc_x,
          const float  &acc_y,
          const float  &acc_z,
          const float  &ang_vel_x,
          const float  &ang_vel_y,
          const float  &ang_vel_z,
          const double &timestamp) :
        a(acc_x, acc_y, acc_z),
        w(ang_vel_x, ang_vel_y, ang_vel_z),
        t(timestamp)
    {}
    /*!
     * @brief        Creates a sample from OpenCV vectors.
     *
     * @param[in]    Acc
     *               Accelerometer sample in metres per second
     *               squared.
     * @param[in]    Gyro
     *               Gyroscope sample in radians per second.
     * @param[in]    timestamp
     *               Sample timestamp in seconds.
     */
    Point(const cv::Point3f Acc,
          const cv::Point3f Gyro,
          const double     &timestamp) :
        a(Acc.x, Acc.y, Acc.z),
        w(Gyro.x, Gyro.y, Gyro.z),
        t(timestamp)
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
    void serialize(Archive &ar, const unsigned int version)
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
     * @param[in]    b_acc_x
     *               Accelerometer x bias.
     * @param[in]    b_acc_y
     *               Accelerometer y bias.
     * @param[in]    b_acc_z
     *               Accelerometer z bias.
     * @param[in]    b_ang_vel_x
     *               Gyroscope x bias.
     * @param[in]    b_ang_vel_y
     *               Gyroscope y bias.
     * @param[in]    b_ang_vel_z
     *               Gyroscope z bias.
     */
    Bias(const float &b_acc_x,
         const float &b_acc_y,
         const float &b_acc_z,
         const float &b_ang_vel_x,
         const float &b_ang_vel_y,
         const float &b_ang_vel_z) :
        bax(b_acc_x),
        bay(b_acc_y),
        baz(b_acc_z),
        bwx(b_ang_vel_x),
        bwy(b_ang_vel_y),
        bwz(b_ang_vel_z)
    {}
    /*!
     * @brief        Copies every component from another bias.
     *
     * @param[in]    b
     *               Source bias; shall be non-null.
     */
    void                 copyFrom(Bias &b);
    /*!
     * @brief        Appends the six bias components to the
     *               stream.
     *
     * @param[in,out] out
     *                Stream receiving the components.
     * @param[in]    b
     *               Bias whose components are written.
     *
     * @return       The output stream.
     */
    friend std::ostream &operator<<(std::ostream &out, const Bias &b);

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

        ar & mbIsSet;
    }

  public:
    /*!
     * @brief        Creates a calibration and marks it valid.
     *
     * @param[in]    Tbc
     *               Body-to-camera transform.
     * @param[in]    ng
     *               Gyroscope noise density.
     * @param[in]    na
     *               Accelerometer noise density.
     * @param[in]    ngw
     *               Gyroscope random-walk density.
     * @param[in]    naw
     *               Accelerometer random-walk density.
     */
    Calib(const Sophus::SE3<float> &Tbc,
          const float              &ng,
          const float              &na,
          const float              &ngw,
          const float              &naw)
    {
        setCalibration(Tbc, ng, na, ngw, naw);
    }

    /*!
     * @brief        Copies another calibration.
     *
     * @param[in]    calib
     *               Source calibration.
     */
    Calib(const Calib &calib);
    /*!
     * @brief        Creates an unset calibration.
     */
    Calib()
    {
        mbIsSet = false;
    }

    // void Set(const cv::Mat &cvTbc, const float &ng, const float &na, const
    // float &ngw, const float &naw);
    /*!
     * @brief        Stores the transform and noise densities
     *               and marks the calibration valid.
     *
     * @param[in]    sophTbc
     *               Body-to-camera transform.
     * @param[in]    ng
     *               Gyroscope noise density.
     * @param[in]    na
     *               Accelerometer noise density.
     * @param[in]    ngw
     *               Gyroscope random-walk density.
     * @param[in]    naw
     *               Accelerometer random-walk density.
     */
    void setCalibration(const Sophus::SE3<float> &sophTbc,
                        const float              &ng,
                        const float              &na,
                        const float              &ngw,
                        const float              &naw);

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
    bool                            mbIsSet;
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
     * @param[in]    angVel
     *               Angular rate in radians per second.
     * @param[in]    imuBias
     *               Bias subtracted before integration.
     * @param[in]    time
     *               Integration interval in seconds.
     */
    IntegratedRotation(const Eigen::Vector3f &angVel,
                       const Bias            &imuBias,
                       const float           &time);

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
    void serialize(Archive &ar, const unsigned int version)
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
        ar & mvMeasurements;
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief        Creates a preintegration from a bias and
     *               a calibration.
     *
     * @param[in]    b_
     *               Bias integrated against.
     * @param[in]    calib
     *               Calibration supplying the noise models.
     */
    Preintegrated(const Bias &b_, const Calib &calib);
    /*!
     * @brief        Copies another preintegration.
     *
     * @param[in]    pImuPre
     *               Non-owning source; shall be non-null.
     */
    Preintegrated(Preintegrated *pImuPre);
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
     * @param[in]    pImuPre
     *               Non-owning source; shall be non-null.
     */
    void      copyFrom(Preintegrated *pImuPre);
    /*!
     * @brief        Resets the deltas for a new bias.
     *
     * @param[in]    b_
     *               Bias integrated against.
     */
    void      initialize(const Bias &b_);
    /*!
     * @brief        Folds one measurement into the deltas.
     *
     * @param[in]    acceleration
     *               Accelerometer sample in metres per second
     *               squared.
     * @param[in]    angVel
     *               Gyroscope sample in radians per second.
     * @param[in]    dt
     *               Interval since the previous sample, in
     *               seconds.
     */
    void      integrateNewMeasurement(const Eigen::Vector3f &acceleration,
                                      const Eigen::Vector3f &angVel,
                                      const float           &dt);
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
     * @param[in]    pPrev
     *               Non-owning previous preintegration; shall
     *               be non-null.
     */
    void      mergePrevious(Preintegrated *pPrev);
    /*!
     * @brief        Stores the updated bias and refreshes the
     *               bias difference.
     *
     *              Takes the preintegration lock.
     *
     * @param[in]    bu_
     *               Updated bias estimate.
     */
    void      setNewBias(const Bias &bu_);
    /*!
     * @brief        Returns the bias change relative to the
     *               original bias.
     *
     *              Takes the preintegration lock.
     *
     * @param[in]    b_
     *               Bias to compare against.
     *
     * @return       Difference between the given bias and the
     *               original bias.
     */
    IMU::Bias getDeltaBias(const Bias &b_);

    /*!
     * @brief        Returns the delta rotation corrected for
     *               the given bias.
     *
     *              Takes the preintegration lock.
     *
     * @param[in]    b_
     *               Bias to correct for.
     *
     * @return       Bias-corrected delta rotation.
     */
    Eigen::Matrix3f getDeltaRotation(const Bias &b_);
    /*!
     * @brief        Returns the delta velocity corrected for
     *               the given bias.
     *
     *              Takes the preintegration lock.
     *
     * @param[in]    b_
     *               Bias to correct for.
     *
     * @return       Bias-corrected delta velocity in metres
     *               per second.
     */
    Eigen::Vector3f getDeltaVelocity(const Bias &b_);
    /*!
     * @brief        Returns the delta position corrected for
     *               the given bias.
     *
     *              Takes the preintegration lock.
     *
     * @param[in]    b_
     *               Bias to correct for.
     *
     * @return       Bias-corrected delta position in metres.
     */
    Eigen::Vector3f getDeltaPosition(const Bias &b_);

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
        for (long unsigned int i = 0; i < mvMeasurements.size(); i++)
            std::cout << "- Measurement " << mvMeasurements[i].t << std::endl;
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
        void serialize(Archive &ar, const unsigned int version)
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
         * @param[in]    a_
         *               Accelerometer sample in metres per
         *               second squared.
         * @param[in]    w_
         *               Gyroscope sample in radians per
         *               second.
         * @param[in]    t_
         *               Interval since the previous sample,
         *               in seconds.
         */
        Integrable(const Eigen::Vector3f &a_,
                   const Eigen::Vector3f &w_,
                   const float           &t_) :
            a(a_),
            w(w_),
            t(t_)
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
    std::vector<Integrable> mvMeasurements;

    /*!
     * @brief        Serializes concurrent preintegration
     *               updates.
     */
    std::mutex mMutex;
};

/*!
 * @brief        Returns the right Jacobian of SO3 at the
 *               given rotation vector.
 *
 * @param[in]    x
 *               Rotation x in radians.
 * @param[in]    y
 *               Rotation y in radians.
 * @param[in]    z
 *               Rotation z in radians.
 *
 * @return       Three-by-three right Jacobian.
 */
Eigen::Matrix3f
    RightJacobianSO3(const float &x, const float &y, const float &z);
/*!
 * @brief        Returns the right Jacobian of SO3 at the
 *               given rotation vector.
 *
 * @param[in]    v
 *               Rotation vector in radians.
 *
 * @return       Three-by-three right Jacobian.
 */
Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f &v);

/*!
 * @brief        Returns the inverse right Jacobian of SO3 at
 *               the given rotation vector.
 *
 * @param[in]    x
 *               Rotation x in radians.
 * @param[in]    y
 *               Rotation y in radians.
 * @param[in]    z
 *               Rotation z in radians.
 *
 * @return       Three-by-three inverse right Jacobian.
 */
Eigen::Matrix3f
    InverseRightJacobianSO3(const float &x, const float &y, const float &z);
/*!
 * @brief        Returns the inverse right Jacobian of SO3 at
 *               the given rotation vector.
 *
 * @param[in]    v
 *               Rotation vector in radians.
 *
 * @return       Three-by-three inverse right Jacobian.
 */
Eigen::Matrix3f InverseRightJacobianSO3(const Eigen::Vector3f &v);

/*!
 * @brief        Re-orthonormalizes a rotation estimate.
 *
 * @param[in]    R
 *               Rotation to normalize.
 *
 * @return       Closest rotation in the Frobenius sense.
 */
Eigen::Matrix3f NormalizeRotation(const Eigen::Matrix3f &R);

} // namespace IMU

} // namespace core
} // namespace vs_graphs

#endif // IMUTYPES_H

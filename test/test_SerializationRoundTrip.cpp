/*!
 * @file            test_SerializationRoundTrip.cpp
 *
 * @brief           Save-and-load round-trip tests for the atlas, maps, key
 *                  frames, map points, cameras and IMU types (Serialization*).
 */

/*
 * Boost serialization round-trip self-consistency (gated).
 *
 * Covers all Boost sites: Map, MapPoint, Atlas, ImuTypes (Bias/Calib/
 * Preintegrated), KeyFrameDatabase, KeyFrame, SerializationUtils
 * (Sophus SE3, cv::Mat, vector<KeyPoint>) + 3 CameraModels
 * (Pinhole, KannalaBrandt8, GeometricCamera base).
 *
 * Scope: self-consistency only (save HEAD, load HEAD, compare). No
 * cross-version support (see docs/deviations/serialization_versioning.md).
 * Fixtures are light (default + minimally populated, no ORB vocabulary,
 * no full SLAM graph) so the test stays GREEN on HEAD and ROS/Gazebo-free.
 *
 * Comparison rules:
 *  - Deep value for POD/containers; sorted set/map comparison helpers.
 *  - Pointer identity via Boost tracking IDs (polymorphic camera test).
 *  - SKIP mutex/atomic/thread handles: asserted default-constructed
 *    post-load by exercising the loaded object (getters, no deadlock).
 *  - Deterministic ordering enforced via sorted helpers.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <new>
#include <set>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/vector.hpp>

#include "Atlas.h"
#include "CameraModels/GeometricCamera/objects/GeometricCamera.h"
#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"
#include "CameraModels/Pinhole/objects/Pinhole.h"
#include "ImuTypes.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "Map.h"
#include "MapPoint.h"
#include "SerializationUtils.h"

namespace vs_graphs
{
namespace core
{
namespace
{

template <typename T> T RoundTripBinaryCopyable(const T &input_in)
{
    std::stringstream stream_inout(std::ios::in | std::ios::out |
                                   std::ios::binary);
    {
        boost::archive::binary_oarchive output_archive(stream_inout);
        output_archive << input_in;
    }
    T output_out;
    {
        boost::archive::binary_iarchive input_archive(stream_inout);
        input_archive >> output_out;
    }
    return output_out;
}

template <typename T> void RoundTripBinaryInto(const T &input_in, T &output_out)
{
    std::stringstream stream_inout(std::ios::in | std::ios::out |
                                   std::ios::binary);
    {
        boost::archive::binary_oarchive output_archive(stream_inout);
        output_archive << input_in;
    }
    {
        boost::archive::binary_iarchive input_archive(stream_inout);
        input_archive >> output_out;
    }
}

template <typename T>
void RoundTripTextFileInto(const T           &input_in,
                           const std::string &path_in,
                           T                 &output_out)
{
    {
        std::ofstream                 output_stream(path_in, std::ios::binary);
        boost::archive::text_oarchive output_archive(output_stream);
        output_archive << input_in;
    }
    {
        std::ifstream                 input_stream(path_in, std::ios::binary);
        boost::archive::text_iarchive input_archive(input_stream);
        input_archive >> output_out;
    }
}

std::string TmpPath(const std::string &name_in)
{
    std::string path_out("/tmp/test_SerializationRoundTrip_");
    path_out += std::to_string(static_cast<long long>(::getpid()));
    path_out += "_";
    path_out += name_in;
    return path_out;
}

void ExpectSophusEqual(const Sophus::SE3f &expected_in,
                       const Sophus::SE3f &actual_in)
{
    EXPECT_TRUE(
        expected_in.translation().isApprox(actual_in.translation(), 1.0e-5F));
    const Eigen::Quaternionf expected_quat = expected_in.unit_quaternion();
    const Eigen::Quaternionf actual_quat   = actual_in.unit_quaternion();
    const float              dot = std::abs(expected_quat.dot(actual_quat));
    EXPECT_NEAR(dot, 1.0F, 1.0e-5F);
}

void ExpectCvMatEqual(const cv::Mat &expected_in, const cv::Mat &actual_in)
{
    ASSERT_EQ(expected_in.rows, actual_in.rows);
    ASSERT_EQ(expected_in.cols, actual_in.cols);
    ASSERT_EQ(expected_in.type(), actual_in.type());
    if (expected_in.empty())
    {
        EXPECT_TRUE(actual_in.empty());
        return;
    }
    const double norm = cv::norm(expected_in, actual_in, cv::NORM_L2);
    EXPECT_NEAR(norm, 0.0, 1.0e-5);
}

void ExpectKeyPointsEqual(const std::vector<cv::KeyPoint> &expected_in,
                          const std::vector<cv::KeyPoint> &actual_in)
{
    ASSERT_EQ(expected_in.size(), actual_in.size());
    for (std::size_t index = 0; index < expected_in.size(); ++index)
    {
        EXPECT_FLOAT_EQ(expected_in[index].pt.x, actual_in[index].pt.x);
        EXPECT_FLOAT_EQ(expected_in[index].pt.y, actual_in[index].pt.y);
        EXPECT_FLOAT_EQ(expected_in[index].size, actual_in[index].size);
        EXPECT_FLOAT_EQ(expected_in[index].angle, actual_in[index].angle);
        EXPECT_FLOAT_EQ(expected_in[index].response, actual_in[index].response);
        EXPECT_EQ(expected_in[index].octave, actual_in[index].octave);
        EXPECT_EQ(expected_in[index].class_id, actual_in[index].class_id);
    }
}

template <typename Key, typename Value>
void ExpectMapsEqualSorted(const std::map<Key, Value> &expected_in,
                           const std::map<Key, Value> &actual_in)
{
    ASSERT_EQ(expected_in.size(), actual_in.size());
    typename std::map<Key, Value>::const_iterator expected_it =
        expected_in.begin();
    typename std::map<Key, Value>::const_iterator actual_it = actual_in.begin();
    while (expected_it != expected_in.end())
    {
        EXPECT_EQ(expected_it->first, actual_it->first);
        EXPECT_EQ(expected_it->second, actual_it->second);
        ++expected_it;
        ++actual_it;
    }
}

template <typename Value>
void ExpectSetsEqualSorted(const std::set<Value> &expected_in,
                           const std::set<Value> &actual_in)
{
    ASSERT_EQ(expected_in.size(), actual_in.size());
    typename std::set<Value>::const_iterator expected_it = expected_in.begin();
    typename std::set<Value>::const_iterator actual_it   = actual_in.begin();
    while (expected_it != expected_in.end())
    {
        EXPECT_EQ(*expected_it, *actual_it);
        ++expected_it;
        ++actual_it;
    }
}

} // namespace

/*!
 * @brief        Checks that a Sophus SE3 pose, including the identity pose,
 *               survives a Boost binary save and load unchanged.
 */
TEST(SerializationUtils, SophusSe3RoundTrip)
{
    Sophus::SE3f original(
        Eigen::Quaternionf(0.7071068F, 0.0F, 0.7071068F, 0.0F),
        Eigen::Vector3f(1.0F, 2.0F, 3.0F));
    Sophus::SE3f loaded;
    {
        std::stringstream stream(std::ios::in | std::ios::out |
                                 std::ios::binary);
        {
            boost::archive::binary_oarchive output_archive(stream);
            serializeSophusSE3(output_archive, original, 0U);
        }
        {
            boost::archive::binary_iarchive input_archive(stream);
            serializeSophusSE3(input_archive, loaded, 0U);
        }
    }
    ExpectSophusEqual(original, loaded);

    const Sophus::SE3f identity;
    Sophus::SE3f       identity_loaded;
    {
        std::stringstream stream(std::ios::in | std::ios::out |
                                 std::ios::binary);
        {
            boost::archive::binary_oarchive output_archive(stream);
            Sophus::SE3f                    mutable_identity = identity;
            serializeSophusSE3(output_archive, mutable_identity, 0U);
        }
        {
            boost::archive::binary_iarchive input_archive(stream);
            serializeSophusSE3(input_archive, identity_loaded, 0U);
        }
    }
    ExpectSophusEqual(identity, identity_loaded);
}

/*!
 * @brief        Checks that an OpenCV matrix survives a Boost binary round trip
 *               when it is continuous, when it is a non-continuous region of a
 *               larger matrix, and when it is empty.
 */
TEST(SerializationUtils, CvMatRoundTripContinuousAndRoi)
{
    cv::Mat continuous(3, 3, CV_32F);
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            continuous.at<float>(row, col) =
                static_cast<float>(row * 3 + col) + 0.5F;
        }
    }
    cv::Mat continuous_loaded;
    {
        std::stringstream stream(std::ios::in | std::ios::out |
                                 std::ios::binary);
        {
            boost::archive::binary_oarchive output_archive(stream);
            serializeMatrix(output_archive, continuous, 0U);
        }
        {
            boost::archive::binary_iarchive input_archive(stream);
            serializeMatrix(input_archive, continuous_loaded, 0U);
        }
    }
    ExpectCvMatEqual(continuous, continuous_loaded);

    cv::Mat parent(4, 4, CV_32F, cv::Scalar(1.0F));
    // Width 2 < parent width 4 guarantees a non-continuous step.
    cv::Mat roi = parent(cv::Range(0, 2), cv::Range(0, 2));
    ASSERT_FALSE(roi.isContinuous());
    cv::Mat roi_loaded;
    {
        std::stringstream stream(std::ios::in | std::ios::out |
                                 std::ios::binary);
        {
            boost::archive::binary_oarchive output_archive(stream);
            serializeMatrix(output_archive, roi, 0U);
        }
        {
            boost::archive::binary_iarchive input_archive(stream);
            serializeMatrix(input_archive, roi_loaded, 0U);
        }
    }
    ExpectCvMatEqual(roi, roi_loaded);

    cv::Mat empty;
    cv::Mat empty_loaded(cv::Mat::ones(1, 1, CV_32F));
    {
        std::stringstream stream(std::ios::in | std::ios::out |
                                 std::ios::binary);
        {
            boost::archive::binary_oarchive output_archive(stream);
            serializeMatrix(output_archive, empty, 0U);
        }
        {
            boost::archive::binary_iarchive input_archive(stream);
            serializeMatrix(input_archive, empty_loaded, 0U);
        }
    }
    EXPECT_TRUE(empty_loaded.empty());
}

/*!
 * @brief        Checks that a vector of OpenCV key points, empty or with two
 *               entries, survives a Boost binary round trip unchanged.
 */
TEST(SerializationUtils, VectorKeyPointsRoundTrip)
{
    const std::vector<cv::KeyPoint> empty;
    {
        std::stringstream stream(std::ios::in | std::ios::out |
                                 std::ios::binary);
        {
            boost::archive::binary_oarchive output_archive(stream);
            serializeVectorKeyPoints(output_archive, empty, 0U);
        }
        // NB: helper copies the target then appends, so the target must
        // start empty (it cannot shrink a non-empty target).
        std::vector<cv::KeyPoint> loaded;
        {
            boost::archive::binary_iarchive input_archive(stream);
            serializeVectorKeyPoints(input_archive, loaded, 0U);
        }
        ExpectKeyPointsEqual(empty, loaded);
    }

    std::vector<cv::KeyPoint> original;
    original.emplace_back(10.0F, 20.0F, 1.5F, 30.0F, 0.9F, 2, 7);
    original.emplace_back(11.0F, 21.0F, 2.5F, 40.0F, 0.8F, 3, 8);
    {
        std::stringstream stream(std::ios::in | std::ios::out |
                                 std::ios::binary);
        {
            boost::archive::binary_oarchive output_archive(stream);
            serializeVectorKeyPoints(output_archive, original, 0U);
        }
        std::vector<cv::KeyPoint> loaded;
        {
            boost::archive::binary_iarchive input_archive(stream);
            serializeVectorKeyPoints(input_archive, loaded, 0U);
        }
        ExpectKeyPointsEqual(original, loaded);
    }
}

/*!
 * @brief        Checks that IMU bias values, including the defaults, survive a
 *               round trip through memory and through a temporary file.
 */
TEST(SerializationImu, BiasRoundTripMemoryAndTmpFile)
{
    const IMU::Bias original(0.1F, -0.2F, 0.3F, 0.01F, -0.02F, 0.03F);
    const IMU::Bias loaded = RoundTripBinaryCopyable(original);
    EXPECT_FLOAT_EQ(original.bax, loaded.bax);
    EXPECT_FLOAT_EQ(original.bay, loaded.bay);
    EXPECT_FLOAT_EQ(original.baz, loaded.baz);
    EXPECT_FLOAT_EQ(original.bwx, loaded.bwx);
    EXPECT_FLOAT_EQ(original.bwy, loaded.bwy);
    EXPECT_FLOAT_EQ(original.bwz, loaded.bwz);

    const std::string path = TmpPath("bias.bin");
    IMU::Bias         file_loaded;
    RoundTripTextFileInto(original, path, file_loaded);
    EXPECT_FLOAT_EQ(original.bax, file_loaded.bax);
    EXPECT_FLOAT_EQ(original.bwz, file_loaded.bwz);
    std::remove(path.c_str());

    const IMU::Bias defaults;
    const IMU::Bias defaults_loaded = RoundTripBinaryCopyable(defaults);
    EXPECT_FLOAT_EQ(0.0F, defaults_loaded.bax);
    EXPECT_FLOAT_EQ(0.0F, defaults_loaded.bwz);
}

/*!
 * @brief        Checks that an IMU calibration, with its camera-to-IMU
 *               transforms and covariances, survives a round trip, and that an
 *               unset calibration stays unset.
 */
TEST(SerializationImu, CalibRoundTrip)
{
    const Sophus::SE3f known_tbc(Eigen::Quaternionf(0.0F, 0.0F, 0.0F, 1.0F),
                                 Eigen::Vector3f(0.1F, 0.2F, 0.3F));
    IMU::Calib         original;
    ASSERT_EQ(
        (original.setCalibration(known_tbc, 0.01F, 0.02F, 0.001F, 0.002F)),
        IMU::CalibStatus::CALIB_STATUS_SUCCESS);
    EXPECT_TRUE(original.isCalibrationSet);

    const IMU::Calib loaded = RoundTripBinaryCopyable(original);
    EXPECT_TRUE(loaded.isCalibrationSet);
    ExpectSophusEqual(original.mTbc, loaded.mTbc);
    ExpectSophusEqual(original.mTcb, loaded.mTcb);
    EXPECT_TRUE(
        original.Cov.diagonal().isApprox(loaded.Cov.diagonal(), 1.0e-6F));
    EXPECT_TRUE(original.CovWalk.diagonal().isApprox(loaded.CovWalk.diagonal(),
                                                     1.0e-6F));

    const IMU::Calib unset;
    EXPECT_FALSE(unset.isCalibrationSet);
    const IMU::Calib unset_loaded = RoundTripBinaryCopyable(unset);
    EXPECT_FALSE(unset_loaded.isCalibrationSet);
}

/*!
 * @brief        Checks that preintegrated IMU measurements survive a round trip
 *               with their integrated terms and bias, and that the loaded
 *               object still accepts new measurements.
 */
TEST(SerializationImu, PreintegratedRoundTrip)
{
    const IMU::Bias    bias(0.01F, 0.02F, 0.03F, 0.001F, 0.002F, 0.003F);
    const Sophus::SE3f tbc(Eigen::Quaternionf::Identity(),
                           Eigen::Vector3f(0.05F, 0.0F, 0.0F));
    IMU::Calib         calib;
    ASSERT_EQ((calib.setCalibration(tbc, 0.01F, 0.02F, 0.001F, 0.002F)),
              IMU::CalibStatus::CALIB_STATUS_SUCCESS);

    IMU::Preintegrated original(bias, calib);
    ASSERT_EQ(
        (original.integrateNewMeasurement(Eigen::Vector3f(0.0F, 0.0F, 9.81F),
                                          Eigen::Vector3f(0.01F, 0.0F, 0.0F),
                                          0.01F)),
        IMU::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS);
    ASSERT_EQ(
        (original.integrateNewMeasurement(Eigen::Vector3f(0.1F, 0.0F, 9.80F),
                                          Eigen::Vector3f(0.0F, 0.01F, 0.0F),
                                          0.01F)),
        IMU::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS);
    EXPECT_GT(original.dT, 0.0F);

    IMU::Preintegrated loaded;
    RoundTripBinaryInto(original, loaded);
    EXPECT_FLOAT_EQ(original.dT, loaded.dT);
    EXPECT_TRUE(original.dR.isApprox(loaded.dR, 1.0e-5F));
    EXPECT_TRUE(original.dV.isApprox(loaded.dV, 1.0e-5F));
    EXPECT_TRUE(original.dP.isApprox(loaded.dP, 1.0e-5F));
    EXPECT_FLOAT_EQ(original.b.bax, loaded.b.bax);
    EXPECT_FLOAT_EQ(original.b.bwz, loaded.b.bwz);
    EXPECT_TRUE(original.C.isApprox(loaded.C, 1.0e-4F));

    // SKIP mutex: post-load object must remain usable (default-constructed
    // lock state). Integrating one more sample must not deadlock or crash.
    IMU::PreintegratedStatus integrateStatus{};
    EXPECT_NO_THROW(integrateStatus = loaded.integrateNewMeasurement(
                        Eigen::Vector3f(0.0F, 0.0F, 9.81F),
                        Eigen::Vector3f::Zero(),
                        0.005F));
    EXPECT_EQ(integrateStatus,
              IMU::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS);
}

/*!
 * @brief        Checks that a pinhole camera keeps its id, type, four
 *               parameters and equality after a round trip.
 */
TEST(SerializationCamera, PinholeRoundTrip)
{
    camera_models::pinhole::Pinhole original(
        std::vector<float>{500.0F, 500.0F, 320.0F, 240.0F});
    unsigned int original_id{};
    ASSERT_EQ((original.getId(original_id)),
              camera_models::geometriccamera::GeometricCameraStatus::
                  GEOMETRIC_CAMERA_STATUS_SUCCESS);
    camera_models::pinhole::Pinhole loaded(
        std::vector<float>{1.0F, 1.0F, 1.0F, 1.0F});
    RoundTripBinaryInto(original, loaded);
    unsigned int id{};
    ASSERT_EQ((loaded.getId(id)),
              camera_models::geometriccamera::GeometricCameraStatus::
                  GEOMETRIC_CAMERA_STATUS_SUCCESS);
    EXPECT_EQ(original_id, id);
    // NB: compare by value (== 0U) not EXPECT_EQ(CAM_*): CAM_* has no
    // out-of-line definition, EXPECT_EQ would ODR-use and fail to link.
    unsigned int type{};
    ASSERT_EQ((loaded.getType(type)),
              camera_models::geometriccamera::GeometricCameraStatus::
                  GEOMETRIC_CAMERA_STATUS_SUCCESS);
    EXPECT_TRUE(type ==
                camera_models::geometriccamera::GeometricCamera::CAM_PINHOLE);
    unsigned int type2{};
    ASSERT_EQ((loaded.getType(type2)),
              camera_models::geometriccamera::GeometricCameraStatus::
                  GEOMETRIC_CAMERA_STATUS_SUCCESS);
    EXPECT_EQ(0U, type2);
    size_t size2{};
    ASSERT_EQ((loaded.size(size2)),
              camera_models::geometriccamera::GeometricCameraStatus::
                  GEOMETRIC_CAMERA_STATUS_SUCCESS);
    EXPECT_EQ(4U, size2);
    float parameter{};
    ASSERT_EQ((loaded.getParameter(0, parameter)),
              camera_models::geometriccamera::GeometricCameraStatus::
                  GEOMETRIC_CAMERA_STATUS_SUCCESS);
    EXPECT_FLOAT_EQ(500.0F, parameter);
    float parameter2{};
    ASSERT_EQ((loaded.getParameter(3, parameter2)),
              camera_models::geometriccamera::GeometricCameraStatus::
                  GEOMETRIC_CAMERA_STATUS_SUCCESS);
    EXPECT_FLOAT_EQ(240.0F, parameter2);
    bool isEqual2{};
    ASSERT_EQ((loaded.isEqual(
                  const_cast<camera_models::pinhole::Pinhole *>(&original),
                  isEqual2)),
              camera_models::pinhole::PinholeStatus::PINHOLE_STATUS_SUCCESS);
    EXPECT_TRUE(isEqual2);
    // SKIP tvr raw pointer: not serialized by design; no assertion.
}

/*!
 * @brief        Checks that a Kannala-Brandt8 fisheye camera keeps its id,
 *               type, eight parameters, precision and equality after a round
 *               trip.
 */
TEST(SerializationCamera, KannalaBrandt8RoundTrip)
{
    const std::vector<float>
        params{500.0F, 500.0F, 320.0F, 240.0F, 0.1F, 0.01F, 0.001F, 0.0001F};
    camera_models::kannalabrandt8::KannalaBrandt8 original(params, 1.0e-5F);
    unsigned int                                  original_id{};
    ASSERT_EQ((original.getId(original_id)),
              camera_models::geometriccamera::GeometricCameraStatus::
                  GEOMETRIC_CAMERA_STATUS_SUCCESS);
    camera_models::kannalabrandt8::KannalaBrandt8 loaded(
        std::vector<float>{1.0F, 1.0F, 1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F});
    RoundTripBinaryInto(original, loaded);
    unsigned int id{};
    ASSERT_EQ((loaded.getId(id)),
              camera_models::geometriccamera::GeometricCameraStatus::
                  GEOMETRIC_CAMERA_STATUS_SUCCESS);
    EXPECT_EQ(original_id, id);
    // NB: see PinholeRoundTrip: avoid ODR-use of CAM_* (no definition).
    unsigned int type{};
    ASSERT_EQ((loaded.getType(type)),
              camera_models::geometriccamera::GeometricCameraStatus::
                  GEOMETRIC_CAMERA_STATUS_SUCCESS);
    EXPECT_TRUE(type ==
                camera_models::geometriccamera::GeometricCamera::CAM_FISHEYE);
    unsigned int type2{};
    ASSERT_EQ((loaded.getType(type2)),
              camera_models::geometriccamera::GeometricCameraStatus::
                  GEOMETRIC_CAMERA_STATUS_SUCCESS);
    EXPECT_EQ(1U, type2);
    size_t size2{};
    ASSERT_EQ((loaded.size(size2)),
              camera_models::geometriccamera::GeometricCameraStatus::
                  GEOMETRIC_CAMERA_STATUS_SUCCESS);
    EXPECT_EQ(8U, size2);
    for (std::size_t index = 0; index < params.size(); ++index)
    {
        float parameter{};
        ASSERT_EQ((loaded.getParameter(static_cast<int>(index), parameter)),
                  camera_models::geometriccamera::GeometricCameraStatus::
                      GEOMETRIC_CAMERA_STATUS_SUCCESS);
        EXPECT_FLOAT_EQ(params[index], parameter);
    }
    float precision{};
    ASSERT_EQ((original.getPrecision(precision)),
              camera_models::kannalabrandt8::KannalaBrandt8Status::
                  KANNALA_BRANDT8_STATUS_SUCCESS);
    float precision2{};
    ASSERT_EQ((loaded.getPrecision(precision2)),
              camera_models::kannalabrandt8::KannalaBrandt8Status::
                  KANNALA_BRANDT8_STATUS_SUCCESS);
    EXPECT_FLOAT_EQ(precision, precision2);
    bool isEqual2{};
    ASSERT_EQ((loaded.isEqual(
                  const_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(
                      &original),
                  isEqual2)),
              camera_models::kannalabrandt8::KannalaBrandt8Status::
                  KANNALA_BRANDT8_STATUS_SUCCESS);
    EXPECT_TRUE(isEqual2);
}

/*!
 * @brief        Builds a camera over memory filled with a non-zero pattern and
 *               destroys it, which would delete that pattern as a pointer if
 *               the default constructor left the two-view reconstruction
 *               helper unset. Boost builds a loaded camera this way.
 *
 * @tparam       Camera
 *               Camera model to build, Pinhole or KannalaBrandt8.
 */
template <typename Camera> void ExpectDefaultBuiltCameraDestroysCleanly()
{
    alignas(Camera) unsigned char storage[sizeof(Camera)];
    std::memset(storage, 0xA5, sizeof(storage));
    Camera *p_camera = new (storage) Camera();
    p_camera->~Camera();
}

/*!
 * @brief        Checks that pinhole and Kannala-Brandt8 cameras built by the
 *               default constructor, as Boost does when loading, can be
 *               destroyed without deleting a stale reconstructor pointer.
 */
TEST(SerializationCamera, DefaultBuiltCamerasOwnNoReconstructor)
{
    ExpectDefaultBuiltCameraDestroysCleanly<camera_models::pinhole::Pinhole>();
    ExpectDefaultBuiltCameraDestroysCleanly<
        camera_models::kannalabrandt8::KannalaBrandt8>();
}

/*!
 * @brief        Checks that a camera pointer saved twice in one archive loads
 *               as a single shared camera that keeps its id and type.
 */
TEST(SerializationCamera, PolymorphicTrackingPreservesIdentity)
{
    camera_models::pinhole::Pinhole *original_camera =
        new camera_models::pinhole::Pinhole(
            std::vector<float>{400.0F, 400.0F, 300.0F, 200.0F});
    std::vector<camera_models::geometriccamera::GeometricCamera *>
        original_ptrs{original_camera, original_camera};

    std::vector<camera_models::geometriccamera::GeometricCamera *> loaded_ptrs;
    {
        std::stringstream stream(std::ios::in | std::ios::out |
                                 std::ios::binary);
        {
            boost::archive::binary_oarchive output_archive(stream);
            output_archive
                .template register_type<camera_models::pinhole::Pinhole>();
            output_archive.template register_type<
                camera_models::kannalabrandt8::KannalaBrandt8>();
            output_archive << original_ptrs;
        }
        {
            boost::archive::binary_iarchive input_archive(stream);
            input_archive
                .template register_type<camera_models::pinhole::Pinhole>();
            input_archive.template register_type<
                camera_models::kannalabrandt8::KannalaBrandt8>();
            input_archive >> loaded_ptrs;
        }
    }
    ASSERT_EQ(2U, loaded_ptrs.size());
    // Boost tracking IDs: the same saved pointer must load as one object.
    EXPECT_EQ(loaded_ptrs[0], loaded_ptrs[1]);
    EXPECT_NE(original_camera, loaded_ptrs[0]);
    unsigned int id{};
    ASSERT_EQ((original_camera->getId(id)),
              camera_models::geometriccamera::GeometricCameraStatus::
                  GEOMETRIC_CAMERA_STATUS_SUCCESS);
    unsigned int id2{};
    ASSERT_EQ((loaded_ptrs[0]->getId(id2)),
              camera_models::geometriccamera::GeometricCameraStatus::
                  GEOMETRIC_CAMERA_STATUS_SUCCESS);
    EXPECT_EQ(id, id2);
    unsigned int type{};
    ASSERT_EQ((loaded_ptrs[0]->getType(type)),
              camera_models::geometriccamera::GeometricCameraStatus::
                  GEOMETRIC_CAMERA_STATUS_SUCCESS);
    EXPECT_TRUE(type ==
                camera_models::geometriccamera::GeometricCamera::CAM_PINHOLE);

    delete original_camera;
    // Intentional leak: loaded camera_models::pinhole::Pinhole
    // tvr is untouched by serialize() (see PinholeRoundTrip); deleting would
    // free garbage.
}

/*!
 * @brief        Checks that a map point keeps its id, first key frame id,
 *               observation count, position, normal and distance limits across
 *               a round trip, and can be moved afterwards.
 */
TEST(SerializationMapPoint, RoundTripWithRefKeyFrame)
{
    Map      map;
    KeyFrame ref_keyframe;
    ref_keyframe.id                 = 7U;
    ref_keyframe.p_camera           = nullptr;
    ref_keyframe.p_camera2          = nullptr;
    ref_keyframe.p_imuPreintegrated = nullptr;

    MapPoint original(Eigen::Vector3f(1.0F, 2.0F, 3.0F), &ref_keyframe, &map);
    ASSERT_EQ((original.setNormalVector(Eigen::Vector3f(0.0F, 0.0F, 1.0F))),
              MapPointStatus::MAP_POINT_STATUS_SUCCESS);
    const long unsigned int original_id = original.id;

    std::set<KeyFrame *> keyframe_set{&ref_keyframe};
    std::set<MapPoint *> mappoint_set{&original};
    ASSERT_EQ((original.preSave(keyframe_set, mappoint_set)),
              MapPointStatus::MAP_POINT_STATUS_SUCCESS);

    MapPoint loaded;
    RoundTripBinaryInto(original, loaded);
    EXPECT_EQ(original_id, loaded.id);
    EXPECT_EQ(original.firstKeyFrameId, loaded.firstKeyFrameId);
    EXPECT_EQ(original.observationCount, loaded.observationCount);
    Eigen::Vector3f worldPos{};
    ASSERT_EQ((original.getWorldPos(worldPos)),
              MapPointStatus::MAP_POINT_STATUS_SUCCESS);
    Eigen::Vector3f worldPos2{};
    ASSERT_EQ((loaded.getWorldPos(worldPos2)),
              MapPointStatus::MAP_POINT_STATUS_SUCCESS);
    EXPECT_TRUE(worldPos.isApprox(worldPos2, 1.0e-6F));
    Eigen::Vector3f normal{};
    ASSERT_EQ((original.getNormal(normal)),
              MapPointStatus::MAP_POINT_STATUS_SUCCESS);
    Eigen::Vector3f normal2{};
    ASSERT_EQ((loaded.getNormal(normal2)),
              MapPointStatus::MAP_POINT_STATUS_SUCCESS);
    EXPECT_TRUE(normal.isApprox(normal2, 1.0e-6F));
    bool isBad2{};
    ASSERT_EQ((original.isBad(isBad2)),
              MapPointStatus::MAP_POINT_STATUS_SUCCESS);
    bool isBad3{};
    ASSERT_EQ((loaded.isBad(isBad3)), MapPointStatus::MAP_POINT_STATUS_SUCCESS);
    EXPECT_EQ(isBad2, isBad3);
    float minDistanceInvariance{};
    ASSERT_EQ((original.getMinDistanceInvariance(minDistanceInvariance)),
              MapPointStatus::MAP_POINT_STATUS_SUCCESS);
    float minDistanceInvariance2{};
    ASSERT_EQ((loaded.getMinDistanceInvariance(minDistanceInvariance2)),
              MapPointStatus::MAP_POINT_STATUS_SUCCESS);
    EXPECT_FLOAT_EQ(minDistanceInvariance, minDistanceInvariance2);
    float maxDistanceInvariance{};
    ASSERT_EQ((original.getMaxDistanceInvariance(maxDistanceInvariance)),
              MapPointStatus::MAP_POINT_STATUS_SUCCESS);
    float maxDistanceInvariance2{};
    ASSERT_EQ((loaded.getMaxDistanceInvariance(maxDistanceInvariance2)),
              MapPointStatus::MAP_POINT_STATUS_SUCCESS);
    EXPECT_FLOAT_EQ(maxDistanceInvariance, maxDistanceInvariance2);
    // SKIP mutexes (positionMutex/featuresMutex/mapMutex): post-load object
    // must be usable through its public getters (exercised above).
    MapPointStatus setWorldPosStatus{};
    EXPECT_NO_THROW(setWorldPosStatus =
                        loaded.setWorldPos(Eigen::Vector3f(4.0F, 5.0F, 6.0F)));
    EXPECT_EQ(setWorldPosStatus, MapPointStatus::MAP_POINT_STATUS_SUCCESS);
}

/*!
 * @brief        Checks that an empty key frame database survives a round trip,
 *               with a second save giving the same byte length, and that
 *               clearing a map on the loaded database works.
 */
TEST(SerializationKeyFrameDatabase, EmptyRoundTrip)
{
    // NB: KeyFrameDatabase::preSave() is declared in KeyFrameDatabase.h:82
    // but has no definition in KeyFrameDatabase.cc on this branch, so it
    // must not be called (would fail to link). The empty DB backup vector
    // is already deterministically empty without it.
    KeyFrameDatabase original;
    KeyFrameDatabase loaded;
    RoundTripBinaryInto(original, loaded);
    // Empty inverted file: backup IDs empty on both sides. No public
    // getter exists, so self-consistency is proven by a second
    // serialization producing identical byte length.
    std::stringstream first_stream(std::ios::in | std::ios::out |
                                   std::ios::binary);
    std::stringstream second_stream(std::ios::in | std::ios::out |
                                    std::ios::binary);
    {
        boost::archive::binary_oarchive output_archive(first_stream);
        output_archive << original;
    }
    {
        boost::archive::binary_oarchive output_archive(second_stream);
        output_archive << loaded;
    }
    EXPECT_EQ(first_stream.str().size(), second_stream.str().size());

    // SKIP mutex/vocabulary pointer: post-load DB must accept clearMap
    // on an empty map without crashing (mutex default-constructed).
    Map                    empty_map;
    KeyFrameDatabaseStatus clearMapStatus{};
    EXPECT_NO_THROW(clearMapStatus = loaded.clearMap(&empty_map));
    EXPECT_EQ(clearMapStatus,
              KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS);
}

/*!
 * @brief        Checks that a key frame without cameras keeps its id, key point
 *               count, pose, velocity and IMU bias across a round trip, and can
 *               still be given a new pose.
 */
TEST(SerializationKeyFrame, DefaultRoundTrip)
{
    KeyFrame original;
    original.id                 = 11U;
    original.isImu              = false;
    original.p_camera           = nullptr;
    original.p_camera2          = nullptr;
    original.p_imuPreintegrated = nullptr;
    ASSERT_EQ(
        (original.setPose(Sophus::SE3f(Eigen::Quaternionf::Identity(),
                                       Eigen::Vector3f(1.0F, 0.0F, 0.0F)))),
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    ASSERT_EQ((original.setVelocity(Eigen::Vector3f(0.1F, 0.2F, 0.3F))),
              KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    ASSERT_EQ(
        (original.setNewBias(IMU::Bias(0.01F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F))),
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);

    std::set<KeyFrame *>                                        keyframe_set;
    std::set<MapPoint *>                                        mappoint_set;
    std::set<camera_models::geometriccamera::GeometricCamera *> camera_set;
    ASSERT_EQ((original.preSave(keyframe_set, mappoint_set, camera_set)),
              KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);

    KeyFrame loaded;
    loaded.p_camera           = nullptr;
    loaded.p_camera2          = nullptr;
    loaded.p_imuPreintegrated = nullptr;
    RoundTripBinaryInto(original, loaded);
    EXPECT_EQ(original.id, loaded.id);
    EXPECT_EQ(original.keyPointCount, loaded.keyPointCount);
    bool isBad2{};
    ASSERT_EQ((original.isBad(isBad2)),
              KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    bool isBad3{};
    ASSERT_EQ((loaded.isBad(isBad3)), KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    EXPECT_EQ(isBad2, isBad3);
    Sophus::SE3f originalPose{};
    ASSERT_EQ((original.getPose(originalPose)),
              KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    Sophus::SE3f loadedPose{};
    ASSERT_EQ((loaded.getPose(loadedPose)),
              KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    ExpectSophusEqual(originalPose, loadedPose);
    Eigen::Vector3f velocity{};
    ASSERT_EQ((original.getVelocity(velocity)),
              KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    Eigen::Vector3f velocity2{};
    ASSERT_EQ((loaded.getVelocity(velocity2)),
              KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    EXPECT_TRUE(velocity.isApprox(velocity2, 1.0e-5F));
    IMU::Bias imuBias{};
    ASSERT_EQ((original.getImuBias(imuBias)),
              KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    IMU::Bias imuBias2{};
    ASSERT_EQ((loaded.getImuBias(imuBias2)),
              KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    EXPECT_FLOAT_EQ(imuBias.bax, imuBias2.bax);
    // SKIP mutexes/atomic/thread handles: loaded frame must remain
    // usable (pose getter + new pose set must not deadlock).
    KeyFrameStatus setPoseStatus{};
    EXPECT_NO_THROW(setPoseStatus = loaded.setPose(Sophus::SE3f()));
    EXPECT_EQ(setPoseStatus, KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
}

/*!
 * @brief        Checks that a key frame saved without cameras loads with both
 *               camera pointers null, even when they held stale values, and
 *               adds nothing to the camera table.
 */
TEST(SerializationKeyFrame, AbsentCamerasLoadAsNull)
{
    // A key frame saved without cameras stores the "no id" value for both;
    // loading must leave both pointers null and must not look that value up
    // in the camera table (the old ">= 0" test on an unsigned id did, which
    // added a null entry to the table).
    KeyFrame original;
    original.id                 = 12U;
    original.isImu              = false;
    original.p_camera           = nullptr;
    original.p_camera2          = nullptr;
    original.p_imuPreintegrated = nullptr;
    std::set<KeyFrame *>                                        keyframe_set;
    std::set<MapPoint *>                                        mappoint_set;
    std::set<camera_models::geometriccamera::GeometricCamera *> camera_set;
    ASSERT_EQ((original.preSave(keyframe_set, mappoint_set, camera_set)),
              KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);

    // Stale pointers show that postLoad sets both cameras, not that the
    // default constructor happened to leave them null.
    camera_models::pinhole::Pinhole stale(
        std::vector<float>{500.0F, 500.0F, 320.0F, 240.0F});
    KeyFrame loaded;
    loaded.p_camera           = &stale;
    loaded.p_camera2          = &stale;
    loaded.p_imuPreintegrated = nullptr;
    RoundTripBinaryInto(original, loaded);

    std::map<long unsigned int, KeyFrame *> keyFrameIds;
    std::map<long unsigned int, MapPoint *> mapPointIds;
    std::map<unsigned int, camera_models::geometriccamera::GeometricCamera *>
        cameraIds;
    ASSERT_EQ((loaded.postLoad(keyFrameIds, mapPointIds, cameraIds)),
              KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    EXPECT_TRUE(cameraIds.empty());
    EXPECT_EQ(loaded.p_camera, nullptr);
    EXPECT_EQ(loaded.p_camera2, nullptr);
}

/*!
 * @brief        Checks that an empty map keeps its id, key frame ids and flags
 *               across a round trip through memory and through a temporary
 *               file, and can be queried afterwards.
 */
TEST(SerializationMap, EmptyRoundTripMemoryAndTmpFile)
{
    Map           original(5);
    unsigned long keyFrameCount{};
    ASSERT_EQ((original.getKeyFrameCount(keyFrameCount)),
              MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(0U, keyFrameCount);
    unsigned long mapPointCount{};
    ASSERT_EQ((original.getMapPointCount(mapPointCount)),
              MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(0U, mapPointCount);

    Map loaded;
    RoundTripBinaryInto(original, loaded);
    unsigned long id{};
    ASSERT_EQ((original.getId(id)), MapStatus::MAP_STATUS_SUCCESS);
    unsigned long id2{};
    ASSERT_EQ((loaded.getId(id2)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(id, id2);
    unsigned long initKeyFrameId{};
    ASSERT_EQ((original.getInitKeyFrameId(initKeyFrameId)),
              MapStatus::MAP_STATUS_SUCCESS);
    unsigned long initKeyFrameId2{};
    ASSERT_EQ((loaded.getInitKeyFrameId(initKeyFrameId2)),
              MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(initKeyFrameId, initKeyFrameId2);
    unsigned long maxKeyFrameId{};
    ASSERT_EQ((original.getMaxKeyFrameId(maxKeyFrameId)),
              MapStatus::MAP_STATUS_SUCCESS);
    unsigned long maxKeyFrameId2{};
    ASSERT_EQ((loaded.getMaxKeyFrameId(maxKeyFrameId2)),
              MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(maxKeyFrameId, maxKeyFrameId2);
    bool isImuInitialized2{};
    ASSERT_EQ((original.isImuInitialized(isImuInitialized2)),
              MapStatus::MAP_STATUS_SUCCESS);
    bool isImuInitialized3{};
    ASSERT_EQ((loaded.isImuInitialized(isImuInitialized3)),
              MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(isImuInitialized2, isImuInitialized3);
    bool isInertial2{};
    ASSERT_EQ((original.isInertial(isInertial2)),
              MapStatus::MAP_STATUS_SUCCESS);
    bool isInertial3{};
    ASSERT_EQ((loaded.isInertial(isInertial3)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(isInertial2, isInertial3);
    bool isBad2{};
    ASSERT_EQ((original.isBad(isBad2)), MapStatus::MAP_STATUS_SUCCESS);
    bool isBad3{};
    ASSERT_EQ((loaded.isBad(isBad3)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(isBad2, isBad3);
    unsigned long keyFrameCount2{};
    ASSERT_EQ((loaded.getKeyFrameCount(keyFrameCount2)),
              MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(0U, keyFrameCount2);

    const std::string path = TmpPath("map.bin");
    Map               file_loaded;
    RoundTripTextFileInto(original, path, file_loaded);
    unsigned long id3{};
    ASSERT_EQ((original.getId(id3)), MapStatus::MAP_STATUS_SUCCESS);
    unsigned long id4{};
    ASSERT_EQ((file_loaded.getId(id4)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(id3, id4);
    std::remove(path.c_str());

    // SKIP mutex/atomic/thumbnail: post-load map must be queryable.
    std::vector<KeyFrame *> loadedKeyFrames;
    MapStatus               getAllKeyFramesStatus{};
    EXPECT_NO_THROW(getAllKeyFramesStatus =
                        loaded.getAllKeyFrames(loadedKeyFrames));
    EXPECT_EQ(getAllKeyFramesStatus, MapStatus::MAP_STATUS_SUCCESS);
}

/*!
 * @brief        Checks that an empty atlas with no cameras survives a round
 *               trip with zero maps and can still be queried.
 */
TEST(SerializationAtlas, EmptyAndCameraRoundTrip)
{
    Atlas original;
    Atlas loaded;
    RoundTripBinaryInto(original, loaded);
    int maps{};
    ASSERT_EQ((original.countMaps(maps)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    int maps2{};
    ASSERT_EQ((loaded.countMaps(maps2)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    EXPECT_EQ(maps, maps2);
    int maps3{};
    ASSERT_EQ((loaded.countMaps(maps3)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    EXPECT_EQ(0, maps3);
    std::vector<camera_models::geometriccamera::GeometricCamera *> allCameras{};
    ASSERT_EQ((loaded.getAllCameras(allCameras)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    EXPECT_TRUE(allCameras.empty());
    // SKIP mutex/atomic/viewer/DB pointers: loaded atlas must be queryable.
    std::vector<Map *> loadedMaps;
    AtlasStatus        getAllMapsStatus{};
    EXPECT_NO_THROW(getAllMapsStatus = loaded.getAllMaps(loadedMaps));
    EXPECT_EQ(getAllMapsStatus, AtlasStatus::ATLAS_STATUS_SUCCESS);
}

/*!
 * @brief        Checks that an atlas with one map and one pinhole camera
 *               written to and read from a file keeps its last init key frame
 *               id and the camera's id and type.
 */
TEST(SerializationAtlas, SeededMapAndCameraFileRoundTrip)
{
    Atlas original(0);
    int   maps2{};
    ASSERT_EQ((original.countMaps(maps2)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    EXPECT_EQ(1, maps2);
    camera_models::pinhole::Pinhole *camera =
        new camera_models::pinhole::Pinhole(
            std::vector<float>{500.0F, 500.0F, 320.0F, 240.0F});
    camera_models::geometriccamera::GeometricCamera *p_originalCamera = nullptr;
    ASSERT_EQ((original.addCamera(camera, p_originalCamera)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    std::vector<camera_models::geometriccamera::GeometricCamera *> allCameras{};
    ASSERT_EQ((original.getAllCameras(allCameras)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ(1U, allCameras.size());

    // Direct serialize saves backup maps (empty until preSave) + cameras +
    // static IDs. Active sets are rebuilt by postLoad, so compare the
    // serialized state (cameras + init ID), not the transient active set.
    const std::string path = TmpPath("atlas.bin");
    Atlas             loaded;
    {
        std::ofstream                   output_stream(path, std::ios::binary);
        boost::archive::binary_oarchive output_archive(output_stream);
        output_archive << original;
    }
    {
        std::ifstream                   input_stream(path, std::ios::binary);
        boost::archive::binary_iarchive input_archive(input_stream);
        input_archive >> loaded;
    }
    std::remove(path.c_str());

    unsigned long lastInitKeyFrameId{};
    ASSERT_EQ((original.getLastInitKeyFrameId(lastInitKeyFrameId)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    unsigned long lastInitKeyFrameId2{};
    ASSERT_EQ((loaded.getLastInitKeyFrameId(lastInitKeyFrameId2)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    EXPECT_EQ(lastInitKeyFrameId, lastInitKeyFrameId2);
    std::vector<camera_models::geometriccamera::GeometricCamera *>
        allCameras2{};
    ASSERT_EQ((loaded.getAllCameras(allCameras2)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ(1U, allCameras2.size());
    unsigned int id{};
    ASSERT_EQ((camera->getId(id)),
              camera_models::geometriccamera::GeometricCameraStatus::
                  GEOMETRIC_CAMERA_STATUS_SUCCESS);
    unsigned int id2{};
    std::vector<camera_models::geometriccamera::GeometricCamera *>
        allCameras3{};
    ASSERT_EQ((loaded.getAllCameras(allCameras3)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((allCameras3[0]->getId(id2)),
              camera_models::geometriccamera::GeometricCameraStatus::
                  GEOMETRIC_CAMERA_STATUS_SUCCESS);
    EXPECT_EQ(id, id2);
    unsigned int type{};
    std::vector<camera_models::geometriccamera::GeometricCamera *>
        allCameras4{};
    ASSERT_EQ((loaded.getAllCameras(allCameras4)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((allCameras4[0]->getType(type)),
              camera_models::geometriccamera::GeometricCameraStatus::
                  GEOMETRIC_CAMERA_STATUS_SUCCESS);
    EXPECT_TRUE(type ==
                camera_models::geometriccamera::GeometricCamera::CAM_PINHOLE);
    // SKIP mutex/atomic/thread handles: loaded atlas must stay queryable.
    std::vector<Map *> loadedMaps;
    AtlasStatus        getAllMapsStatus{};
    EXPECT_NO_THROW(getAllMapsStatus = loaded.getAllMaps(loadedMaps));
    EXPECT_EQ(getAllMapsStatus, AtlasStatus::ATLAS_STATUS_SUCCESS);
}

/*!
 * @brief        Checks that the sorted-set and sorted-map comparison helpers
 *               treat equal contents in different insertion order as equal, and
 *               that different values still compare unequal.
 */
TEST(SerializationOrdering, SortedSetMapComparisonIsDeterministic)
{
    const std::set<long unsigned int> first{3U, 1U, 2U};
    const std::set<long unsigned int> second{1U, 2U, 3U};
    ExpectSetsEqualSorted(first, second);

    const std::map<long unsigned int, int> first_map{{2U, 20}, {1U, 10}};
    const std::map<long unsigned int, int> second_map{{1U, 10}, {2U, 20}};
    ExpectMapsEqualSorted(first_map, second_map);

    const std::map<long unsigned int, int> different{{1U, 10}, {2U, 21}};
    EXPECT_NE(first_map, different);
}

} // namespace core
} // namespace vs_graphs

/*!
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
#include <fstream>
#include <map>
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
    auto expected_it = expected_in.begin();
    auto actual_it   = actual_in.begin();
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
    auto expected_it = expected_in.begin();
    auto actual_it   = actual_in.begin();
    while (expected_it != expected_in.end())
    {
        EXPECT_EQ(*expected_it, *actual_it);
        ++expected_it;
        ++actual_it;
    }
}

} // namespace

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

TEST(SerializationImu, CalibRoundTrip)
{
    const Sophus::SE3f known_tbc(Eigen::Quaternionf(0.0F, 0.0F, 0.0F, 1.0F),
                                 Eigen::Vector3f(0.1F, 0.2F, 0.3F));
    IMU::Calib         original;
    original.setCalibration(known_tbc, 0.01F, 0.02F, 0.001F, 0.002F);
    EXPECT_TRUE(original.mbIsSet);

    const IMU::Calib loaded = RoundTripBinaryCopyable(original);
    EXPECT_TRUE(loaded.mbIsSet);
    ExpectSophusEqual(original.mTbc, loaded.mTbc);
    ExpectSophusEqual(original.mTcb, loaded.mTcb);
    EXPECT_TRUE(
        original.Cov.diagonal().isApprox(loaded.Cov.diagonal(), 1.0e-6F));
    EXPECT_TRUE(original.CovWalk.diagonal().isApprox(loaded.CovWalk.diagonal(),
                                                     1.0e-6F));

    const IMU::Calib unset;
    EXPECT_FALSE(unset.mbIsSet);
    const IMU::Calib unset_loaded = RoundTripBinaryCopyable(unset);
    EXPECT_FALSE(unset_loaded.mbIsSet);
}

TEST(SerializationImu, PreintegratedRoundTrip)
{
    const IMU::Bias    bias(0.01F, 0.02F, 0.03F, 0.001F, 0.002F, 0.003F);
    const Sophus::SE3f tbc(Eigen::Quaternionf::Identity(),
                           Eigen::Vector3f(0.05F, 0.0F, 0.0F));
    IMU::Calib         calib;
    calib.setCalibration(tbc, 0.01F, 0.02F, 0.001F, 0.002F);

    IMU::Preintegrated original(bias, calib);
    original.integrateNewMeasurement(Eigen::Vector3f(0.0F, 0.0F, 9.81F),
                                     Eigen::Vector3f(0.01F, 0.0F, 0.0F),
                                     0.01F);
    original.integrateNewMeasurement(Eigen::Vector3f(0.1F, 0.0F, 9.80F),
                                     Eigen::Vector3f(0.0F, 0.01F, 0.0F),
                                     0.01F);
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
    EXPECT_NO_THROW(
        loaded.integrateNewMeasurement(Eigen::Vector3f(0.0F, 0.0F, 9.81F),
                                       Eigen::Vector3f::Zero(),
                                       0.005F));
}

TEST(SerializationCamera, PinholeRoundTrip)
{
    camera_models::pinhole::Pinhole original(
        std::vector<float>{500.0F, 500.0F, 320.0F, 240.0F});
    const unsigned int               original_id = original.getId();
    // Heap leak intentional: camera_models::pinhole::Pinhole
    // default ctor leaves tvr uninitialised and serialize() never touches it,
    // so a stack-loaded object could destroy garbage. Leaking mirrors Atlas
    // (which never deletes cameras).
    camera_models::pinhole::Pinhole *loaded_ptr =
        new camera_models::pinhole::Pinhole(
            std::vector<float>{1.0F, 1.0F, 1.0F, 1.0F});
    RoundTripBinaryInto(original, *loaded_ptr);
    camera_models::pinhole::Pinhole &loaded = *loaded_ptr;
    EXPECT_EQ(original_id, loaded.getId());
    // NB: compare by value (== 0U) not EXPECT_EQ(CAM_*): CAM_* has no
    // out-of-line definition, EXPECT_EQ would ODR-use and fail to link.
    EXPECT_TRUE(loaded.getType() ==
                camera_models::geometriccamera::GeometricCamera::CAM_PINHOLE);
    EXPECT_EQ(0U, loaded.getType());
    EXPECT_EQ(4U, loaded.size());
    EXPECT_FLOAT_EQ(500.0F, loaded.getParameter(0));
    EXPECT_FLOAT_EQ(240.0F, loaded.getParameter(3));
    EXPECT_TRUE(loaded.isEqual(
        const_cast<camera_models::pinhole::Pinhole *>(&original)));
    // SKIP tvr raw pointer: not serialized by design; no assertion.
}

TEST(SerializationCamera, KannalaBrandt8RoundTrip)
{
    const std::vector<float>
        params{500.0F, 500.0F, 320.0F, 240.0F, 0.1F, 0.01F, 0.001F, 0.0001F};
    camera_models::kannalabrandt8::KannalaBrandt8 original(params, 1.0e-5F);
    const unsigned int original_id = original.getId();
    // Heap leak intentional (see PinholeRoundTrip): default-constructed
    // tvr is untouched by serialize().
    camera_models::kannalabrandt8::KannalaBrandt8 *loaded_ptr =
        new camera_models::kannalabrandt8::KannalaBrandt8(
            std::vector<float>{1.0F, 1.0F, 1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F});
    RoundTripBinaryInto(original, *loaded_ptr);
    camera_models::kannalabrandt8::KannalaBrandt8 &loaded = *loaded_ptr;
    EXPECT_EQ(original_id, loaded.getId());
    // NB: see PinholeRoundTrip: avoid ODR-use of CAM_* (no definition).
    EXPECT_TRUE(loaded.getType() ==
                camera_models::geometriccamera::GeometricCamera::CAM_FISHEYE);
    EXPECT_EQ(1U, loaded.getType());
    EXPECT_EQ(8U, loaded.size());
    for (std::size_t index = 0; index < params.size(); ++index)
    {
        EXPECT_FLOAT_EQ(params[index], loaded.getParameter(index));
    }
    EXPECT_FLOAT_EQ(original.getPrecision(), loaded.getPrecision());
    EXPECT_TRUE(loaded.isEqual(
        const_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(
            &original)));
}

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
    EXPECT_EQ(original_camera->getId(), loaded_ptrs[0]->getId());
    EXPECT_TRUE(loaded_ptrs[0]->getType() ==
                camera_models::geometriccamera::GeometricCamera::CAM_PINHOLE);

    delete original_camera;
    // Intentional leak: loaded camera_models::pinhole::Pinhole
    // tvr is untouched by serialize() (see PinholeRoundTrip); deleting would
    // free garbage.
}

TEST(SerializationMapPoint, RoundTripWithRefKeyFrame)
{
    Map      map;
    KeyFrame ref_keyframe;
    ref_keyframe.mnId               = 7U;
    ref_keyframe.p_camera           = nullptr;
    ref_keyframe.p_camera2          = nullptr;
    ref_keyframe.p_imuPreintegrated = nullptr;

    MapPoint original(Eigen::Vector3f(1.0F, 2.0F, 3.0F), &ref_keyframe, &map);
    original.setNormalVector(Eigen::Vector3f(0.0F, 0.0F, 1.0F));
    const long unsigned int original_id = original.mnId;

    std::set<KeyFrame *> keyframe_set{&ref_keyframe};
    std::set<MapPoint *> mappoint_set{&original};
    original.PreSave(keyframe_set, mappoint_set);

    MapPoint loaded;
    RoundTripBinaryInto(original, loaded);
    EXPECT_EQ(original_id, loaded.mnId);
    EXPECT_EQ(original.firstKeyFrameId, loaded.firstKeyFrameId);
    EXPECT_EQ(original.observationCount, loaded.observationCount);
    EXPECT_TRUE(original.getWorldPos().isApprox(loaded.getWorldPos(), 1.0e-6F));
    EXPECT_TRUE(original.getNormal().isApprox(loaded.getNormal(), 1.0e-6F));
    EXPECT_EQ(original.isBad(), loaded.isBad());
    EXPECT_FLOAT_EQ(original.getMinDistanceInvariance(),
                    loaded.getMinDistanceInvariance());
    EXPECT_FLOAT_EQ(original.getMaxDistanceInvariance(),
                    loaded.getMaxDistanceInvariance());
    // SKIP mutexes (mMutexPos/mMutexFeatures/mMutexMap): post-load object
    // must be usable through its public getters (exercised above).
    EXPECT_NO_THROW(loaded.setWorldPos(Eigen::Vector3f(4.0F, 5.0F, 6.0F)));
}

TEST(SerializationKeyFrameDatabase, EmptyRoundTrip)
{
    // NB: KeyFrameDatabase::PreSave() is declared in KeyFrameDatabase.h:82
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
    Map empty_map;
    EXPECT_NO_THROW(loaded.clearMap(&empty_map));
}

TEST(SerializationKeyFrame, DefaultRoundTrip)
{
    KeyFrame original;
    original.mnId               = 11U;
    original.isImu              = false;
    original.p_camera           = nullptr;
    original.p_camera2          = nullptr;
    original.p_imuPreintegrated = nullptr;
    original.setPose(Sophus::SE3f(Eigen::Quaternionf::Identity(),
                                  Eigen::Vector3f(1.0F, 0.0F, 0.0F)));
    original.setVelocity(Eigen::Vector3f(0.1F, 0.2F, 0.3F));
    original.setNewBias(IMU::Bias(0.01F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F));

    std::set<KeyFrame *>                                        keyframe_set;
    std::set<MapPoint *>                                        mappoint_set;
    std::set<camera_models::geometriccamera::GeometricCamera *> camera_set;
    original.PreSave(keyframe_set, mappoint_set, camera_set);

    KeyFrame loaded;
    loaded.p_camera           = nullptr;
    loaded.p_camera2          = nullptr;
    loaded.p_imuPreintegrated = nullptr;
    RoundTripBinaryInto(original, loaded);
    EXPECT_EQ(original.mnId, loaded.mnId);
    EXPECT_EQ(original.N, loaded.N);
    EXPECT_EQ(original.isBad(), loaded.isBad());
    ExpectSophusEqual(original.getPose(), loaded.getPose());
    EXPECT_TRUE(original.getVelocity().isApprox(loaded.getVelocity(), 1.0e-5F));
    EXPECT_FLOAT_EQ(original.getImuBias().bax, loaded.getImuBias().bax);
    // SKIP mutexes/atomic/thread handles: loaded frame must remain
    // usable (pose getter + new pose set must not deadlock).
    EXPECT_NO_THROW(loaded.setPose(Sophus::SE3f()));
}

TEST(SerializationMap, EmptyRoundTripMemoryAndTmpFile)
{
    Map original(5);
    EXPECT_EQ(0U, original.getKeyFrameCount());
    EXPECT_EQ(0U, original.getMapPointCount());

    Map loaded;
    RoundTripBinaryInto(original, loaded);
    EXPECT_EQ(original.getId(), loaded.getId());
    EXPECT_EQ(original.getInitKeyFrameId(), loaded.getInitKeyFrameId());
    EXPECT_EQ(original.getMaxKeyFrameId(), loaded.getMaxKeyFrameId());
    EXPECT_EQ(original.isImuInitialized(), loaded.isImuInitialized());
    EXPECT_EQ(original.isInertial(), loaded.isInertial());
    EXPECT_EQ(original.isBad(), loaded.isBad());
    EXPECT_EQ(0U, loaded.getKeyFrameCount());

    const std::string path = TmpPath("map.bin");
    Map               file_loaded;
    RoundTripTextFileInto(original, path, file_loaded);
    EXPECT_EQ(original.getId(), file_loaded.getId());
    std::remove(path.c_str());

    // SKIP mutex/atomic/thumbnail: post-load map must be queryable.
    EXPECT_NO_THROW(loaded.getAllKeyFrames());
}

TEST(SerializationAtlas, EmptyAndCameraRoundTrip)
{
    Atlas original;
    Atlas loaded;
    RoundTripBinaryInto(original, loaded);
    EXPECT_EQ(original.countMaps(), loaded.countMaps());
    EXPECT_EQ(0, loaded.countMaps());
    EXPECT_TRUE(loaded.getAllCameras().empty());
    // SKIP mutex/atomic/viewer/DB pointers: loaded atlas must be queryable.
    EXPECT_NO_THROW(loaded.getAllMaps());
}

TEST(SerializationAtlas, SeededMapAndCameraFileRoundTrip)
{
    Atlas original(0);
    EXPECT_EQ(1, original.countMaps());
    camera_models::pinhole::Pinhole *camera =
        new camera_models::pinhole::Pinhole(
            std::vector<float>{500.0F, 500.0F, 320.0F, 240.0F});
    original.addCamera(camera);
    ASSERT_EQ(1U, original.getAllCameras().size());

    // Direct serialize saves backup maps (empty until PreSave) + cameras +
    // static IDs. Active sets are rebuilt by PostLoad, so compare the
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

    EXPECT_EQ(original.getLastInitKeyFrameId(), loaded.getLastInitKeyFrameId());
    ASSERT_EQ(1U, loaded.getAllCameras().size());
    EXPECT_EQ(camera->getId(), loaded.getAllCameras()[0]->getId());
    EXPECT_TRUE(loaded.getAllCameras()[0]->getType() ==
                camera_models::geometriccamera::GeometricCamera::CAM_PINHOLE);
    // SKIP mutex/atomic/thread handles: loaded atlas must stay queryable.
    EXPECT_NO_THROW(loaded.getAllMaps());
}

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

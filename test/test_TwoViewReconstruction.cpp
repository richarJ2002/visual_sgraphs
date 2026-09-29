/*!
 * @file test_TwoViewReconstruction.cpp
 * @brief Regression coverage for TwoViewReconstruction's homography
 *        reconstruction path: reconstructH() must publish the structure of
 *        its winning motion hypothesis through the vP3D output parameter,
 *        exactly as its sibling reconstructF() does, because
 *        Tracking::createInitialMapMonocular() reads those points straight
 *        after Reconstruct() returns.
 */

#include "TwoViewReconstruction.h"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <cstddef>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace
{

/*! Synthetic pinhole intrinsics shared by both views. */
constexpr float FOCAL_LENGTH_X_PIXELS    = 500.0F;
constexpr float FOCAL_LENGTH_Y_PIXELS    = 500.0F;
constexpr float PRINCIPAL_POINT_X_PIXELS = 320.0F;
constexpr float PRINCIPAL_POINT_Y_PIXELS = 240.0F;
constexpr float IMAGE_WIDTH_PIXELS       = 640.0F;
constexpr float IMAGE_HEIGHT_PIXELS      = 480.0F;

/*! Distance from the first camera centre to the observed plane. */
constexpr float PLANE_DISTANCE_M = 3.0F;

/*! Lateral offset of the second camera centre in the first camera frame. */
constexpr float CAMERA_BASELINE_M = 0.6F;

/*! Keypoint grid laid out over the first image. */
constexpr int   GRID_COLUMN_COUNT    = 16;
constexpr int   GRID_ROW_COUNT       = 16;
constexpr float GRID_ORIGIN_X_PIXELS = 100.0F;
constexpr float GRID_ORIGIN_Y_PIXELS = 70.0F;
constexpr float GRID_SPAN_X_PIXELS   = 440.0F;
constexpr float GRID_SPAN_Y_PIXELS   = 340.0F;

/*! Arguments reconstructH() receives from Reconstruct(). */
constexpr float MIN_PARALLAX_DEG       = 1.0F;
constexpr int   MIN_TRIANGULATED_COUNT = 50;

/*! checkRT() only keeps a point whose squared reprojection error stays below
 * 4 * sigma^2, so a reconstructed and flagged point is guaranteed to fall
 * within 2 pixels of its first-view keypoint. */
constexpr double REPROJECTION_TOLERANCE_PIXELS = 2.0;

/*!
 * @brief        One deterministic, noise-free two-view observation of a
 *               single tilted plane.
 */
struct PlanarTwoViewScene
{
    /*! Shared pinhole calibration of both views. */
    Eigen::Matrix3f calibrationMatrix;

    /*! Plane-induced pixel homography mapping view 1 into view 2. */
    Eigen::Matrix3f homography_view1ToView2;

    /*! Ground-truth rotation of the second view relative to the first. */
    Eigen::Matrix3f rotation_view1ToView2;

    /*! Ground-truth translation of the second view, in the second frame. */
    Eigen::Vector3f translation_view1ToView2_m;

    /*! Keypoints of the reference view. */
    std::vector<cv::KeyPoint> keypointsView1;

    /*! Keypoints of the current view. */
    std::vector<cv::KeyPoint> keypointsView2;

    /*! Index into keypointsView2 for every keypoint of view 1. */
    std::vector<int> matchesView1ToView2;
};

/*!
 * @brief        Builds a planar scene whose homography decomposition has a
 *               single dominant motion hypothesis.
 *
 * The plane is strongly tilted with respect to the optical axis; a
 * fronto-parallel plane leaves the Faugeras twisted-pair ambiguity almost
 * unresolved, so reconstructH() would reject its own winning hypothesis.
 *
 * @return       A fully populated planar two-view scene.
 */
PlanarTwoViewScene buildPlanarTwoViewScene()
{
    PlanarTwoViewScene scene;

    scene.calibrationMatrix << FOCAL_LENGTH_X_PIXELS, 0.0F,
        PRINCIPAL_POINT_X_PIXELS, 0.0F, FOCAL_LENGTH_Y_PIXELS,
        PRINCIPAL_POINT_Y_PIXELS, 0.0F, 0.0F, 1.0F;
    const Eigen::Matrix3f inverseCalibrationMatrix =
        scene.calibrationMatrix.inverse();

    /* Plane expressed in the first camera frame as planeNormal . X =
     * PLANE_DISTANCE_M, with planeNormal a unit vector. */
    Eigen::Vector3f planeNormal(0.75F, 0.50F, 1.0F);
    planeNormal.normalize();

    /* The second camera sits CAMERA_BASELINE_M to the right of the first and
     * verges back onto the scene centre so the plane stays in view. */
    const float vergenceAngle_rad =
        std::atan2(-CAMERA_BASELINE_M, PLANE_DISTANCE_M);
    Eigen::Matrix3f rotation_view2ToView1;
    rotation_view2ToView1 << std::cos(vergenceAngle_rad), 0.0F,
        std::sin(vergenceAngle_rad), 0.0F, 1.0F, 0.0F,
        -std::sin(vergenceAngle_rad), 0.0F, std::cos(vergenceAngle_rad);
    const Eigen::Vector3f cameraCentreView2_m(CAMERA_BASELINE_M, 0.0F, 0.0F);

    scene.rotation_view1ToView2 = rotation_view2ToView1.transpose();
    scene.translation_view1ToView2_m =
        -scene.rotation_view1ToView2 * cameraCentreView2_m;

    scene.homography_view1ToView2 =
        scene.calibrationMatrix *
        (scene.rotation_view1ToView2 + scene.translation_view1ToView2_m *
                                           planeNormal.transpose() /
                                           PLANE_DISTANCE_M) *
        inverseCalibrationMatrix;

    /* Back-project a regular pixel grid of view 1 onto the plane, then
     * project each plane point into view 2. Starting from view-1 pixels
     * keeps every correspondence inside the first image by construction. */
    for (int gridRow = 0; gridRow < GRID_ROW_COUNT; gridRow++)
    {
        for (int gridColumn = 0; gridColumn < GRID_COLUMN_COUNT; gridColumn++)
        {
            const float pixelX = GRID_ORIGIN_X_PIXELS +
                                 static_cast<float>(gridColumn) *
                                     GRID_SPAN_X_PIXELS /
                                     static_cast<float>(GRID_COLUMN_COUNT - 1);
            const float pixelY = GRID_ORIGIN_Y_PIXELS +
                                 static_cast<float>(gridRow) *
                                     GRID_SPAN_Y_PIXELS /
                                     static_cast<float>(GRID_ROW_COUNT - 1);

            const Eigen::Vector3f viewingRay =
                inverseCalibrationMatrix *
                Eigen::Vector3f(pixelX, pixelY, 1.0F);
            const float rayPlaneProjection = planeNormal.dot(viewingRay);
            if (rayPlaneProjection <= 0.0F)
            {
                continue;
            }

            const Eigen::Vector3f pointView1_m =
                viewingRay * (PLANE_DISTANCE_M / rayPlaneProjection);
            const Eigen::Vector3f pointView2_m =
                scene.rotation_view1ToView2 * pointView1_m +
                scene.translation_view1ToView2_m;
            if (pointView2_m(2) <= 0.0F)
            {
                continue;
            }

            const float projectedX =
                FOCAL_LENGTH_X_PIXELS * pointView2_m(0) / pointView2_m(2) +
                PRINCIPAL_POINT_X_PIXELS;
            const float projectedY =
                FOCAL_LENGTH_Y_PIXELS * pointView2_m(1) / pointView2_m(2) +
                PRINCIPAL_POINT_Y_PIXELS;
            if (projectedX < 0.0F || projectedX > IMAGE_WIDTH_PIXELS ||
                projectedY < 0.0F || projectedY > IMAGE_HEIGHT_PIXELS)
            {
                continue;
            }

            scene.matchesView1ToView2.push_back(
                static_cast<int>(scene.keypointsView2.size()));
            scene.keypointsView1.push_back(cv::KeyPoint(pixelX, pixelY, 1.0F));
            scene.keypointsView2.push_back(
                cv::KeyPoint(projectedX, projectedY, 1.0F));
        }
    }

    return scene;
}

/*! Signature of the private TwoViewReconstruction::reconstructH(). */
using ReconstructHPointer = TwoViewReconstructionStatus (
    TwoViewReconstruction::*)(std::vector<bool> &,
                              Eigen::Matrix3f &,
                              Eigen::Matrix3f &,
                              Sophus::SE3f &,
                              std::vector<cv::Point3f> &,
                              std::vector<bool> &,
                              float,
                              int,
                              bool &);

/*!
 * @brief        Carries the recovered member pointer out of the explicit
 *               instantiation below.
 */
struct ReconstructHAccess
{
    using type = ReconstructHPointer;

    friend type getReconstructH(ReconstructHAccess);
};

/*!
 * @brief        Publishes a private member pointer through a friend function.
 *
 * Access checking is not applied to the names used in an explicit
 * instantiation ([temp.explicit]), so this is a standard-conforming way to
 * reach a private method without touching the production header. It is used
 * here because Reconstruct() cannot be steered onto the homography branch
 * from a test: that branch needs SH / (SH + SF) > 0.50, and for an exactly
 * planar scene the fundamental matrix fits every correspondence at least as
 * well as the homography does, which pins the ratio at 0.50.
 */
template <typename TagType, typename TagType::type MemberPointer>
struct PrivateMethodPublisher
{
    friend typename TagType::type getReconstructH(TagType)
    {
        return MemberPointer;
    }
};

template struct PrivateMethodPublisher<ReconstructHAccess,
                                       &TwoViewReconstruction::reconstructH>;

/*!
 * @brief        Loads the scene into the private keypoint and match state
 *               that reconstructH() reads.
 *
 * Reconstruct() is the only public entry point that fills that state. Its
 * own verdict is irrelevant here and is deliberately ignored.
 *
 * @param[in]    scene_in
 *               Scene whose correspondences are loaded.
 *
 * @param[in,out] reconstruction_inout
 *               Reconstruction object receiving the correspondences.
 */
void loadSceneCorrespondences(const PlanarTwoViewScene &scene_in,
                              TwoViewReconstruction    &reconstruction_inout)
{
    Sophus::SE3f             unusedPose;
    std::vector<cv::Point3f> unusedPoints;
    std::vector<bool>        unusedFlags;

    bool reconstructionIsReconstructed{};
    ASSERT_EQ(
        (reconstruction_inout.reconstruct(scene_in.keypointsView1,
                                          scene_in.keypointsView2,
                                          scene_in.matchesView1ToView2,
                                          unusedPose,
                                          unusedPoints,
                                          unusedFlags,
                                          reconstructionIsReconstructed)),
        TwoViewReconstructionStatus::TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS);
}

} // namespace

TEST(TwoViewReconstruction, HomographyBranchPublishesTriangulatedStructure)
{
    /* Before the fix reconstructH() computed bestP3D and then dropped it, so
     * the caller-owned vP3D stayed exactly as it was handed in. */
    const PlanarTwoViewScene scene = buildPlanarTwoViewScene();
    ASSERT_GT(scene.keypointsView1.size(), 150U);

    TwoViewReconstruction reconstruction(scene.calibrationMatrix, 1.0F, 200);
    loadSceneCorrespondences(scene, reconstruction);

    std::vector<bool> matchInliers(scene.matchesView1ToView2.size(), true);
    Eigen::Matrix3f   homography        = scene.homography_view1ToView2;
    Eigen::Matrix3f   calibrationMatrix = scene.calibrationMatrix;
    Sophus::SE3f      pose_view1ToView2;
    std::vector<cv::Point3f> pointsView1_m;
    std::vector<bool>        triangulatedFlags;

    bool isReconstructed{};
    ASSERT_EQ(
        ((reconstruction.*
          getReconstructH(ReconstructHAccess{}))(matchInliers,
                                                 homography,
                                                 calibrationMatrix,
                                                 pose_view1ToView2,
                                                 pointsView1_m,
                                                 triangulatedFlags,
                                                 MIN_PARALLAX_DEG,
                                                 MIN_TRIANGULATED_COUNT,
                                                 isReconstructed)),
        TwoViewReconstructionStatus::TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS);

    ASSERT_TRUE(isReconstructed);
    /* Both assertions are fatal: the per-point checks below index vP3D by
     * keypoint, so a short or empty vector would read out of bounds. */
    ASSERT_EQ(pointsView1_m.size(), scene.keypointsView1.size());
    ASSERT_EQ(triangulatedFlags.size(), scene.keypointsView1.size());

    std::size_t triangulatedCount = 0U;
    for (std::size_t keypointIndex = 0U;
         keypointIndex < triangulatedFlags.size();
         keypointIndex++)
    {
        if (!triangulatedFlags[keypointIndex])
        {
            continue;
        }

        triangulatedCount++;

        const cv::Point3f &point_m = pointsView1_m[keypointIndex];
        ASSERT_TRUE(std::isfinite(point_m.x)) << "index " << keypointIndex;
        ASSERT_TRUE(std::isfinite(point_m.y)) << "index " << keypointIndex;
        ASSERT_TRUE(std::isfinite(point_m.z)) << "index " << keypointIndex;
        EXPECT_GT(point_m.z, 0.0F) << "index " << keypointIndex;

        /* Monocular structure is recovered up to scale, so only the
         * scale-invariant reprojection into view 1 can be checked. */
        const double reprojectedX =
            static_cast<double>(FOCAL_LENGTH_X_PIXELS) *
                static_cast<double>(point_m.x) /
                static_cast<double>(point_m.z) +
            static_cast<double>(PRINCIPAL_POINT_X_PIXELS);
        const double reprojectedY =
            static_cast<double>(FOCAL_LENGTH_Y_PIXELS) *
                static_cast<double>(point_m.y) /
                static_cast<double>(point_m.z) +
            static_cast<double>(PRINCIPAL_POINT_Y_PIXELS);
        const double offsetX =
            reprojectedX -
            static_cast<double>(scene.keypointsView1[keypointIndex].pt.x);
        const double offsetY =
            reprojectedY -
            static_cast<double>(scene.keypointsView1[keypointIndex].pt.y);

        EXPECT_LT(std::sqrt(offsetX * offsetX + offsetY * offsetY),
                  REPROJECTION_TOLERANCE_PIXELS)
            << "index " << keypointIndex;
    }

    EXPECT_GT(triangulatedCount,
              static_cast<std::size_t>(MIN_TRIANGULATED_COUNT));
}

TEST(TwoViewReconstruction, HomographyBranchRejectsAnEmptyInlierSet)
{
    /* With no inlier left, every motion hypothesis triangulates nothing, so
     * the branch reports failure and publishes no structure. */
    const PlanarTwoViewScene scene = buildPlanarTwoViewScene();

    TwoViewReconstruction reconstruction(scene.calibrationMatrix, 1.0F, 200);
    loadSceneCorrespondences(scene, reconstruction);

    std::vector<bool> matchInliers(scene.matchesView1ToView2.size(), false);
    Eigen::Matrix3f   homography        = scene.homography_view1ToView2;
    Eigen::Matrix3f   calibrationMatrix = scene.calibrationMatrix;
    Sophus::SE3f      pose_view1ToView2;
    std::vector<cv::Point3f> pointsView1_m;
    std::vector<bool>        triangulatedFlags;

    bool isReconstructed{};
    ASSERT_EQ(
        ((reconstruction.*
          getReconstructH(ReconstructHAccess{}))(matchInliers,
                                                 homography,
                                                 calibrationMatrix,
                                                 pose_view1ToView2,
                                                 pointsView1_m,
                                                 triangulatedFlags,
                                                 MIN_PARALLAX_DEG,
                                                 MIN_TRIANGULATED_COUNT,
                                                 isReconstructed)),
        TwoViewReconstructionStatus::TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS);

    EXPECT_FALSE(isReconstructed);
    EXPECT_TRUE(pointsView1_m.empty());
}

} // namespace core
} // namespace vs_graphs

/**
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

#include "SemanticsManager.h"
#include "Semantic/SemanticAxiomEvaluator.h"
#include "Semantic/SemanticAxiomEvaluator/EnumNames.h"
#include "Semantic/SemanticCandidates.h"
#include "Semantic/SemanticCanonicalSerialization.h"
#include "Semantic/SemanticGraphSnapshot.h"
#include "Semantic/SemanticVerify.h"
#include "Semantic/Sha256Digest.h"
#include "Semantic/ValueOrder.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#ifdef VS_GRAPHS_ENABLE_ROOM_TRACKER_TEST_HOOK
#include <functional>
#endif
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace ORB_SLAM3
{
namespace
{
/*!
 * @brief       Tests whether an observed finite wall separates two positions.
 *
 *              The infinite plane performs the side test, while the mapped
 *              cloud bounds reject unrelated coplanar wall segments. This is
 *              used as a veto when connected free-space evidence suggests two
 *              room hypotheses may describe the same physical room.
 *
 * @param[in]   wallList_World_in
 *              Candidate wall surfaces expressed in the active map frame.
 * @param[in]   firstPoint_World_m_in
 *              First position expressed in the active map frame, in metres.
 * @param[in]   secondPoint_World_m_in
 *              Second position expressed in the active map frame, in metres.
 * @param[in]   finiteBoundsMargin_m_in
 *              Margin applied around the observed wall-cloud bounds.
 *
 * @return      True when the segment crosses an observed finite wall patch.
 */
bool hasSeparatingFiniteWall(const std::vector<Plane *> &wallList_World_in,
                             const Eigen::Vector3d      &firstPoint_World_m_in,
                             const Eigen::Vector3d      &secondPoint_World_m_in,
                             const double finiteBoundsMargin_m_in)
{
    constexpr double minimumSideDistance_m = 0.10;

    for (Plane *p_wall : wallList_World_in)
    {
        if (p_wall == nullptr || p_wall->isBad() ||
            p_wall->getPlaneType() != Plane::planeVariant::WALL)
        {
            continue;
        }

        const Plane::GeometrySnapshot wallGeometry =
            p_wall->getGeometrySnapshot();
        Eigen::Vector4d wallEquation_World = wallGeometry.equation_World;
        const double    wallNormalNorm = wallEquation_World.head<3>().norm();

        if (!wallEquation_World.allFinite() || wallNormalNorm < 1e-8)
        {
            continue;
        }

        wallEquation_World /= wallNormalNorm;

        const double firstSide_m =
            wallEquation_World.head<3>().dot(firstPoint_World_m_in) +
            wallEquation_World(3);
        const double secondSide_m =
            wallEquation_World.head<3>().dot(secondPoint_World_m_in) +
            wallEquation_World(3);

        if (firstSide_m * secondSide_m >= 0.0 ||
            std::abs(firstSide_m) < minimumSideDistance_m ||
            std::abs(secondSide_m) < minimumSideDistance_m)
        {
            continue;
        }

        const double interpolation = firstSide_m / (firstSide_m - secondSide_m);
        const Eigen::Vector3d intersection_World_m =
            firstPoint_World_m_in +
            interpolation * (secondPoint_World_m_in - firstPoint_World_m_in);

        const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_wallCloud =
            wallGeometry.supportCloud;

        if (p_wallCloud == nullptr || p_wallCloud->empty())
        {
            continue;
        }

        const Eigen::Vector3d wallCentroid_World_m =
            wallGeometry.centroid_World_m;
        const Eigen::Vector3d wallAxisU_World =
            wallEquation_World.head<3>().unitOrthogonal().normalized();
        const Eigen::Vector3d wallAxisV_World =
            wallEquation_World.head<3>().cross(wallAxisU_World).normalized();

        double minimumWallU_m = std::numeric_limits<double>::infinity();
        double maximumWallU_m = -std::numeric_limits<double>::infinity();
        double minimumWallV_m = std::numeric_limits<double>::infinity();
        double maximumWallV_m = -std::numeric_limits<double>::infinity();

        for (const pcl::PointXYZRGBA &wallPoint : p_wallCloud->points)
        {
            if (!pcl::isFinite(wallPoint))
            {
                continue;
            }

            const Eigen::Vector3d wallPoint_World_m(
                static_cast<double>(wallPoint.x),
                static_cast<double>(wallPoint.y),
                static_cast<double>(wallPoint.z));
            const Eigen::Vector3d wallOffset_World_m =
                wallPoint_World_m - wallCentroid_World_m;

            const double wallU_m = wallOffset_World_m.dot(wallAxisU_World);
            const double wallV_m = wallOffset_World_m.dot(wallAxisV_World);

            minimumWallU_m = std::min(minimumWallU_m, wallU_m);
            maximumWallU_m = std::max(maximumWallU_m, wallU_m);
            minimumWallV_m = std::min(minimumWallV_m, wallV_m);
            maximumWallV_m = std::max(maximumWallV_m, wallV_m);
        }

        if (!std::isfinite(minimumWallU_m) || !std::isfinite(maximumWallU_m) ||
            !std::isfinite(minimumWallV_m) || !std::isfinite(maximumWallV_m))
        {
            continue;
        }

        const Eigen::Vector3d intersectionOffset_World_m =
            intersection_World_m - wallCentroid_World_m;
        const double intersectionU_m =
            intersectionOffset_World_m.dot(wallAxisU_World);
        const double intersectionV_m =
            intersectionOffset_World_m.dot(wallAxisV_World);

        if (intersectionU_m >= minimumWallU_m - finiteBoundsMargin_m_in &&
            intersectionU_m <= maximumWallU_m + finiteBoundsMargin_m_in &&
            intersectionV_m >= minimumWallV_m - finiteBoundsMargin_m_in &&
            intersectionV_m <= maximumWallV_m + finiteBoundsMargin_m_in)
        {
            return true;
        }
    }

    return false;
}

/*!
 * @brief Finite horizontal representation of one observed room wall.
 */
struct FiniteWallSegment2d
{
    /*!
     * @brief       TODO
     *
     * @frame       N/A
     * @unit        N/A
     */
    Plane *p_wall = nullptr;

    /*!
     * @brief       TODO
     *
     * @frame       TODO
     * @unit        meters
     */
    Eigen::Vector2d start_World_m = Eigen::Vector2d::Zero();

    /*!
     * @brief       TODO
     *
     * @frame       TODO
     * @unit        meters
     */
    Eigen::Vector2d end_World_m = Eigen::Vector2d::Zero();

    /*!
     * @brief       TODO
     *
     * @frame       N/A
     * @unit        meters
     */
    double length_m = 0.0;

    /*!
     * @brief       TODO
     *
     * @frame       N/A
     * @unit        N/A
     */
    double supportScore = 0.0;
};

struct WallAdmissionEvidence
{
    /*!
     * @brief       TODO
     *
     * @frame       N/A
     * @unit        N/A
     */
    bool admissible = false;

    /*!
     * @brief       TODO
     *
     * @frame       N/A
     * @unit        N/A
     */
    bool adequateFiniteFit = false;

    /*!
     * @brief       TODO
     *
     * @frame       N/A
     * @unit        Number of Points
     */
    std::size_t finitePointCount = 0U;

    /*!
     * @brief       TODO
     *
     * @frame       N/A
     * @unit        Number of Points
     */
    std::size_t fittedPointCount = 0U;
    /*!
     * @brief       TODO
     *
     * @frame       N/A
     * @unit        Number of Obeservations
     */
    std::size_t observationCount = 0U;
};

/*!
 * @brief           TODO
 *
 * @param[in]       p_wall_in
 *                  TODO
 *
 * @param[in]       p_systemParams_in
 *                  TODO
 *
 * @param[in]       groundNormal_World_in
 *                  TODO
 *
 * @return          TODO
 */
WallAdmissionEvidence
    evaluateWallAdmissionEvidence(Plane                 *p_wall_in,
                                  const SystemParams    *p_systemParams_in,
                                  const Eigen::Vector3d &groundNormal_World_in)
{
    WallAdmissionEvidence evidence;

    if (p_wall_in == nullptr || p_wall_in->isBad() ||
        p_systemParams_in == nullptr)
    {
        return evidence;
    }

    evidence.observationCount = p_wall_in->getObservationCount();

    const Plane::GeometrySnapshot geometry = p_wall_in->getGeometrySnapshot();
    Eigen::Vector4d               equation_World = geometry.equation_World;
    const double                  normalNorm = equation_World.head<3>().norm();

    if (!equation_World.allFinite() || !std::isfinite(normalNorm) ||
        normalNorm < 1e-8)
    {
        return evidence;
    }

    equation_World /= normalNorm;
    const Eigen::Vector3d normal_World = equation_World.head<3>();

    if (!equation_World.allFinite() ||
        std::abs(normal_World.norm() - 1.0) > 1e-6)
    {
        return evidence;
    }

    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_cloud =
        geometry.supportCloud;

    if (p_cloud == nullptr)
    {
        return evidence;
    }

    /*!
     * Ground-aligned in-plane axes so the two extents below correspond to
     * physical horizontal width and vertical height, not an arbitrary
     * in-plane rotation. A door-frame post rotated relative to an arbitrary
     * unitOrthogonal() axis can inflate BOTH bounding-box extents past a
     * width/height threshold even though its true width is a few
     * centimetres -- ground-anchoring the axes removes that degree of
     * freedom. axisHorizontal is the wall's own horizontal (along-wall)
     * direction: orthogonal to both the ground normal and the wall normal.
     * Falls back to the previous arbitrary-orthogonal axes when no ground
     * plane is available yet (early in a mission) or the wall is itself
     * near-horizontal (groundNormal parallel to normal_World).
     */
    const double    groundNormalNorm = groundNormal_World_in.norm();
    Eigen::Vector3d axisU_World      = Eigen::Vector3d::Zero();
    Eigen::Vector3d axisV_World      = Eigen::Vector3d::Zero();
    if (std::isfinite(groundNormalNorm) && groundNormalNorm > 1e-8)
    {
        const Eigen::Vector3d unitGroundNormal_World =
            groundNormal_World_in / groundNormalNorm;
        const Eigen::Vector3d horizontalCandidate_World =
            unitGroundNormal_World.cross(normal_World);
        const double horizontalNorm = horizontalCandidate_World.norm();
        if (std::isfinite(horizontalNorm) && horizontalNorm > 1e-3)
        {
            axisU_World = horizontalCandidate_World / horizontalNorm;
            axisV_World = axisU_World.cross(normal_World).normalized();
        }
    }
    if (axisU_World.squaredNorm() < 0.5 || axisV_World.squaredNorm() < 0.5)
    {
        axisU_World = normal_World.unitOrthogonal().normalized();
        axisV_World = normal_World.cross(axisU_World).normalized();
    }
    double minimumU_m = std::numeric_limits<double>::infinity();
    double maximumU_m = -std::numeric_limits<double>::infinity();
    double minimumV_m = std::numeric_limits<double>::infinity();
    double maximumV_m = -std::numeric_limits<double>::infinity();

    for (const pcl::PointXYZRGBA &point : p_cloud->points)
    {
        if (!pcl::isFinite(point))
        {
            continue;
        }

        const Eigen::Vector3d point_World_m(point.x, point.y, point.z);
        evidence.finitePointCount++;

        const double fitDistance_m =
            std::abs(normal_World.dot(point_World_m) + equation_World(3));

        if (fitDistance_m > p_systemParams_in->seg.ransac.distance_thresh)
        {
            continue;
        }

        const double pointU_m = point_World_m.dot(axisU_World);
        const double pointV_m = point_World_m.dot(axisV_World);
        minimumU_m            = std::min(minimumU_m, pointU_m);
        maximumU_m            = std::max(maximumU_m, pointU_m);
        minimumV_m            = std::min(minimumV_m, pointV_m);
        maximumV_m            = std::max(maximumV_m, pointV_m);
        evidence.fittedPointCount++;
    }

    const double extentU_m     = maximumU_m - minimumU_m;
    const double extentV_m     = maximumV_m - minimumV_m;
    const double majorExtent_m = std::max(extentU_m, extentV_m);
    const double minorExtent_m = std::min(extentU_m, extentV_m);
    const double area_m2       = majorExtent_m * minorExtent_m;
    const SystemParams::sem_seg::WallCreation &wallCreation =
        p_systemParams_in->sem_seg.wallCreation;
    const double fitSupportRatio =
        evidence.finitePointCount > 0U
            ? static_cast<double>(evidence.fittedPointCount) /
                  static_cast<double>(evidence.finitePointCount)
            : 0.0;

    evidence.adequateFiniteFit =
        evidence.fittedPointCount >= 20U &&
        fitSupportRatio >=
            p_systemParams_in->room_seg.minimumWallSupportRatio &&
        std::isfinite(majorExtent_m) && std::isfinite(minorExtent_m) &&
        std::isfinite(area_m2) &&
        majorExtent_m >= wallCreation.minimumMajorExtent_m &&
        minorExtent_m >= wallCreation.minimumMinorExtent_m &&
        area_m2 >= wallCreation.minimumArea_m2;

    const std::size_t strongObservationPointCount =
        wallCreation.connectivity.enabled
            ? std::max<std::size_t>(
                  wallCreation.minimumPointCount,
                  wallCreation.connectivity.minimumComponentPointCount)
            : wallCreation.minimumPointCount;
    const bool strongFirstObservationEvidence =
        evidence.adequateFiniteFit &&
        evidence.fittedPointCount >= strongObservationPointCount;
    const bool repeatedObservationEvidence =
        evidence.observationCount >=
        std::max<std::size_t>(
            p_systemParams_in->room_seg.minimumWallObservationCount,
            1U);
    const bool wallDominatesSemantics =
        p_wall_in->getPlaneType() == Plane::planeVariant::WALL &&
        p_wall_in->getExpectedPlaneType() == Plane::planeVariant::WALL;

    evidence.admissible =
        wallDominatesSemantics && evidence.adequateFiniteFit &&
        (repeatedObservationEvidence || strongFirstObservationEvidence);
    return evidence;
}

/*!
 * @brief Computes the scalar two-dimensional cross product.
 *
 * @param[in] firstVector_in First vector.
 * @param[in] secondVector_in Second vector.
 * @return Signed scalar cross product.
 */
double crossProduct2d(const Eigen::Vector2d &firstVector_in,
                      const Eigen::Vector2d &secondVector_in)
{
    return firstVector_in.x() * secondVector_in.y() -
           firstVector_in.y() * secondVector_in.x();
}

/*!
 * @brief           Builds a robust finite wall segment on the horizontal ground
 *                  plane.
 *
 * @param[in]       p_wall_in
 *                  Wall whose observed cloud defines the finite extent.
 *
 * @param[in]       groundNormal_World_in
 *                  Unit ground normal in the world frame.
 *
 * @param[in]       groundAxisU_World_in
 *                  First horizontal ground axis.
 *
 * @param[in]       groundAxisV_World_in
 *                  Second horizontal ground axis.
 *
 * @param[in]       endpointTrimRatio_in
 *                  Fraction trimmed from both extent tails.
 *
 * @param[in]       minimumWallLength_m_in
 *                  Minimum accepted horizontal length.
 *
 * @param[out]      segment_out
 *                  Resulting finite horizontal segment.
 *
 * @return          True when the wall provides a valid finite segment.
 */
bool buildFiniteWallSegment2d(Plane                 *p_wall_in,
                              const Eigen::Vector3d &groundNormal_World_in,
                              const Eigen::Vector3d &groundAxisU_World_in,
                              const Eigen::Vector3d &groundAxisV_World_in,
                              const double           endpointTrimRatio_in,
                              const double           minimumWallLength_m_in,
                              FiniteWallSegment2d   &segment_out)
{
    if (p_wall_in == nullptr || p_wall_in->isBad())
    {
        return false;
    }

    const Plane::GeometrySnapshot wallGeometry =
        p_wall_in->getGeometrySnapshot();
    Eigen::Vector4d wallEquation_World = wallGeometry.equation_World;
    const double    wallNormalNorm     = wallEquation_World.head<3>().norm();

    if (!wallEquation_World.allFinite() || wallNormalNorm < 1e-8)
    {
        return false;
    }

    wallEquation_World /= wallNormalNorm;

    Eigen::Vector3d horizontalWallNormal_World =
        wallEquation_World.head<3>() -
        wallEquation_World.head<3>().dot(groundNormal_World_in) *
            groundNormal_World_in;

    const double horizontalWallNormalNorm = horizontalWallNormal_World.norm();

    if (!std::isfinite(horizontalWallNormalNorm) ||
        horizontalWallNormalNorm < 1e-8)
    {
        return false;
    }

    horizontalWallNormal_World /= horizontalWallNormalNorm;

    const Eigen::Vector3d wallTangent_World =
        groundNormal_World_in.cross(horizontalWallNormal_World).normalized();

    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_wallCloud =
        wallGeometry.supportCloud;

    if (p_wallCloud == nullptr || p_wallCloud->empty())
    {
        return false;
    }

    std::vector<double> wallPointCoordinates_m;
    wallPointCoordinates_m.reserve(p_wallCloud->size());

    for (const pcl::PointXYZRGBA &wallPoint : p_wallCloud->points)
    {
        if (!pcl::isFinite(wallPoint))
        {
            continue;
        }

        const Eigen::Vector3d wallPoint_World_m(
            static_cast<double>(wallPoint.x),
            static_cast<double>(wallPoint.y),
            static_cast<double>(wallPoint.z));

        wallPointCoordinates_m.push_back(
            wallPoint_World_m.dot(wallTangent_World));
    }

    if (wallPointCoordinates_m.size() < 2U)
    {
        return false;
    }

    std::sort(wallPointCoordinates_m.begin(), wallPointCoordinates_m.end());

    const double boundedTrimRatio = std::clamp(endpointTrimRatio_in, 0.0, 0.45);
    const std::size_t trimmedPointCount = static_cast<std::size_t>(
        std::floor(boundedTrimRatio *
                   static_cast<double>(wallPointCoordinates_m.size() - 1U)));
    const std::size_t maximumCoordinateIndex =
        wallPointCoordinates_m.size() - 1U - trimmedPointCount;

    const double minimumWallCoordinate_m =
        wallPointCoordinates_m[trimmedPointCount];
    const double maximumWallCoordinate_m =
        wallPointCoordinates_m[maximumCoordinateIndex];

    const Eigen::Vector3d wallCentroid_World_m =
        p_wall_in->getCentroid().cast<double>();

    if (!wallCentroid_World_m.allFinite())
    {
        return false;
    }

    const double centroidCoordinate_m =
        wallCentroid_World_m.dot(wallTangent_World);
    const Eigen::Vector3d segmentStart_World_m =
        wallCentroid_World_m +
        (minimumWallCoordinate_m - centroidCoordinate_m) * wallTangent_World;
    const Eigen::Vector3d segmentEnd_World_m =
        wallCentroid_World_m +
        (maximumWallCoordinate_m - centroidCoordinate_m) * wallTangent_World;

    segment_out.p_wall        = p_wall_in;
    segment_out.start_World_m = {
        segmentStart_World_m.dot(groundAxisU_World_in),
        segmentStart_World_m.dot(groundAxisV_World_in)};
    segment_out.end_World_m = {segmentEnd_World_m.dot(groundAxisU_World_in),
                               segmentEnd_World_m.dot(groundAxisV_World_in)};
    segment_out.length_m =
        (segment_out.end_World_m - segment_out.start_World_m).norm();
    segment_out.supportScore =
        static_cast<double>(std::max<std::size_t>(
            static_cast<std::size_t>(p_wall_in->getObservationCount()),
            1U)) *
        std::sqrt(std::max(segment_out.length_m, 0.0));

    return segment_out.start_World_m.allFinite() &&
           segment_out.end_World_m.allFinite() &&
           std::isfinite(segment_out.length_m) &&
           segment_out.length_m >= minimumWallLength_m_in;
}

/*!
 * @brief           Intersects the infinite lines supporting two finite wall
 *                  segments.
 *
 * @param[in]       firstSegment_in
 *                  First wall segment.
 *
 * @param[in]       secondSegment_in
 *                  Second wall segment.
 *
 * @param[out]      intersection_World_m_out
 *                  Intersection in horizontal world axes.
 *
 * @param[out]      firstParameter_out
 *                  Parametric coordinate on the first segment.
 *
 * @param[out]      secondParameter_out
 *                  Parametric coordinate on the second segment.
 *
 * @return          False when the supporting lines are parallel.
 */
bool intersectSupportingLines(const FiniteWallSegment2d &firstSegment_in,
                              const FiniteWallSegment2d &secondSegment_in,
                              Eigen::Vector2d &intersection_World_m_out,
                              double          &firstParameter_out,
                              double          &secondParameter_out)
{
    const Eigen::Vector2d firstDirection =
        firstSegment_in.end_World_m - firstSegment_in.start_World_m;
    const Eigen::Vector2d secondDirection =
        secondSegment_in.end_World_m - secondSegment_in.start_World_m;
    const double denominator = crossProduct2d(firstDirection, secondDirection);

    if (!std::isfinite(denominator) || std::abs(denominator) < 1e-8)
    {
        return false;
    }

    const Eigen::Vector2d startOffset =
        secondSegment_in.start_World_m - firstSegment_in.start_World_m;
    firstParameter_out =
        crossProduct2d(startOffset, secondDirection) / denominator;
    secondParameter_out =
        crossProduct2d(startOffset, firstDirection) / denominator;
    intersection_World_m_out =
        firstSegment_in.start_World_m + firstParameter_out * firstDirection;

    return intersection_World_m_out.allFinite() &&
           std::isfinite(firstParameter_out) &&
           std::isfinite(secondParameter_out);
}

/*!
 * @brief Returns the Euclidean distance from a point to a finite segment.
 */
double pointToSegmentDistance_m(const Eigen::Vector2d     &point_World_m_in,
                                const FiniteWallSegment2d &segment_in)
{
    const Eigen::Vector2d segmentDirection =
        segment_in.end_World_m - segment_in.start_World_m;
    const double squaredLength = segmentDirection.squaredNorm();

    if (squaredLength < 1e-12)
    {
        return (point_World_m_in - segment_in.start_World_m).norm();
    }

    const double interpolation = std::clamp(
        (point_World_m_in - segment_in.start_World_m).dot(segmentDirection) /
            squaredLength,
        0.0,
        1.0);

    return (point_World_m_in -
            (segment_in.start_World_m + interpolation * segmentDirection))
        .norm();
}

/*! @brief Result of attempting to close a set of wall segments into one
 *  ordered, non-open loop. */
struct WallLoopClosure
{
    bool                         hasOpenBoundary = true;
    std::vector<Eigen::Vector2d> corners_World_m;
};

/*!
 * @brief Attempts to close the given wall segments (sorted here by angle
 *        from the room centroid) into one ordered loop, exactly as
 *        validateRoomBoundaries() always did for a room's full wall set.
 *        Factored out so the caller can retry on a reduced subset when the
 *        full set doesn't close (see validateRoomBoundaries()'s single-
 *        outlier-exclusion retry).
 */
WallLoopClosure tryCloseWallLoop(
    std::vector<FiniteWallSegment2d>                wallSegments_in,
    const Eigen::Vector2d                          &roomCentroid_Ground_m_in,
    const SystemParams::room_seg::BoundaryTopology &topologyParameters_in)
{
    WallLoopClosure result;

    if (wallSegments_in.empty())
    {
        return result;
    }

    std::sort(
        wallSegments_in.begin(),
        wallSegments_in.end(),
        [&roomCentroid_Ground_m_in](const FiniteWallSegment2d &firstSegment,
                                    const FiniteWallSegment2d &secondSegment)
        {
            const Eigen::Vector2d firstMidpoint =
                0.5 * (firstSegment.start_World_m + firstSegment.end_World_m) -
                roomCentroid_Ground_m_in;
            const Eigen::Vector2d secondMidpoint =
                0.5 *
                    (secondSegment.start_World_m + secondSegment.end_World_m) -
                roomCentroid_Ground_m_in;

            return std::atan2(firstMidpoint.y(), firstMidpoint.x()) <
                   std::atan2(secondMidpoint.y(), secondMidpoint.x());
        });

    result.corners_World_m.reserve(wallSegments_in.size());

    for (std::size_t wallIndex = 0U; wallIndex < wallSegments_in.size();
         ++wallIndex)
    {
        const FiniteWallSegment2d &currentWall = wallSegments_in[wallIndex];
        const FiniteWallSegment2d &nextWall =
            wallSegments_in[(wallIndex + 1U) % wallSegments_in.size()];
        Eigen::Vector2d corner_World_m;
        double          currentParameter = 0.0;
        double          nextParameter    = 0.0;

        if (intersectSupportingLines(currentWall,
                                     nextWall,
                                     corner_World_m,
                                     currentParameter,
                                     nextParameter))
        {
            const double currentCornerGap_m =
                pointToSegmentDistance_m(corner_World_m, currentWall);
            const double nextCornerGap_m =
                pointToSegmentDistance_m(corner_World_m, nextWall);

            if (currentCornerGap_m <=
                    topologyParameters_in.maximumCornerGap_m &&
                nextCornerGap_m <= topologyParameters_in.maximumCornerGap_m)
            {
                result.corners_World_m.push_back(corner_World_m);
                continue;
            }
        }

        const std::array<std::pair<Eigen::Vector2d, Eigen::Vector2d>, 4>
            endpointPairs = {
                {{currentWall.start_World_m, nextWall.start_World_m},
                 {currentWall.start_World_m, nextWall.end_World_m},
                 {currentWall.end_World_m, nextWall.start_World_m},
                 {currentWall.end_World_m, nextWall.end_World_m}}};

        auto nearestEndpointPair = std::min_element(
            endpointPairs.begin(),
            endpointPairs.end(),
            [](const auto &firstPair, const auto &secondPair)
            {
                return (firstPair.first - firstPair.second).squaredNorm() <
                       (secondPair.first - secondPair.second).squaredNorm();
            });

        if ((nearestEndpointPair->first - nearestEndpointPair->second).norm() <=
            topologyParameters_in.maximumCornerGap_m)
        {
            result.corners_World_m.push_back(
                0.5 *
                (nearestEndpointPair->first + nearestEndpointPair->second));
            continue;
        }

        result.corners_World_m.clear();
        return result;
    }

    if (result.corners_World_m.size() == wallSegments_in.size())
    {
        result.hasOpenBoundary = false;
    }
    else
    {
        result.corners_World_m.clear();
    }

    return result;
}

/*!
 * @brief Computes the unsigned area of an ordered horizontal polygon.
 */
/*!
 * @brief Finds the angular sectors (from roomCentroid_Ground_m_in) with no
 *        wall evidence -- the "where is this room still unobserved" signal
 *        (user rule: track and expose incomplete-room state, not just a
 *        pass/fail boundary status).
 *
 *        Deliberately coarser than the corner-closing algorithm above: each
 *        wall is reduced to its 2D midpoint angle from the centroid, not its
 *        true angular extent, trading a small amount of precision (a wide
 *        wall's own angular span isn't subtracted from a neighbouring gap)
 *        for a computation that stays meaningful at any wall count,
 *        including 0 or 1 -- the boundary-loop algorithm's own machinery
 *        only starts producing useful output once minimumWallCount is met.
 */
std::vector<Room::ObservationGap> computeRoomObservationGaps(
    const std::vector<FiniteWallSegment2d> &wallSegments_in,
    const Eigen::Vector2d                  &roomCentroid_Ground_m_in,
    /* An axis-aligned (or any) rectangle's four wall midpoints sit exactly
     * on its principal axes as seen from the centroid -- always exactly 90
     * deg apart by construction, regardless of aspect ratio. The threshold
     * must clear that deterministic case with margin, or every well-formed
     * rectangular room reports four phantom gaps. */
    double gapThreshold_rad_in = 100.0 * M_PI / 180.0)
{
    std::vector<Room::ObservationGap> gaps;

    if (wallSegments_in.empty())
    {
        gaps.push_back({0.0, 2.0 * M_PI});
        return gaps;
    }

    std::vector<double> midpointAngles_rad;
    midpointAngles_rad.reserve(wallSegments_in.size());
    for (const FiniteWallSegment2d &segment : wallSegments_in)
    {
        const Eigen::Vector2d midpoint_Ground_m =
            0.5 * (segment.start_World_m + segment.end_World_m) -
            roomCentroid_Ground_m_in;
        if (!midpoint_Ground_m.allFinite() ||
            midpoint_Ground_m.squaredNorm() < 1e-12)
        {
            continue;
        }
        midpointAngles_rad.push_back(
            std::atan2(midpoint_Ground_m.y(), midpoint_Ground_m.x()));
    }

    if (midpointAngles_rad.empty())
    {
        gaps.push_back({0.0, 2.0 * M_PI});
        return gaps;
    }

    std::sort(midpointAngles_rad.begin(), midpointAngles_rad.end());

    for (std::size_t index = 0U; index < midpointAngles_rad.size(); ++index)
    {
        const double thisAngle_rad = midpointAngles_rad[index];
        const double nextAngle_rad = (index + 1U < midpointAngles_rad.size())
                                         ? midpointAngles_rad[index + 1U]
                                         : midpointAngles_rad[0] + 2.0 * M_PI;
        const double span_rad      = nextAngle_rad - thisAngle_rad;

        if (span_rad > gapThreshold_rad_in)
        {
            gaps.push_back({thisAngle_rad, span_rad});
        }
    }

    return gaps;
}

double computePolygonArea_m2(
    const std::vector<Eigen::Vector2d> &polygonVertices_World_m_in)
{
    if (polygonVertices_World_m_in.size() < 3U)
    {
        return 0.0;
    }

    double signedTwiceArea_m2 = 0.0;

    for (std::size_t vertexIndex = 0U;
         vertexIndex < polygonVertices_World_m_in.size();
         ++vertexIndex)
    {
        const Eigen::Vector2d &currentVertex =
            polygonVertices_World_m_in[vertexIndex];
        const Eigen::Vector2d &nextVertex =
            polygonVertices_World_m_in[(vertexIndex + 1U) %
                                       polygonVertices_World_m_in.size()];
        signedTwiceArea_m2 += crossProduct2d(currentVertex, nextVertex);
    }

    return 0.5 * std::abs(signedTwiceArea_m2);
}

/*!
 * @brief Tests whether a segment crosses a passage aperture.
 *
 * @param[in] segmentStart_World_m_in First endpoint in the active map frame.
 * @param[in] segmentEnd_World_m_in Second endpoint in the active map frame.
 * @param[in] p_passage_in Passage defining the finite aperture.
 * @param[in] groundNormal_World_in Unit ground normal in the active map frame.
 * @param[in] openingMargin_m_in Aperture expansion used for noisy geometry.
 * @param[in] minimumSideDistance_m_in Required endpoint distance from plane.
 * @param[in] requirePassable_in True when the passage must already be passable
 *              before the geometric test may fire. Far-side wall routing may
 *              pass false so the aperture geometry alone drives the decision
 *              even while the passage is still being confirmed.
 * @return True when the segment crosses inside the finite opening.
 */
/*!
 * @brief       Core aperture-crossing math shared by both a confirmed
 *              Passage and a still-unconfirmed OpenPassageEvidence
 *              hypothesis (see segmentCrossesOpenPassageEvidence below) --
 *              the two differ only in where the plane equation, centroid,
 *              and opening size come from, never in how the crossing test
 *              itself works.
 */
static bool
    segmentCrossesAperture(const Eigen::Vector3d &segmentStart_World_m_in,
                           const Eigen::Vector3d &segmentEnd_World_m_in,
                           const Eigen::Vector4d &apertureEquation_World_in,
                           const Eigen::Vector3d &apertureCentroid_World_m_in,
                           const double           apertureWidth_m_in,
                           const double           apertureHeight_m_in,
                           const Eigen::Vector3d &groundNormal_World_in,
                           const double           openingMargin_m_in,
                           const double           minimumSideDistance_m_in)
{
    if (!segmentStart_World_m_in.allFinite() ||
        !segmentEnd_World_m_in.allFinite() ||
        !apertureCentroid_World_m_in.allFinite())
    {
        return false;
    }

    Eigen::Vector4d apertureEquation_World = apertureEquation_World_in;
    const double apertureNormalNorm = apertureEquation_World.head<3>().norm();

    if (!apertureEquation_World.allFinite() || apertureNormalNorm < 1e-8)
    {
        return false;
    }

    apertureEquation_World /= apertureNormalNorm;
    const Eigen::Vector3d apertureNormal_World =
        apertureEquation_World.head<3>();
    const double startSide_m =
        apertureNormal_World.dot(segmentStart_World_m_in) +
        apertureEquation_World(3);
    const double endSide_m = apertureNormal_World.dot(segmentEnd_World_m_in) +
                             apertureEquation_World(3);

    if (startSide_m * endSide_m >= 0.0 ||
        std::abs(startSide_m) < minimumSideDistance_m_in ||
        std::abs(endSide_m) < minimumSideDistance_m_in)
    {
        return false;
    }

    const double interpolation = startSide_m / (startSide_m - endSide_m);

    if (!std::isfinite(interpolation) || interpolation < 0.0 ||
        interpolation > 1.0)
    {
        return false;
    }

    const Eigen::Vector3d intersection_World_m =
        segmentStart_World_m_in +
        interpolation * (segmentEnd_World_m_in - segmentStart_World_m_in);

    Eigen::Vector3d apertureOffset_World_m =
        intersection_World_m - apertureCentroid_World_m_in;
    apertureOffset_World_m -=
        apertureOffset_World_m.dot(apertureNormal_World) * apertureNormal_World;

    const double verticalOffset_m =
        std::abs(apertureOffset_World_m.dot(groundNormal_World_in));
    const Eigen::Vector3d horizontalOffset_World_m =
        apertureOffset_World_m -
        apertureOffset_World_m.dot(groundNormal_World_in) *
            groundNormal_World_in;
    const double horizontalOffset_m = horizontalOffset_World_m.norm();

    return horizontalOffset_m <=
               0.5 * apertureWidth_m_in + openingMargin_m_in &&
           verticalOffset_m <= 0.5 * apertureHeight_m_in + openingMargin_m_in;
}

bool segmentCrossesPassageOpening(
    const Eigen::Vector3d &segmentStart_World_m_in,
    const Eigen::Vector3d &segmentEnd_World_m_in,
    Passage               *p_passage_in,
    const Eigen::Vector3d &groundNormal_World_in,
    const double           openingMargin_m_in,
    const double           minimumSideDistance_m_in,
    const bool             requirePassable_in = true)
{
    if (p_passage_in == nullptr ||
        (requirePassable_in && !p_passage_in->isPassable()))
    {
        return false;
    }

    return segmentCrossesAperture(segmentStart_World_m_in,
                                  segmentEnd_World_m_in,
                                  p_passage_in->getGlobalEquation().coeffs(),
                                  p_passage_in->getCentroid(),
                                  p_passage_in->getWidth(),
                                  p_passage_in->getHeight(),
                                  groundNormal_World_in,
                                  openingMargin_m_in,
                                  minimumSideDistance_m_in);
}

/*!
 * @brief       Same aperture-crossing test as segmentCrossesPassageOpening,
 *              but against a still-unconfirmed OpenPassageEvidence
 *              hypothesis instead of a confirmed Passage.
 *
 *              Passage confirmation requires several genuinely independent
 *              Voxblox skeleton snapshots (minimumConfirmationSnapshots,
 *              config-gated to guard against double-counting one latched ROS
 *              message -- see the skeleton-fingerprint check in
 *              detectDoorsAndDoorways()) and therefore real elapsed
 *              exploration time. Until that confirmation completes, no
 *              Passage object exists for mpAtlas->GetAllPassages() to
 *              return, so any far-side-routing check that only consults
 *              confirmed passages is blind for that entire window -- a wall
 *              genuinely on the far side of a real, already-evidenced
 *              opening falls through to ordinary admission and gets bound
 *              to the WRONG (near) room, exactly the corruption far-side
 *              routing exists to prevent. Using the same aperture geometry
 *              math against the pending evidence (its supporting wall's
 *              plane stands in for the eventual passage plane, its
 *              openingRadius_m/heightSpan_m for the eventual width/height --
 *              the same derivation createMapPassage() itself uses once
 *              confirmed) closes that window without weakening the
 *              confirmation gate itself: the passage still is not created,
 *              only wall ADMISSION becomes conservative while its identity
 *              is still ambiguous.
 */
static bool segmentCrossesOpenPassageEvidence(
    const Eigen::Vector3d &segmentStart_World_m_in,
    const Eigen::Vector3d &segmentEnd_World_m_in,
    Plane                 *p_evidenceSupportingWall_in,
    const Eigen::Vector3d &evidenceCentroid_World_m_in,
    const double           evidenceOpeningRadius_m_in,
    const double           evidenceHeightSpan_m_in,
    const Eigen::Vector3d &groundNormal_World_in,
    const double           openingMargin_m_in,
    const double           minimumSideDistance_m_in)
{
    if (p_evidenceSupportingWall_in == nullptr ||
        p_evidenceSupportingWall_in->isBad() ||
        evidenceOpeningRadius_m_in <= 0.0)
    {
        return false;
    }

    constexpr double defaultOpenPassageHeight_m = 2.0;

    return segmentCrossesAperture(
        segmentStart_World_m_in,
        segmentEnd_World_m_in,
        p_evidenceSupportingWall_in->getGlobalEquation().coeffs(),
        evidenceCentroid_World_m_in,
        2.0 * evidenceOpeningRadius_m_in,
        std::max(evidenceHeightSpan_m_in, defaultOpenPassageHeight_m),
        groundNormal_World_in,
        openingMargin_m_in,
        minimumSideDistance_m_in);
}

/*!
 * @brief       Tests whether a straight segment between two points is
 *              blocked by a wall belonging to a room other than the ones
 *              the segment is meant to connect.
 *
 *              Threading one passage's own bounded aperture is necessary
 *              but not sufficient proof that two points are the direct two
 *              sides of THAT passage: in a corridor with several rooms and
 *              doors in a row, a straight line can thread one passage's
 *              opening while still passing directly through an
 *              intervening room's own wall. When it does, something else
 *              -- a wall, and by implication a room -- provably sits
 *              between the two points, so they are not each other's
 *              direct neighbour through this passage.
 *
 * @param[in]   segmentStart_World_m_in
 *              One endpoint of the candidate segment.
 * @param[in]   segmentEnd_World_m_in
 *              The other endpoint of the candidate segment.
 * @param[in]   excludedRooms_in
 *              Rooms whose own walls are not "foreign" -- typically the
 *              rooms/placeholders the segment itself is testing.
 * @param[in]   allRooms_in
 *              Every currently known room to search for a blocking wall.
 * @param[in]   groundAxisU_World_in
 *              First horizontal ground axis (matches buildFiniteWallSegment2d).
 * @param[in]   groundAxisV_World_in
 *              Second horizontal ground axis.
 * @param[in]   groundNormal_World_in
 *              Unit ground normal in the world frame.
 * @param[in]   endpointTrimRatio_in
 *              Forwarded to buildFiniteWallSegment2d.
 * @param[in]   minimumWallLength_m_in
 *              Forwarded to buildFiniteWallSegment2d.
 *
 * @return      True when a foreign room's own finite wall extent blocks
 *              the segment.
 */
bool segmentCrossesForeignWall(
    const Eigen::Vector3d                &segmentStart_World_m_in,
    const Eigen::Vector3d                &segmentEnd_World_m_in,
    const std::vector<ORB_SLAM3::Room *> &excludedRooms_in,
    const std::vector<ORB_SLAM3::Room *> &allRooms_in,
    const Eigen::Vector3d                &groundAxisU_World_in,
    const Eigen::Vector3d                &groundAxisV_World_in,
    const Eigen::Vector3d                &groundNormal_World_in,
    const double                          endpointTrimRatio_in,
    const double                          minimumWallLength_m_in)
{
    if (!segmentStart_World_m_in.allFinite() ||
        !segmentEnd_World_m_in.allFinite())
    {
        return false;
    }

    FiniteWallSegment2d testSegment;
    testSegment.start_World_m = {
        segmentStart_World_m_in.dot(groundAxisU_World_in),
        segmentStart_World_m_in.dot(groundAxisV_World_in)};
    testSegment.end_World_m = {segmentEnd_World_m_in.dot(groundAxisU_World_in),
                               segmentEnd_World_m_in.dot(groundAxisV_World_in)};

    for (ORB_SLAM3::Room *p_room : allRooms_in)
    {
        if (p_room == nullptr || p_room->isBad() ||
            std::find(excludedRooms_in.begin(),
                      excludedRooms_in.end(),
                      p_room) != excludedRooms_in.end())
        {
            continue;
        }

        for (Plane *p_wall : p_room->getWalls())
        {
            FiniteWallSegment2d wallSegment;

            if (!buildFiniteWallSegment2d(p_wall,
                                          groundNormal_World_in,
                                          groundAxisU_World_in,
                                          groundAxisV_World_in,
                                          endpointTrimRatio_in,
                                          minimumWallLength_m_in,
                                          wallSegment))
            {
                continue;
            }

            Eigen::Vector2d intersection_World_m;
            double          testParameter = 0.0;
            double          wallParameter = 0.0;

            if (intersectSupportingLines(testSegment,
                                         wallSegment,
                                         intersection_World_m,
                                         testParameter,
                                         wallParameter) &&
                testParameter > 0.0 && testParameter < 1.0 &&
                wallParameter >= 0.0 && wallParameter <= 1.0)
            {
                return true;
            }
        }
    }

    return false;
}

/*!
 * @brief       Tests whether two maps observe a common tagged room name.
 *
 *              A new map is a deterministic merge candidate for the current
 *              map only when at least one non-empty room tag collected from
 *              the other map's detected and marker-based rooms also appears
 *              among the current map's tagged rooms. Room tags originate from
 *              context snapshots and are propagated by matchRoomsToContext,
 *              so they are a stable correspondences key between maps.
 *
 * @param[in]   p_firstMap_in
 *              Map whose detected and marker-based room tags are collected.
 * @param[in]   p_secondMap_in
 *              Map whose tagged rooms are tested against the collected tags.
 *
 * @return      True when both maps observe at least one shared room tag.
 */
bool sharesRoomNameTag(Map *p_firstMap_in, Map *p_secondMap_in)
{
    std::unordered_set<std::string> firstMapRoomTags;
    for (Room *p_room : p_firstMap_in->GetAllDetectedMapRooms())
    {
        if (p_room->hasRoomTag())
        {
            firstMapRoomTags.insert(p_room->getRoomTag());
        }
    }
    for (Room *p_room : p_firstMap_in->GetAllMarkerBasedMapRooms())
    {
        if (p_room->hasRoomTag())
        {
            firstMapRoomTags.insert(p_room->getRoomTag());
        }
    }

    if (firstMapRoomTags.empty())
    {
        std::cout << "[SemMgr] sharesRoomNameTag: first map (id="
                  << p_firstMap_in->GetId()
                  << ") has NO tagged rooms (detected="
                  << p_firstMap_in->GetAllDetectedMapRooms().size()
                  << ", marker="
                  << p_firstMap_in->GetAllMarkerBasedMapRooms().size() << ")"
                  << std::endl;
        return false;
    }

    for (Room *p_room : p_secondMap_in->GetAllDetectedMapRooms())
    {
        if (p_room->hasRoomTag() &&
            firstMapRoomTags.count(p_room->getRoomTag()) != 0U)
        {
            std::cout << "[SemMgr] sharesRoomNameTag: MATCH found tag "
                      << p_room->getRoomTag() << " between maps "
                      << p_firstMap_in->GetId() << " and "
                      << p_secondMap_in->GetId() << std::endl;
            return true;
        }
    }
    for (Room *p_room : p_secondMap_in->GetAllMarkerBasedMapRooms())
    {
        if (p_room->hasRoomTag() &&
            firstMapRoomTags.count(p_room->getRoomTag()) != 0U)
        {
            std::cout << "[SemMgr] sharesRoomNameTag: MATCH found tag "
                      << p_room->getRoomTag() << " between maps "
                      << p_firstMap_in->GetId() << " and "
                      << p_secondMap_in->GetId() << std::endl;
            return true;
        }
    }

    std::cout << "[SemMgr] sharesRoomNameTag: NO match. First map (id="
              << p_firstMap_in->GetId() << ") tags: " << firstMapRoomTags.size()
              << ", second map (id=" << p_secondMap_in->GetId() << ") detected="
              << p_secondMap_in->GetAllDetectedMapRooms().size() << " marker="
              << p_secondMap_in->GetAllMarkerBasedMapRooms().size()
              << std::endl;

    return false;
}

/*!
 * @brief Projects a WALL Plane's finite support cloud onto a shared in-plane
 *        tangent frame, returning the resulting axis-aligned interval.
 *
 * @return false when the plane has no usable geometry (null/empty cloud, or
 *         a degenerate equation); the caller must treat that as "cannot
 *         claim overlap" rather than as a zero-size interval.
 */
bool projectPlaneFootprintOntoSharedAxes(Plane                 *p_plane_in,
                                         const Eigen::Vector3d &axisU_World_in,
                                         const Eigen::Vector3d &axisV_World_in,
                                         double                &minU_m_out,
                                         double                &maxU_m_out,
                                         double                &minV_m_out,
                                         double                &maxV_m_out)
{
    if (p_plane_in == nullptr)
    {
        return false;
    }

    const Plane::GeometrySnapshot geometry = p_plane_in->getGeometrySnapshot();
    if (geometry.supportCloud == nullptr || geometry.supportCloud->empty())
    {
        return false;
    }

    minU_m_out = std::numeric_limits<double>::infinity();
    maxU_m_out = -std::numeric_limits<double>::infinity();
    minV_m_out = std::numeric_limits<double>::infinity();
    maxV_m_out = -std::numeric_limits<double>::infinity();

    for (const pcl::PointXYZRGBA &point : geometry.supportCloud->points)
    {
        if (!pcl::isFinite(point))
        {
            continue;
        }
        const Eigen::Vector3d point_World_m(point.x, point.y, point.z);
        const double          pointU_m = point_World_m.dot(axisU_World_in);
        const double          pointV_m = point_World_m.dot(axisV_World_in);
        minU_m_out                     = std::min(minU_m_out, pointU_m);
        maxU_m_out                     = std::max(maxU_m_out, pointU_m);
        minV_m_out                     = std::min(minV_m_out, pointV_m);
        maxV_m_out                     = std::max(maxV_m_out, pointV_m);
    }

    return std::isfinite(minU_m_out) && std::isfinite(maxU_m_out) &&
           std::isfinite(minV_m_out) && std::isfinite(maxV_m_out);
}

/*!
 * @brief Decides whether two WALL Planes are plausibly the two opposite
 *        faces of the same physical wall (axiom (e)): parallel, a plausible
 *        wall thickness apart, observed from opposite exterior sides, and
 *        overlapping in-plane footprint.
 */
bool arePlausibleTwinWallFaces(Plane                 *p_first_in,
                               Plane                 *p_second_in,
                               double                 minimumThickness_m_in,
                               double                 maximumThickness_m_in,
                               double                 minimumOverlapRatio_in,
                               const Eigen::Vector3d &groundNormal_World_in)
{
    if (p_first_in == nullptr || p_first_in->isBad() ||
        p_second_in == nullptr || p_second_in->isBad() ||
        p_first_in == p_second_in)
    {
        return false;
    }

    if (!Utils::arePlanesParallel(p_first_in, p_second_in))
    {
        return false;
    }

    Eigen::Vector4d equation1   = p_first_in->getGlobalEquation().coeffs();
    Eigen::Vector4d equation2   = p_second_in->getGlobalEquation().coeffs();
    const double    normalNorm1 = equation1.head<3>().norm();
    const double    normalNorm2 = equation2.head<3>().norm();

    if (!equation1.allFinite() || !equation2.allFinite() ||
        normalNorm1 < 1e-8 || normalNorm2 < 1e-8)
    {
        return false;
    }

    equation1 /= normalNorm1;
    equation2 /= normalNorm2;

    /* Align equation2's sign to equation1's before comparing offsets --
     * plane-equation sign is arbitrary. */
    Eigen::Vector4d alignedEquation2 = equation2;
    if (equation1.head<3>().dot(equation2.head<3>()) < 0.0)
    {
        alignedEquation2 *= -1.0;
    }
    const double separation_m = std::abs(equation1(3) - alignedEquation2(3));

    if (separation_m < minimumThickness_m_in ||
        separation_m > maximumThickness_m_in)
    {
        return false;
    }

    /* Opposite-exterior-side check, generalising isWallFaceForeignToRoom's
     * side-sign math from plane-to-room to plane-to-plane: a wall's two
     * faces are observed from cameras standing on opposite exterior sides,
     * so each face's observation origin must resolve to opposite sides of
     * the OTHER face's equation. */
    const std::optional<Eigen::Vector3d> origin1 =
        p_first_in->getObservationOrigin_World();
    const std::optional<Eigen::Vector3d> origin2 =
        p_second_in->getObservationOrigin_World();

    if (!origin1.has_value() || !origin1->allFinite() || !origin2.has_value() ||
        !origin2->allFinite())
    {
        /* Planes created before the stamp existed carry no face identity;
         * make no claim rather than a wrong one. */
        return false;
    }

    constexpr double minimumResolvableSide_m = 0.10;
    const double     side1AtOrigin1 =
        equation1.head<3>().dot(origin1.value()) + equation1(3);
    const double side1AtOrigin2 =
        equation1.head<3>().dot(origin2.value()) + equation1(3);
    const double side2AtOrigin1 =
        equation2.head<3>().dot(origin1.value()) + equation2(3);
    const double side2AtOrigin2 =
        equation2.head<3>().dot(origin2.value()) + equation2(3);

    if (!std::isfinite(side1AtOrigin1) || !std::isfinite(side1AtOrigin2) ||
        !std::isfinite(side2AtOrigin1) || !std::isfinite(side2AtOrigin2) ||
        std::abs(side1AtOrigin1) < minimumResolvableSide_m ||
        std::abs(side1AtOrigin2) < minimumResolvableSide_m ||
        std::abs(side2AtOrigin1) < minimumResolvableSide_m ||
        std::abs(side2AtOrigin2) < minimumResolvableSide_m)
    {
        return false;
    }

    const bool oppositeAcrossPlane1 = (side1AtOrigin1 * side1AtOrigin2) < 0.0;
    const bool oppositeAcrossPlane2 = (side2AtOrigin1 * side2AtOrigin2) < 0.0;

    if (!oppositeAcrossPlane1 || !oppositeAcrossPlane2)
    {
        return false;
    }

    /* In-plane footprint overlap, projected onto one shared ground-anchored
     * tangent frame so the two planes' (possibly differently canonicalised)
     * own local U/V axes don't have to agree. */
    const double    groundNormalNorm = groundNormal_World_in.norm();
    Eigen::Vector3d axisU_World      = Eigen::Vector3d::Zero();
    Eigen::Vector3d axisV_World      = Eigen::Vector3d::Zero();
    if (std::isfinite(groundNormalNorm) && groundNormalNorm > 1e-8)
    {
        const Eigen::Vector3d unitGroundNormal_World =
            groundNormal_World_in / groundNormalNorm;
        const Eigen::Vector3d horizontalCandidate_World =
            unitGroundNormal_World.cross(equation1.head<3>());
        const double horizontalNorm = horizontalCandidate_World.norm();
        if (std::isfinite(horizontalNorm) && horizontalNorm > 1e-3)
        {
            axisU_World = horizontalCandidate_World / horizontalNorm;
            axisV_World = axisU_World.cross(equation1.head<3>()).normalized();
        }
    }
    if (axisU_World.squaredNorm() < 0.5 || axisV_World.squaredNorm() < 0.5)
    {
        axisU_World = equation1.head<3>().unitOrthogonal().normalized();
        axisV_World = equation1.head<3>().cross(axisU_World).normalized();
    }

    double minU1 = 0.0, maxU1 = 0.0, minV1 = 0.0, maxV1 = 0.0;
    double minU2 = 0.0, maxU2 = 0.0, minV2 = 0.0, maxV2 = 0.0;
    if (!projectPlaneFootprintOntoSharedAxes(p_first_in,
                                             axisU_World,
                                             axisV_World,
                                             minU1,
                                             maxU1,
                                             minV1,
                                             maxV1) ||
        !projectPlaneFootprintOntoSharedAxes(p_second_in,
                                             axisU_World,
                                             axisV_World,
                                             minU2,
                                             maxU2,
                                             minV2,
                                             maxV2))
    {
        return false;
    }

    const double overlapU_m = std::min(maxU1, maxU2) - std::max(minU1, minU2);
    const double overlapV_m = std::min(maxV1, maxV2) - std::max(minV1, minV2);

    if (overlapU_m <= 0.0 || overlapV_m <= 0.0)
    {
        return false;
    }

    const double overlapArea_m2 = overlapU_m * overlapV_m;
    const double area1_m2       = (maxU1 - minU1) * (maxV1 - minV1);
    const double area2_m2       = (maxU2 - minU2) * (maxV2 - minV2);
    const double smallerArea_m2 = std::min(area1_m2, area2_m2);

    if (!std::isfinite(smallerArea_m2) || smallerArea_m2 < 1e-6)
    {
        return false;
    }

    return (overlapArea_m2 / smallerArea_m2) >= minimumOverlapRatio_in;
}

} // namespace

SemanticsManager::SemanticsManager(Atlas *pAtlas)
{
    /* Store the address of the atlas map */
    mpAtlas = pAtlas;

    /* Get the system parameters */
    sysParams = SystemParams::GetParams();

    /* Configure the room-tracking state machine (WP13 Section 18.4). */
    RoomTrackerConfig trackerConfig;
    trackerConfig.crossing_dwell_s =
        static_cast<double>(sysParams->room_tracking.crossing_dwell_s);
    trackerConfig.crossing_confidence =
        static_cast<double>(sysParams->room_tracking.crossing_confidence);
    trackerConfig.lost_timeout_s =
        static_cast<double>(sysParams->room_tracking.lost_timeout_s);
    trackerConfig.reacquire_timeout_s =
        static_cast<double>(sysParams->room_tracking.reacquire_timeout_s);
    trackerConfig.reacquire_retry_interval_s = static_cast<double>(
        sysParams->room_tracking.reacquire_retry_interval_s);
    trackerConfig.reacquire_max_retries =
        sysParams->room_tracking.reacquire_max_retries;
    trackerConfig.reacquire_min_planes =
        sysParams->room_tracking.reacquire_min_planes;
    roomTracker_ = RoomTracker(trackerConfig);
}

void SemanticsManager::resetTemporalStateForMap(Map *p_activeMap_in)
{
    const std::uint64_t worldFrameEpoch =
        p_activeMap_in != nullptr ? p_activeMap_in->GetWorldFrameEpoch() : 0U;
    const bool mapChanged   = pTemporalStateMap_ != p_activeMap_in;
    const bool frameChanged = !mapChanged && p_activeMap_in != nullptr &&
                              temporalStateWorldFrameEpoch_ != worldFrameEpoch;

    if (!mapChanged && !frameChanged)
    {
        return;
    }

    openPassageEvidence_.clear();
    lastSkeletonFingerprint_ = 0U;
    hasSkeletonFingerprint_  = false;

    if (mapChanged)
    {
        {
            std::lock_guard<std::mutex> currentRoomLock(mMutexCurrentRoom);
            currentRoomId_ = -1;
        }
        currentCameraCenter_World_m  = Eigen::Vector3d::Zero();
        previousCameraCenter_World_m = Eigen::Vector3d::Zero();
        hasCameraCenter_             = false;
        pCameraCenterMap_            = p_activeMap_in;
        lastTraversalFrameId_        = 0U;
        lastTraversalKeyFrameId_     = 0U;
        hasTraversalKeyFrameCursor_  = false;

        disconnectedRoomIds_.clear();
        prospectiveRoomCycles_.clear();
        undefendedWalls_.clear();
        loggedOrphanWallIds_.clear();
        loggedWallRejectionReasons_.clear();
        loggedRetiredWallIds_.clear();
        loggedRoomCleanupIds_.clear();
    }
    else if (frameChanged && hasTraversalKeyFrameCursor_)
    {
        /* Re-read the cursor keyframe in the rebased frame. Keeping its IDs
         * preserves every later, not-yet-processed trajectory segment. */
        hasCameraCenter_ = false;
        for (KeyFrame *p_keyFrame : p_activeMap_in->GetAllKeyFrames())
        {
            if (p_keyFrame == nullptr || p_keyFrame->isBad() ||
                p_keyFrame->mnFrameId != lastTraversalFrameId_ ||
                p_keyFrame->mnId != lastTraversalKeyFrameId_)
            {
                continue;
            }

            const Eigen::Vector3d correctedCenter_World_m =
                p_keyFrame->GetCameraCenter().cast<double>();
            if (correctedCenter_World_m.allFinite())
            {
                currentCameraCenter_World_m  = correctedCenter_World_m;
                previousCameraCenter_World_m = correctedCenter_World_m;
                hasCameraCenter_             = true;
                pCameraCenterMap_            = p_activeMap_in;
            }
            break;
        }
    }

    pTemporalStateMap_            = p_activeMap_in;
    temporalStateWorldFrameEpoch_ = worldFrameEpoch;
}

SemanticsManager::ActiveMapBootstrapResult
    SemanticsManager::ensureActiveMapBootstrapHierarchy(
        const std::optional<Eigen::Vector3d> &cameraPositionOverride_World_m_in)
{
    Map *p_activeMap = mpAtlas != nullptr ? mpAtlas->GetCurrentMap() : nullptr;
    if (p_activeMap == nullptr)
    {
        std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                     "\"reason\":\"NO_ACTIVE_MAP\",\"semantic_cycle\":"
                  << pipelineSemanticCycle_ << "}" << std::endl;
        return ActiveMapBootstrapResult::NO_ACTIVE_MAP;
    }

    const std::vector<Room *> activeRooms = p_activeMap->GetAllRooms();
    const auto                resolveLiveRoomById =
        [&activeRooms](const int roomId_in) -> Room *
    {
        if (roomId_in < 0)
        {
            return nullptr;
        }
        for (Room *p_room : activeRooms)
        {
            if (p_room != nullptr && !p_room->isBad() &&
                p_room->getRoomVariant() == Room::roomVariant::ROOM &&
                p_room->getId() == roomId_in)
            {
                return p_room;
            }
        }
        return nullptr;
    };

    int currentRoomId = -1;
    {
        std::lock_guard<std::mutex> currentRoomLock(mMutexCurrentRoom);
        currentRoomId = currentRoomId_;
    }
    const int recoveryRoomId = mpAtlas->getCurrentSemanticRoomIdentity();

    Room *p_bootstrapRoom = resolveLiveRoomById(currentRoomId);
    if (p_bootstrapRoom == nullptr)
    {
        p_bootstrapRoom = resolveLiveRoomById(recoveryRoomId);
    }
    /* When a recovery identity exists (tracking-loss reset), never fall back
     * to an arbitrary lowest-ID live room: a spurious free-space SE# created
     * during the reset transient would otherwise hijack `currentRoomId_` away
     * from the last-known hierarchy. The lowest-ID seed applies to cold start
     * only (no recovery identity). */
    if (p_bootstrapRoom == nullptr && recoveryRoomId < 0)
    {
        for (Room *p_room : activeRooms)
        {
            if (p_room != nullptr && !p_room->isBad() &&
                p_room->getRoomVariant() == Room::roomVariant::ROOM &&
                (p_bootstrapRoom == nullptr ||
                 p_room->getId() < p_bootstrapRoom->getId()))
            {
                p_bootstrapRoom = p_room;
            }
        }
    }

    const std::optional<RoomContextSnapshot> recoveryContext =
        p_bootstrapRoom == nullptr && recoveryRoomId >= 0
            ? mpAtlas->copyLatestRoomContext(recoveryRoomId)
            : std::nullopt;

    Eigen::Vector3d cameraPosition_World_m  = Eigen::Vector3d::Zero();
    bool            hasUsableCameraPosition = false;
    if (cameraPositionOverride_World_m_in.has_value() &&
        cameraPositionOverride_World_m_in->allFinite())
    {
        cameraPosition_World_m  = *cameraPositionOverride_World_m_in;
        hasUsableCameraPosition = true;
    }
    else
    {
        std::vector<KeyFrame *> keyFrames = p_activeMap->GetAllKeyFrames();
        std::sort(keyFrames.begin(), keyFrames.end(), KeyFrame::lId);
        for (std::vector<KeyFrame *>::reverse_iterator keyFrameIterator =
                 keyFrames.rbegin();
             keyFrameIterator != keyFrames.rend();
             ++keyFrameIterator)
        {
            KeyFrame *p_keyFrame = *keyFrameIterator;
            if (p_keyFrame == nullptr || p_keyFrame->isBad())
            {
                continue;
            }
            const Eigen::Vector3d candidatePosition_World_m =
                p_keyFrame->GetCameraCenter().cast<double>();
            /* An exactly-zero center marks an uninitialized first-frame pose
             * (live-observed: brand-new map, identity pose, room planted at
             * the origin), never a genuine measurement: real computed centers
             * carry rotation/translation noise. Accepting it misplaces the
             * bootstrap room and poisons centroid-distance matching for the
             * cycles until walls correct it. Fall through to the snapshot
             * centroid, else yield and retry once poses exist. */
            if (candidatePosition_World_m.allFinite() &&
                !candidatePosition_World_m.isZero())
            {
                cameraPosition_World_m  = candidatePosition_World_m;
                hasUsableCameraPosition = true;
                break;
            }
        }
    }

    if (p_bootstrapRoom == nullptr && !hasUsableCameraPosition)
    {
        /* Same-map reset clears keyframes, so no camera pose exists yet while
         * a valid recovery snapshot does. Recreate the last-known hierarchy
         * at the snapshot centroid now (refined once keyframes return) rather
         * than yielding the cycle to a free-space SE# with a fresh ID. */
        if (recoveryContext.has_value() &&
            recoveryContext->centroid.allFinite())
        {
            cameraPosition_World_m  = recoveryContext->centroid;
            hasUsableCameraPosition = true;
        }
        else
        {
            std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                         "\"map_id\":"
                      << p_activeMap->GetId()
                      << ",\"reason\":\"NO_USABLE_CAMERA_POSE\","
                         "\"semantic_cycle\":"
                      << pipelineSemanticCycle_ << "}" << std::endl;
            return ActiveMapBootstrapResult::NO_USABLE_CAMERA_POSE;
        }
    }

    bool initializedRoom = false;
    bool recoveredRoom   = false;
    if (p_bootstrapRoom == nullptr)
    {
        p_bootstrapRoom = GeoSemHelpers::createBlankRoomCandidate(
            mpAtlas,
            cameraPosition_World_m,
            recoveryContext.has_value()
                ? std::optional<int>(recoveryContext->roomId)
                : std::nullopt);
        if (p_bootstrapRoom == nullptr)
        {
            std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                         "\"map_id\":"
                      << p_activeMap->GetId()
                      << ",\"reason\":\"ROOM_CREATION_FAILED\","
                         "\"semantic_cycle\":"
                      << pipelineSemanticCycle_ << "}" << std::endl;
            return ActiveMapBootstrapResult::ROOM_CREATION_FAILED;
        }
        mpAtlas->AddCandidateMapRoom(p_bootstrapRoom);
        p_activeMap->PromoteCandidateMapRoom(p_bootstrapRoom);
        p_bootstrapRoom->setRoomVariant(Room::roomVariant::ROOM);
        p_bootstrapRoom->setName("Room#" +
                                 std::to_string(p_bootstrapRoom->getId()));
        p_bootstrapRoom->setBoundaryStatus(Room::BoundaryStatus::UNOBSERVED);
        p_bootstrapRoom->setRoomTag(
            recoveryContext.has_value() && !recoveryContext->roomTag.empty()
                ? recoveryContext->roomTag
                : "room_" + std::to_string(p_bootstrapRoom->getId()));
        p_bootstrapRoom->setRecoveryProxy(recoveryContext.has_value());
        if (recoveryContext.has_value())
        {
            p_bootstrapRoom->setPreviouslyVisited(
                recoveryContext->wasPreviouslyVisited);
        }
        initializedRoom = !recoveryContext.has_value();
        recoveredRoom   = recoveryContext.has_value();
    }

    std::vector<Floor *> floors = p_activeMap->GetAllFloors();
    Floor *p_canonicalFloor     = Floor::selectBestObservedFloor(floors);
    if (p_canonicalFloor == nullptr)
    {
        const std::optional<int> recoveryFloorId =
            recoveryContext.has_value() && recoveryContext->floorId >= 0
                ? std::optional<int>(recoveryContext->floorId)
                : std::nullopt;
        GeoSemHelpers::createMapFloor(mpAtlas, recoveryFloorId);
        floors           = p_activeMap->GetAllFloors();
        p_canonicalFloor = Floor::selectBestObservedFloor(floors);
    }
    if (p_canonicalFloor == nullptr)
    {
        std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                     "\"map_id\":"
                  << p_activeMap->GetId()
                  << ",\"reason\":\"FLOOR_CREATION_FAILED\","
                     "\"room_id\":"
                  << p_bootstrapRoom->getId()
                  << ",\"semantic_cycle\":" << pipelineSemanticCycle_ << "}"
                  << std::endl;
        return ActiveMapBootstrapResult::FLOOR_CREATION_FAILED;
    }

    p_canonicalFloor->addRoom(p_bootstrapRoom);
    if (resolveLiveRoomById(currentRoomId) == nullptr)
    {
        {
            std::lock_guard<std::mutex> currentRoomLock(mMutexCurrentRoom);
            currentRoomId_ = p_bootstrapRoom->getId();
            mpAtlas->setCurrentSemanticRoomIdentity(p_bootstrapRoom->getId());
        }
        /* The UAV starts inside the bootstrap room: presence evidences entry.
         * Marked outside the current-room lock; the room owns its mutex. */
        p_bootstrapRoom->setPreviouslyVisited(true);
        /* Mission-chain trace: the room this map started with. Set once;
         * later bootstrap cycles must not overwrite it. */
        if (p_activeMap->getStartingRoom() == nullptr)
        {
            p_activeMap->setStartingRoom(p_bootstrapRoom);
        }
    }

    std::size_t restoredPassageCount = 0U;
    if (recoveredRoom)
    {
        for (const PassageContext &passageContext :
             recoveryContext->passageContexts)
        {
            Passage *p_recoveryPassage =
                p_activeMap->GetPassageById(passageContext.id);
            if (p_recoveryPassage == nullptr)
            {
                /* New object, stable ID: position, orientation, and aperture
                 * dimensions are not knowable across a map break, so only
                 * frame-free state (passable, traversal history, live links
                 * below) is restored. A zero-sized aperture shrinks geometric
                 * tests to their margin sliver, as before this change. */
                p_recoveryPassage = new Passage();
                p_recoveryPassage->setId(passageContext.id);
                p_recoveryPassage->setMap(p_activeMap);
                p_recoveryPassage->setPassable(passageContext.passable);
                p_recoveryPassage->setPassageType(
                    Passage::passageVariant::DOORWAY);
                p_recoveryPassage->setRecoveryProxy(true);
                for (std::size_t observationIndex = 0U;
                     observationIndex < passageContext.traversalKnownToFarCount;
                     ++observationIndex)
                {
                    p_recoveryPassage->addTraversalObservation(
                        Passage::TraversalDirection::KNOWN_TO_FAR);
                }
                for (std::size_t observationIndex = 0U;
                     observationIndex < passageContext.traversalFarToKnownCount;
                     ++observationIndex)
                {
                    p_recoveryPassage->addTraversalObservation(
                        Passage::TraversalDirection::FAR_TO_KNOWN);
                }
                for (std::size_t observationIndex = 0U;
                     observationIndex < passageContext.traversalUnknownCount;
                     ++observationIndex)
                {
                    p_recoveryPassage->addTraversalObservation(
                        Passage::TraversalDirection::UNKNOWN);
                }
                mpAtlas->AddMapPassage(p_recoveryPassage);
            }

            if (passageContext.hasKnownSideRoom &&
                passageContext.knownSideRoomId == p_bootstrapRoom->getId())
            {
                p_recoveryPassage->setKnownSideRoom(p_bootstrapRoom);
            }
            if (passageContext.hasFarSideRoom &&
                passageContext.secondaryRoomId == p_bootstrapRoom->getId())
            {
                p_recoveryPassage->setProspectiveRoom(p_bootstrapRoom);
            }
            if (p_recoveryPassage->getKnownSideProvenance().pRoom == nullptr &&
                p_recoveryPassage->getProspectiveRoom() == nullptr)
            {
                p_recoveryPassage->setKnownSideRoom(p_bootstrapRoom);
            }
            p_bootstrapRoom->setDoorways(p_recoveryPassage);
            ++restoredPassageCount;
        }
    }

    if (initializedRoom)
    {
        std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                     "\"map_id\":"
                  << p_activeMap->GetId()
                  << ",\"reason\":\"BOOTSTRAP_CREATED\",\"room_id\":"
                  << p_bootstrapRoom->getId()
                  << ",\"floor_id\":" << p_canonicalFloor->getId()
                  << ",\"semantic_cycle\":" << pipelineSemanticCycle_ << "}"
                  << std::endl;
        return ActiveMapBootstrapResult::INITIALIZED;
    }

    if (recoveredRoom)
    {
        std::cout << "SG_PIPELINE {\"event\":\"initialization\","
                     "\"map_id\":"
                  << p_activeMap->GetId()
                  << ",\"reason\":\"RECOVERY_RESTORED\",\"room_id\":"
                  << p_bootstrapRoom->getId()
                  << ",\"floor_id\":" << p_canonicalFloor->getId()
                  << ",\"restored_passages\":" << restoredPassageCount
                  << ",\"semantic_cycle\":" << pipelineSemanticCycle_ << "}"
                  << std::endl;
        return ActiveMapBootstrapResult::RECOVERED;
    }

    return ActiveMapBootstrapResult::REUSED;
}

void SemanticsManager::Run(void)
{
    std::size_t summaryCycle = 0U;

    while (true)
    {
        /* Graceful shutdown on System::Shutdown() */
        if (CheckFinish())
        {
            break;
        }

        /* Find the current time of the loop */
        const std::chrono::steady_clock::time_point start =
            std::chrono::steady_clock::now();

        /*!
         * Treat one hierarchy update as an atomic semantic transaction. Map
         * merging takes the same atlas-owned lock before changing frames or
         * ownership, so this cycle can never traverse a half-merged graph.
         */
        std::unique_lock<std::mutex> semanticUpdateLock =
            mpAtlas->acquireSemanticUpdateLock();

        pipelineSemanticCycle_ = ++summaryCycle;
        resetTemporalStateForMap(mpAtlas->GetCurrentMap());
        ensureActiveMapBootstrapHierarchy();

        /* Validate the low-level semantic planes */
        Plane *mainGroundPlane = mpAtlas->GetBiggestGroundPlane();

        /* If there is a ground plane, find its transform and filter planes */
        if (mainGroundPlane != nullptr)
        {
            /* Find the transform from ground plane to horizontal */
            mPlanePoseMat = computePlaneToHorizontal(mainGroundPlane);

            /* Filter ground planes */
            filterGroundPlanes(mainGroundPlane);

            /* Filter the wall planes */
            filterWallPlanes();
        }

        /*!
         * Reconcile duplicate semantic planes before constructing room edges.
         *
         * Plane hypotheses are created from individual keyframe observations,
         * so one physical wall can initially exist as several overlapping or
         * contiguous fragments. Associating those fragments with a room first
         * creates duplicate graph edges and provisional structural elements.
         * The reconciliation pass validates equation, finite extent, and
         * observation-side compatibility before atomically rewiring every
         * existing reference to the retained plane.
         */
        if (sysParams->sem_seg.reassociate.enabled)
        {
            Utils::reAssociateSemanticPlanes(mpAtlas);
        }

        /*  Detect/update passage GEOMETRY before any wall-to-room admission
         * this cycle, so admitWallToRoom()'s far-side-passage backstop (and
         * Pass 1's free-space clustering below) can route a wall to the
         * correct side of a doorway the moment that doorway itself becomes
         * observable, instead of only on the NEXT cycle once a stale
         * passage list catches up. detectDoorsAndDoorways()/updatePassages()
         * only read Plane/Passage data (no Room dependency), so this is safe
         * to run ahead of any room detection or association below. Passage
         * steps that DO need current room membership (traversal evidence,
         * room association, wall detachment, prospective-room cleanup) stay
         * below, after Pass 1's wall admission, where room data exists to
         * work from. */
        if (sysParams->sem_seg.enable_passage_detection)
        {
            detectDoorsAndDoorways(mpAtlas);
            updatePassages(mpAtlas);
            mergeOverlappingPassages();
        }

        /*!
         * Use free-space evidence to create and update rooms.
         *
         * @note         This is the preferred wall-to-room association method.
         */
        if (sysParams->room_seg.method ==
            SystemParams::room_seg::Method::FREE_SPACE)
        {
            detectRoom_FreeSpaceCluster();
        }

        /*!
         * Enforce the semantic hierarchy.
         *
         * @note        wall which was not captured by the free-space room
         *              detector receives either an existing room or a new
         *              provisional structural element.
         */
        associateAllWallsToRooms();

        /* Consolidate only redundant single-wall provisional structures. */
        Utils::reAssociateRooms(mpAtlas);

        /*  Room-dependent passage steps: geometry was already refreshed
         * above, ahead of this cycle's wall admission. */
        if (sysParams->sem_seg.enable_passage_detection)
        {
            updateTraversalEvidence(mpAtlas);
            Utils::reAssociatePassages(mpAtlas);
            associatePassagesToRooms();
            detachWallsBeyondConfirmedPassages();

            /* Continuous rule-invariant sweep (not just at admission time):
             * see enforcePassageSideInvariant()'s own comment. Room<->passage
             * association is current as of the two calls just above. */
            enforcePassageSideInvariant();

            /* PROSPECTIVE ROOM CLEANUP
             * A passage reference is the stable far-side handle. Zero-wall
             * prospectives remain alive while that reference and its geometry
             * are valid; only orphaned or invalid handles are retired. */
            const std::vector<ORB_SLAM3::Room *> candidateRooms =
                mpAtlas->GetAllCandidateMapRooms();
            const std::vector<ORB_SLAM3::Passage *> allPassages =
                mpAtlas->GetAllPassages();
            for (ORB_SLAM3::Room *p_candidate : candidateRooms)
            {
                if (p_candidate == nullptr)
                {
                    continue;
                }

                const int  roomId = p_candidate->getId();
                const bool isTrackedProspective =
                    prospectiveRoomCycles_.count(roomId) > 0U;

                std::vector<ORB_SLAM3::Passage *> referencingPassages;
                for (ORB_SLAM3::Passage *p_passage : allPassages)
                {
                    if (p_passage != nullptr && !p_passage->isBad() &&
                        p_passage->getProspectiveRoom() == p_candidate)
                    {
                        referencingPassages.push_back(p_passage);
                    }
                }

                if (!isTrackedProspective && referencingPassages.empty())
                {
                    continue;
                }

                bool hasValidGeometry =
                    !p_candidate->isBad() &&
                    p_candidate->getCentroid().allFinite() &&
                    p_candidate->getMap() != nullptr &&
                    mpAtlas->isActiveMap(p_candidate->getMap());

                for (ORB_SLAM3::Passage *p_passage : referencingPassages)
                {
                    const Eigen::Vector4d passageEquation_World =
                        p_passage->getGlobalEquation().coeffs();

                    if (!p_passage->getCentroid().allFinite() ||
                        !passageEquation_World.allFinite() ||
                        passageEquation_World.head<3>().norm() < 1e-8)
                    {
                        hasValidGeometry = false;
                        break;
                    }
                }

                if (!p_candidate->isBad() && !referencingPassages.empty() &&
                    hasValidGeometry)
                {
                    /* Wall count and age are deliberately irrelevant here. */
                    prospectiveRoomCycles_[roomId] = 0;
                    continue;
                }

                for (ORB_SLAM3::Passage *p_passage : referencingPassages)
                {
                    p_passage->setProspectiveRoom(nullptr);
                }

                if (!p_candidate->isBad())
                {
                    Map *p_candidateMap = p_candidate->getMap();
                    if (p_candidateMap != nullptr)
                    {
                        p_candidateMap->EraseMarkerBasedMapRoom(p_candidate);
                    }
                    p_candidate->setBad();
                    if (loggedRoomCleanupIds_.insert(roomId).second)
                    {
                        std::cout
                            << "[SemMgr] Cleaning up orphaned prospective Room#"
                            << roomId
                            << " (no active passage reference or invalid "
                               "geometry)."
                            << std::endl;
                    }
                }

                prospectiveRoomCycles_.erase(roomId);
            }
        }

        /* Phase 2: Re-detect rooms from passage-partitioned free-space
         * clusters. Now that passages exist, partitionFreeSpaceAtPassages()
         * will split clusters at doorways, yielding correct per-room clusters.
         */
        if (sysParams->room_seg.method ==
            SystemParams::room_seg::Method::FREE_SPACE)
        {
            detectRoom_FreeSpaceCluster(); // Second pass - updates existing
                                           // rooms
        }

        /* Re-associate walls to rooms after Phase 2 cluster splitting.
         * Walls assigned in Phase 1 may belong to wrong (merged) rooms. */
        associateAllWallsToRooms();

        /* A wall surface is owned by exactly one room in every configuration.
         * Run AFTER Phase 2 so split clusters get correct wall ownership. */
        enforceUniqueWallOwnership();

        /* Link each wall's opposite-facing twin, now that ownership has
         * settled for this cycle. Must run before validateRoomBoundaries()
         * so boundary/corner logic can rely on current twin identity. */
        reconcileWallFacePairs();

        /* Validate room geometry without delaying independent passage data. */
        validateRoomBoundaries();

        /* Retire only stale, weak wall hypotheses left unused by the graph. */
        suppressUndefendedWalls();

        /* Recompute room centroids as mean of associated wall centroids.
         * In FREE_SPACE mode the skeleton-cluster centroid set during room
         * detection is the authoritative room centre; overwriting it with the
         * wall-centroid mean drifts rooms away from the cluster each cycle,
         * which breaks the centroid-keyed room matching in associateRooms()
         * and spawns a duplicate room every run. */
        if (sysParams->room_seg.method !=
            SystemParams::room_seg::Method::FREE_SPACE)
        {
            recomputeRoomCentroidsFromWalls();
        }

        /* Associate every valid room/SE with the floor */
        getUpdatedFloors();

        /* Re-point any room whose own ground plane disagrees with the
         * just-refreshed canonical Floor identity. */
        reconcileRoomGroundPlanes();

        /* Room candidate generation is pre-verification only. The legacy
         * tag-and-wall-transfer entry point remains disabled until P4. */

        /* Advance the room-state machine (WP13 Section 18.2). It consumes the
         * traversal crossings recorded above and reports accepted/rejected
         * transitions. It is read-only with respect to the room id members. */
        const std::chrono::duration<double> roomTrackerElapsed =
            std::chrono::steady_clock::now().time_since_epoch();
        updateRoomTrackerState(roomTrackerElapsed.count());

        /* P4 verification and the shared merge seam are not enabled in P1-P3.
         * In particular, a tag match must never activate MergeMapPair(). */
        std::map<long unsigned int, std::vector<RoomContextSnapshot>>
             copiedContext  = mpAtlas->copyRoomContextHistory();
        Map *p_candidateMap = mpAtlas->GetCurrentMap();
        if (p_candidateMap != nullptr)
        {
            const Atlas::SnapshotCopyResult currentSnapshot =
                mpAtlas->copyRoomContextForMapChecked(p_candidateMap, true);
            if (currentSnapshot.status == Atlas::SnapshotCopyStatus::COMPLETE)
            {
                copiedContext[p_candidateMap->GetId()] =
                    currentSnapshot.snapshots;
            }
        }
        SemanticCandidateConfig candidateConfig;
        candidateConfig.topK = sysParams->candidate_gen.top_k;
        candidateConfig.candidatePairCap =
            sysParams->candidate_gen.candidate_pair_cap;
        candidateConfig.topologyNodesCap =
            sysParams->candidate_gen.topology_nodes_cap;
        candidateConfig.globalFallbackCap =
            sysParams->candidate_gen.global_fallback_cap;
        candidateConfig.weightAngle  = sysParams->candidate_gen.weight_angle;
        candidateConfig.weightExtent = sysParams->candidate_gen.weight_extent;
        candidateConfig.weightAperture =
            sysParams->candidate_gen.weight_aperture;
        candidateConfig.weightTopology =
            sysParams->candidate_gen.weight_topology;
        candidateConfig.angleMissingPenalty =
            sysParams->candidate_gen.angle_missing_penalty;
        candidateConfig.extentMissingPenalty =
            sysParams->candidate_gen.extent_missing_penalty;
        candidateConfig.apertureMissingPenalty =
            sysParams->candidate_gen.aperture_missing_penalty;
        candidateConfig.ambiguityMargin =
            sysParams->candidate_gen.ambiguity_margin;
        candidateConfig.angleTolerance_rad =
            sysParams->candidate_gen.angle_tolerance_rad;
        candidateConfig.runtimeBudget_ms =
            sysParams->candidate_gen.runtime_budget_ms;
        candidateConfig.descriptorElementsCap =
            sysParams->candidate_gen.descriptor_elements_cap;
        candidateConfig.topoRefinementIters =
            sysParams->candidate_gen.topo_refinement_iters;
        /* Section 9.2's "last-confirmed room" anchor for adjacency-
         * prioritised candidate search. -1 (unset) maps to no anchor. */
        const int                lastKnownRoomId = getLastKnownRoomId();
        const std::optional<int> anchorRoomId =
            lastKnownRoomId >= 0 ? std::optional<int>(lastKnownRoomId)
                                 : std::nullopt;
        const std::vector<SemanticCandidate> candidates =
            SemanticCandidates::generate(copiedContext,
                                         candidateConfig,
                                         anchorRoomId);
        std::cout << "[SemMgr] semantic_candidates count=" << candidates.size()
                  << std::endl;

        /* Milestone 1 (WP1-master-plan.md, Part 4): run the Phase 4 verifier
         * on the single best candidate and feed a real VerificationVerdict to
         * roomTracker_ via submitVerificationVerdict(). This still only makes
         * the *verdict* real -- it must not call Atlas::MergeMapPair() or
         * otherwise mutate the Atlas; that trigger is Milestone 3's, gated
         * behind the real-data verifier audit of Milestone 2. */
        evaluateTopCandidateVerification(candidates);

        /* Compact Phase-1 heartbeat: all values come from this completed
         * semantic transaction and are therefore mutually consistent. */
        Map                       *p_pipelineMap = mpAtlas->GetCurrentMap();
        const std::vector<Plane *> pipelinePlanes =
            p_pipelineMap != nullptr ? p_pipelineMap->GetAllPlanes()
                                     : std::vector<Plane *>();
        const std::vector<Room *> pipelineRooms =
            p_pipelineMap != nullptr ? p_pipelineMap->GetAllRooms()
                                     : std::vector<Room *>();
        const std::vector<Passage *> pipelinePassages =
            p_pipelineMap != nullptr ? p_pipelineMap->GetAllPassages()
                                     : std::vector<Passage *>();
        const std::vector<Floor *> pipelineFloors =
            p_pipelineMap != nullptr ? p_pipelineMap->GetAllFloors()
                                     : std::vector<Floor *>();
        const std::vector<std::vector<Eigen::Vector3d>> pipelineClusters =
            p_pipelineMap != nullptr
                ? p_pipelineMap->GetSkeletonClusterPoints()
                : std::vector<std::vector<Eigen::Vector3d>>();

        std::size_t             wallClassCount      = 0U;
        std::size_t             admissibleWallCount = 0U;
        std::size_t             ownedWallCount      = 0U;
        std::unordered_set<int> ownedWallIds;
        Plane                  *p_pipelineGround   = p_pipelineMap != nullptr
                                                         ? p_pipelineMap->GetBiggestGroundPlane()
                                                         : nullptr;
        Eigen::Vector3d pipelineGroundNormal_World = Eigen::Vector3d::Zero();
        if (p_pipelineGround != nullptr && !p_pipelineGround->isBad())
        {
            const Eigen::Vector4d equation =
                p_pipelineGround->getGlobalEquation().coeffs();
            if (equation.allFinite() && equation.head<3>().norm() > 1e-8)
            {
                pipelineGroundNormal_World = equation.head<3>().normalized();
            }
        }
        for (Plane *p_plane : pipelinePlanes)
        {
            if (p_plane == nullptr || p_plane->isBad() ||
                p_plane->getPlaneType() != Plane::planeVariant::WALL)
            {
                continue;
            }
            wallClassCount++;
            if (evaluateWallAdmissionEvidence(p_plane,
                                              sysParams,
                                              pipelineGroundNormal_World)
                    .admissible)
            {
                admissibleWallCount++;
            }
        }
        std::size_t realRoomCount        = 0U;
        std::size_t prospectiveRoomCount = 0U;
        for (Room *p_room : pipelineRooms)
        {
            if (p_room == nullptr || p_room->isBad())
            {
                continue;
            }
            if (p_room->getRoomVariant() == Room::roomVariant::ROOM)
            {
                realRoomCount++;
            }
            else
            {
                prospectiveRoomCount++;
            }
            for (Plane *p_wall : p_room->getWalls())
            {
                if (p_wall != nullptr && !p_wall->isBad())
                {
                    ownedWallIds.insert(p_wall->getId());
                }
            }
        }
        ownedWallCount                  = ownedWallIds.size();
        std::size_t skeletonVertexCount = 0U;
        for (const std::vector<Eigen::Vector3d> &cluster : pipelineClusters)
        {
            skeletonVertexCount += cluster.size();
        }
        const std::size_t pendingWallCount = std::count_if(
            undefendedWalls_.begin(),
            undefendedWalls_.end(),
            [&ownedWallIds](
                const std::pair<const int, UndefendedWallState> &entry)
            { return ownedWallIds.count(entry.first) == 0U; });
        const std::size_t livePassageCount = std::count_if(
            pipelinePassages.begin(),
            pipelinePassages.end(),
            [](Passage *p_passage)
            { return p_passage != nullptr && !p_passage->isBad(); });
        std::cout << "SG_PIPELINE {\"event\":\"heartbeat\",\"map_id\":"
                  << (p_pipelineMap != nullptr
                          ? static_cast<long long>(p_pipelineMap->GetId())
                          : -1)
                  << ",\"semantic_cycle\":" << pipelineSemanticCycle_
                  << ",\"current_room_id\":" << getCurrentRoomId()
                  << ",\"raw_planes\":" << pipelinePlanes.size()
                  << ",\"wall_class_planes\":" << wallClassCount
                  << ",\"admissible_walls\":" << admissibleWallCount
                  << ",\"pending_walls\":" << pendingWallCount
                  << ",\"owned_walls\":" << ownedWallCount
                  << ",\"skeleton_clusters\":" << pipelineClusters.size()
                  << ",\"skeleton_vertices\":" << skeletonVertexCount
                  << ",\"skeleton_edges\":"
                  << (p_pipelineMap != nullptr
                          ? p_pipelineMap->GetSkeletonEdges().size()
                          : 0U)
                  << ",\"real_rooms\":" << realRoomCount
                  << ",\"prospective_rooms\":" << prospectiveRoomCount
                  << ",\"passages\":" << livePassageCount
                  << ",\"floors\":" << pipelineFloors.size() << "}"
                  << std::endl;

        /* ------------------------------------------------------------------ *
         * SEMANTIC MONITOR BOUNDARY (P1.4/P1.7/P1.8, semantic-axiom-
         * reliability-plan.md): capture a complete, pointer-free snapshot
         * plus manager-private evidence while the semantic-update lock is
         * still held, unlock, then evaluate/digest/cache/log outside the
         * lock. Read-only with respect to inference, ownership, passage,
         * room, and completeness decisions -- this never mutates Atlas/Map/
         * Room/Wall/Passage state.
         * ------------------------------------------------------------------ */
        const std::uint64_t semanticCycle = pipelineSemanticCycle_;

        semantic::SemanticGraphSnapshot snapshot =
            semantic::captureSemanticGraphSnapshot(mpAtlas);
        snapshot.managerPrivateOpenPassageHypotheses =
            captureOpenPassageHypotheses();
        snapshot.managerPrivateOpenPassageHypothesesReason =
            semantic::UnavailableReason::NONE;
        snapshot.managerPrivateUnresolvedWallHypotheses =
            captureUnresolvedWallHypotheses();
        snapshot.managerPrivateUnresolvedWallHypothesesReason =
            semantic::UnavailableReason::NONE;

        std::optional<int> currentMapRevision;
        if (snapshot.currentMapId.has_value())
        {
            Map *p_currentMap = mpAtlas->GetCurrentMap();
            if (p_currentMap != nullptr &&
                p_currentMap->GetId() == *snapshot.currentMapId)
            {
                currentMapRevision = p_currentMap->GetMapChangeIndex();
            }
        }

        /* Unlock before any evaluation/serialization/caching/logging work --
         * the monitor must never hold the semantic-update lock while doing
         * read-only diagnostic work. */
        /* Continuous consecutive-map matching, old into current, inside
         * this transaction: at most one merge per cycle; attempts and
         * commits log via SG_PIPELINE. */
        mpAtlas->attemptConsecutiveMergeIfGated();
        semanticUpdateLock.unlock();

        const std::chrono::steady_clock::time_point evaluationStart =
            std::chrono::steady_clock::now();
        const semantic::AxiomEvaluationReport evaluationReport =
            semantic::evaluateState(snapshot);
        const std::vector<semantic::MapCompletenessResult> completenessResults =
            semantic::evaluateMapCompleteness(snapshot);
        const std::string topologyDigest = semantic::sha256HexDigest(
            semantic::serializeSnapshotTopologyOnly(snapshot).dump());
        const std::string fullGeometryDigest = semantic::sha256HexDigest(
            semantic::serializeSnapshotFullGeometry(snapshot).dump());
        const std::chrono::milliseconds evaluationDuration =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - evaluationStart);

        mSemanticReportCache.update(snapshot,
                                    evaluationReport,
                                    completenessResults,
                                    semanticCycle,
                                    snapshot.currentMapId,
                                    currentMapRevision,
                                    topologyDigest,
                                    fullGeometryDigest,
                                    evaluationDuration);

        logSemanticDiagnostics(mSemanticReportCache.getLatest());

        /* Find the time after it took to run the loop */
        const std::chrono::steady_clock::time_point end =
            std::chrono::steady_clock::now();

        /* Calculate the elapsed time */
        const std::chrono::duration<double> elapsed = end - start;

        /* Find how much longer in the loop is left */
        const double remainingSeconds = runInterval_s - elapsed.count();

        /* If there is remaining time, sleep until next loop cycle */
        if (remainingSeconds > 0.0)
        {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(remainingSeconds));
        }
        else
        {
            /* Let a waiting merge or segmentation transaction acquire next. */
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    /* Signal shutdown completion to ~System. */
    SetFinish();
}

std::vector<std::vector<Eigen::Vector3d>>
    SemanticsManager::getLatestSkeletonCluster(void)
{
    /* Lock the skeleton cluster */
    unique_lock<std::mutex> lock(mMutexNewRooms);

    /* Get the latest skeleton cluster from Atlas */
    return mpAtlas->GetSkeletoClusterPoints();
}

semantic::SemanticReportCacheEntry
    SemanticsManager::getSemanticReportCacheEntry(void) const
{
    return mSemanticReportCache.getLatest();
}

bool SemanticsManager::isSemanticReportCacheAvailable(void) const
{
    return mSemanticReportCache.isAvailable();
}

void SemanticsManager::filterWallPlanes(void)
{
    /* Iterate through all the planes and filter the walls */
    for (const auto &plane : mpAtlas->GetAllPlanes())
    {
        /* Skip planes which are not classed as walls */
        if (plane->getExpectedPlaneType() ==
            ORB_SLAM3::Plane::planeVariant::WALL)
        {
            /*!
             * Wall validation based on the mPlanePoseMat only works if the
             * ground plane is set. Needs the correction matrix: mPlanePoseMat.
             */
            Eigen::Vector3f transformedPlaneCoefficients =
                transformPlaneEqToGroundReference(
                    plane->getGlobalEquation().coeffs());

            /*!
             * If the transformed plane is vertical based on absolute value,
             * then assign semantic, otherwise ignore threshold should be
             * leniently set (ideally with correct ground plane reference, this
             * value should be close to 0.00)
             */
            if (abs(transformedPlaneCoefficients(1)) >
                sysParams->sem_seg.max_tilt_wall)
            {
                plane->resetPlaneSemantics();
            }
        }
    }
}

void SemanticsManager::filterGroundPlanes(Plane *groundPlane)
{
    /*!
     * Discard gound planes that have a height above a threshold from the
     * biggest ground plane.
     *
     * [TODO] - Should determine if it is better to use the biggest ground plane
     *          or the lowest ground plane.
     */

    /* Get the median height of the plane to compute the threshold */
    std::optional<float> groundPlaneHeight =
        computeGroundPlaneHeight(groundPlane);
    if (!groundPlaneHeight.has_value())
    {
        /* Nothing to filter against yet -- the main ground plane's support
           cloud is momentarily empty (e.g. right after creation/reset). */
        return;
    }
    float threshY = *groundPlaneHeight - sysParams->sem_seg.max_step_elevation;

    /* Extract the main associated ground plane */
    int groundPlaneId = groundPlane->getId();

    /* Go through all ground planes to check validity */
    for (const auto &plane : mpAtlas->GetAllPlanes())
    {
        /* Skip planes not classed as ground, or are the main ground plane */
        if (plane->getExpectedPlaneType() !=
                ORB_SLAM3::Plane::planeVariant::GROUND ||
            plane->getId() == groundPlaneId)
        {
            continue;
        }

        /* If planes above inverted y threshold, then reset plane semantics.
           Skip (don't filter) a plane whose support cloud is momentarily
           empty -- there's nothing to judge its height against yet. */
        std::optional<float> planeHeight = computeGroundPlaneHeight(plane);
        if (!planeHeight.has_value())
        {
            continue;
        }
        if (*planeHeight < threshY)
        {
            plane->resetPlaneSemantics();
            continue;
        }

        /* Find trnsform of the plane */
        Eigen::Vector3f transformedPlaneCoefficients =
            transformPlaneEqToGroundReference(
                plane->getGlobalEquation().coeffs());

        /*!
         * If the transformed plane is horizontal based on absolute value, then
         * assign semantic.
         *
         * Ignore threshold should be lenient. With correct ground plane
         * reference, this value should be close to 0.00.
         */
        if (abs(transformedPlaneCoefficients(0)) >
            sysParams->sem_seg.max_tilt_ground)
        {
            plane->resetPlaneSemantics();
        }
    }
}
void SemanticsManager::detectOpenPassagesFromSkeletonEdges(
    const std::vector<ORB_SLAM3::Plane *> &wallPlanes)
{
    const SystemParams::sem_seg::PassageDetection &passageParameters =
        sysParams->sem_seg.passageDetection;

    const double minimumSideDistance =
        static_cast<double>(passageParameters.minimumSideDistance_m);
    const double minimumEdgeNormalAlignment =
        static_cast<double>(passageParameters.minimumEdgeNormalAlignment);
    const double minimumOpeningRadius =
        static_cast<double>(passageParameters.minimumOpeningRadius_m);
    const double wallBoundsMargin =
        static_cast<double>(passageParameters.wallBoundsMargin_m);
    const double duplicatePassageDistance =
        static_cast<double>(passageParameters.duplicatePassageDistance_m);
    const double duplicateNormalAlignment =
        static_cast<double>(passageParameters.duplicateNormalAlignment);
    const double ambiguousDuplicateNormalAlignment = static_cast<double>(
        passageParameters.ambiguousDuplicateNormalAlignment);
    const double ambiguousDuplicatePlaneSeparation_m = static_cast<double>(
        passageParameters.ambiguousDuplicatePlaneSeparation_m);
    const double crossingClusterDistance =
        static_cast<double>(passageParameters.crossingClusterDistance_m);
    const std::size_t minimumCrossingClusterSize =
        passageParameters.minimumCrossingClusterSize;
    const std::size_t maximumMissedUpdateCount =
        passageParameters.maximumMissedSnapshots;
    const double minimumHorizontalFlankExtent_m =
        static_cast<double>(passageParameters.minimumHorizontalFlankExtent_m);
    const std::size_t minimumHorizontalFlankPointCount =
        passageParameters.minimumHorizontalFlankPointCount;

    /* Reject skeleton crossings at implausible heights */
    constexpr double minimumCrossingHeight = 0.20;
    constexpr double maximumCrossingHeight = 2.30;

    /*!
     * Preferred centre height for an open doorway.
     *
     * The crossing points determine the horizontal position. This height is
     * used when the skeleton crossings do not span enough of the doorway to
     * estimate its vertical centre reliably.
     */
    constexpr double preferredPassageHeight = 1.00;

    /* Restrict the final passage centre to a sensible doorway-centre range */
    constexpr double minimumPassageCentreHeight = 0.75;
    constexpr double maximumPassageCentreHeight = 1.25;

    /*!
     * Use the measured crossing-height midpoint only when the crossings cover
     * a meaningful vertical portion of the doorway.
     */
    constexpr double minimumMeasuredHeightSpan = 0.50;

    /* Extract the latest raw Voxblox sparse-graph edges */
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        skeletonEdges = mpAtlas->GetSkeletonEdges();

    if (skeletonEdges.empty())
    {
        openPassageEvidence_.clear();
        hasSkeletonFingerprint_ = false;
        return;
    }

    /*
     * SemanticsManager runs more frequently than the Voxblox skeletonizer.
     * Fingerprint the quantized topology so one latched ROS message cannot be
     * counted repeatedly as independent temporal evidence.
     */
    std::uint64_t skeletonFingerprint = 1469598103934665603ULL;

    const auto appendFingerprintCoordinate =
        [&skeletonFingerprint](double value)
    {
        const std::int64_t quantizedValue =
            static_cast<std::int64_t>(std::llround(value / 0.05));

        skeletonFingerprint ^= static_cast<std::uint64_t>(quantizedValue);
        skeletonFingerprint *= 1099511628211ULL;
    };

    for (const auto &skeletonEdge : skeletonEdges)
    {
        appendFingerprintCoordinate(skeletonEdge.first.x());
        appendFingerprintCoordinate(skeletonEdge.first.y());
        appendFingerprintCoordinate(skeletonEdge.first.z());
        appendFingerprintCoordinate(skeletonEdge.second.x());
        appendFingerprintCoordinate(skeletonEdge.second.y());
        appendFingerprintCoordinate(skeletonEdge.second.z());
    }

    if (hasSkeletonFingerprint_ &&
        skeletonFingerprint == lastSkeletonFingerprint_)
    {
        return;
    }

    lastSkeletonFingerprint_ = skeletonFingerprint;
    hasSkeletonFingerprint_  = true;

    /* ---------------------------------------------------------------------- *
     * PREPARE THE GROUND PLANE
     * ---------------------------------------------------------------------- */

    ORB_SLAM3::Plane *groundPlane = mpAtlas->GetBiggestGroundPlane();

    Eigen::Vector4d groundEquation = Eigen::Vector4d::Zero();

    Eigen::Vector3d groundNormal = Eigen::Vector3d::Zero();

    bool hasValidGroundEquation = false;

    if (groundPlane != nullptr && !groundPlane->isBad())
    {
        groundEquation = groundPlane->getGlobalEquation().coeffs();

        const double groundNormalNorm = groundEquation.head<3>().norm();

        if (std::isfinite(groundNormalNorm) && groundNormalNorm > 1e-8)
        {
            groundEquation /= groundNormalNorm;

            groundNormal = groundEquation.head<3>();

            hasValidGroundEquation = true;
        }
    }

    /*
     * A passage is a traversable wall opening relative to a floor. Without a
     * stable ground normal there is no reliable horizontal flank or doorway
     * height test, so retaining candidates would turn depth dropouts and
     * unobserved wall ends into permanent passages.
     */
    if (!hasValidGroundEquation)
    {
        openPassageEvidence_.clear();
        hasSkeletonFingerprint_ = false;
        return;
    }

    /* ---------------------------------------------------------------------- *
     * PASSAGE CANDIDATE TYPE
     * ---------------------------------------------------------------------- */

    struct PassageCandidate
    {
        ORB_SLAM3::Plane *wall = nullptr;

        Eigen::Vector3d crossingPoint = Eigen::Vector3d::Zero();

        double      openingRadius = 0.0;
        /** Vertical span of this cycle's crossing cluster, 0 when not
         *  reliably measured (see minimumMeasuredHeightSpan below). Together
         *  with openingRadius, this is the passage size estimate the user
         *  asked for -- previously only door-typed (closed) passages had a
         *  size at all. */
        double      heightSpan_m      = 0.0;
        std::size_t confirmationCount = 0;
        /** Number of individual skeleton-edge crossings clustered into this
         *  opening THIS cycle alone (see crossingClusters below) -- the
         *  same-cycle evidence-quantity signal passage creation is gated on,
         *  the passage-side equivalent of a wall's cluster point count /
         *  connectivity ratio. Not carried across cycles by the temporal
         *  matching below, unlike openingRadius/heightSpan_m: strength must
         *  be re-earned each cycle, exactly like a wall's own admission
         *  evidence. */
        std::size_t crossingCount = 0;
    };

    struct AcceptedCrossing
    {
        Eigen::Vector3d point_World_m   = Eigen::Vector3d::Zero();
        double          openingRadius_m = 0.0;
    };

    std::vector<PassageCandidate> passageCandidates;

    passageCandidates.reserve(wallPlanes.size());

    /* ---------------------------------------------------------------------- *
     * FIND CROSSINGS FOR EACH WALL
     * ---------------------------------------------------------------------- */

    for (ORB_SLAM3::Plane *wall : wallPlanes)
    {
        if (wall == nullptr || wall->isBad())
        {
            continue;
        }

        const Plane::GeometrySnapshot wallGeometry =
            wall->getGeometrySnapshot();
        const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr wallCloud =
            wallGeometry.supportCloud;

        if (wallCloud == nullptr || wallCloud->empty())
        {
            continue;
        }

        /* Extract and normalise the wall equation */
        Eigen::Vector4d wallEquation = wallGeometry.equation_World;

        const double wallNormalNorm = wallEquation.head<3>().norm();

        if (!std::isfinite(wallNormalNorm) || wallNormalNorm < 1e-8)
        {
            continue;
        }

        wallEquation /= wallNormalNorm;

        const Eigen::Vector3d wallNormal = wallEquation.head<3>();

        const Eigen::Vector3d wallCentroid = wallGeometry.centroid_World_m;

        Eigen::Vector3d horizontalWallTangent_World = Eigen::Vector3d::Zero();
        bool            hasHorizontalWallTangent    = false;

        if (hasValidGroundEquation)
        {
            Eigen::Vector3d horizontalWallNormal_World =
                wallNormal - wallNormal.dot(groundNormal) * groundNormal;

            if (horizontalWallNormal_World.norm() > 1e-8)
            {
                horizontalWallNormal_World.normalize();
                horizontalWallTangent_World =
                    groundNormal.cross(horizontalWallNormal_World).normalized();
                hasHorizontalWallTangent =
                    horizontalWallTangent_World.allFinite();
            }
        }

        /*
         * Construct two axes lying inside the wall plane.
         */
        const Eigen::Vector3d wallAxisU =
            wallNormal.unitOrthogonal().normalized();

        const Eigen::Vector3d wallAxisV =
            wallNormal.cross(wallAxisU).normalized();

        double minimumU = std::numeric_limits<double>::max();

        double maximumU = std::numeric_limits<double>::lowest();

        double minimumV = std::numeric_limits<double>::max();

        double maximumV = std::numeric_limits<double>::lowest();

        std::size_t         validWallPointCount = 0;
        std::vector<double> horizontalWallCoordinates_m;
        horizontalWallCoordinates_m.reserve(wallCloud->size());

        /* Calculate the finite wall bounds */
        for (const pcl::PointXYZRGBA &point : wallCloud->points)
        {
            if (!pcl::isFinite(point))
            {
                continue;
            }

            const Eigen::Vector3d wallPoint(static_cast<double>(point.x),
                                            static_cast<double>(point.y),
                                            static_cast<double>(point.z));

            const Eigen::Vector3d relativePoint = wallPoint - wallCentroid;

            const double coordinateU = relativePoint.dot(wallAxisU);

            const double coordinateV = relativePoint.dot(wallAxisV);

            minimumU = std::min(minimumU, coordinateU);

            maximumU = std::max(maximumU, coordinateU);

            minimumV = std::min(minimumV, coordinateV);

            maximumV = std::max(maximumV, coordinateV);

            if (hasHorizontalWallTangent)
            {
                horizontalWallCoordinates_m.push_back(
                    wallPoint.dot(horizontalWallTangent_World));
            }

            validWallPointCount++;
        }

        if (validWallPointCount == 0)
        {
            continue;
        }

        /* Store every accepted skeleton breach through this wall */
        std::vector<AcceptedCrossing> acceptedCrossings;

        acceptedCrossings.reserve(skeletonEdges.size());

        /* Test every skeleton edge against the current wall */
        for (const auto &skeletonEdge : skeletonEdges)
        {
            const Eigen::Vector3d &edgeStart = skeletonEdge.first;

            const Eigen::Vector3d &edgeEnd = skeletonEdge.second;

            if (!edgeStart.allFinite() || !edgeEnd.allFinite())
            {
                continue;
            }

            const Eigen::Vector3d edgeVector = edgeEnd - edgeStart;

            const double edgeLength = edgeVector.norm();

            if (!std::isfinite(edgeLength) || edgeLength < 1e-6)
            {
                continue;
            }

            const Eigen::Vector3d edgeDirection = edgeVector / edgeLength;

            /* Reject edges which merely graze the wall */
            const double edgeNormalAlignment =
                std::abs(edgeDirection.dot(wallNormal));

            if (edgeNormalAlignment < minimumEdgeNormalAlignment)
            {
                continue;
            }

            const double startDistance =
                wallNormal.dot(edgeStart) + wallEquation(3);

            const double endDistance =
                wallNormal.dot(edgeEnd) + wallEquation(3);

            const bool crossesPlane = (startDistance <= -minimumSideDistance &&
                                       endDistance >= minimumSideDistance) ||
                                      (endDistance <= -minimumSideDistance &&
                                       startDistance >= minimumSideDistance);

            if (!crossesPlane)
            {
                continue;
            }

            const double denominator = startDistance - endDistance;

            if (std::abs(denominator) < 1e-8)
            {
                continue;
            }

            const double interpolation = startDistance / denominator;

            if (interpolation < 0.0 || interpolation > 1.0)
            {
                continue;
            }

            Eigen::Vector3d crossingPoint =
                edgeStart + interpolation * edgeVector;

            /* Project exactly onto the supporting wall */
            const double planeResidual =
                wallNormal.dot(crossingPoint) + wallEquation(3);

            crossingPoint -= planeResidual * wallNormal;

            /* Reject crossings which are too close to the ground or too high */
            if (hasValidGroundEquation)
            {
                const double crossingHeight = std::abs(
                    groundNormal.dot(crossingPoint) + groundEquation(3));

                if (!std::isfinite(crossingHeight) ||
                    crossingHeight < minimumCrossingHeight ||
                    crossingHeight > maximumCrossingHeight)
                {
                    continue;
                }
            }

            /* Check the finite wall bounds */
            const Eigen::Vector3d relativeCrossing =
                crossingPoint - wallCentroid;

            const double crossingU = relativeCrossing.dot(wallAxisU);

            const double crossingV = relativeCrossing.dot(wallAxisV);

            const bool insideFiniteWall =
                crossingU >= minimumU - wallBoundsMargin &&
                crossingU <= maximumU + wallBoundsMargin &&
                crossingV >= minimumV - wallBoundsMargin &&
                crossingV <= maximumV + wallBoundsMargin;

            if (!insideFiniteWall)
            {
                continue;
            }

            /*
             * A cloud ending beside a skeleton edge is an unobserved wall end,
             * not evidence of a doorway. Require mapped wall material on both
             * horizontal sides of the crossing before accepting the aperture.
             */
            if (hasHorizontalWallTangent)
            {
                const double crossingHorizontalCoordinate_m =
                    crossingPoint.dot(horizontalWallTangent_World);
                std::size_t lowerFlankPointCount = 0U;
                std::size_t upperFlankPointCount = 0U;
                double      minimumHorizontalCoordinate_m =
                    std::numeric_limits<double>::infinity();
                double maximumHorizontalCoordinate_m =
                    -std::numeric_limits<double>::infinity();

                for (const double wallCoordinate_m :
                     horizontalWallCoordinates_m)
                {
                    minimumHorizontalCoordinate_m =
                        std::min(minimumHorizontalCoordinate_m,
                                 wallCoordinate_m);
                    maximumHorizontalCoordinate_m =
                        std::max(maximumHorizontalCoordinate_m,
                                 wallCoordinate_m);

                    if (wallCoordinate_m < crossingHorizontalCoordinate_m)
                    {
                        lowerFlankPointCount++;
                    }
                    else if (wallCoordinate_m > crossingHorizontalCoordinate_m)
                    {
                        upperFlankPointCount++;
                    }
                }

                const bool hasLowerFlank =
                    crossingHorizontalCoordinate_m -
                            minimumHorizontalCoordinate_m >=
                        minimumHorizontalFlankExtent_m &&
                    lowerFlankPointCount >= minimumHorizontalFlankPointCount;
                const bool hasUpperFlank =
                    maximumHorizontalCoordinate_m -
                            crossingHorizontalCoordinate_m >=
                        minimumHorizontalFlankExtent_m &&
                    upperFlankPointCount >= minimumHorizontalFlankPointCount;

                if (!hasLowerFlank || !hasUpperFlank)
                {
                    continue;
                }
            }

            /*!
             * Measure the nearest wall point using only displacement inside the
             * wall surface.
             */
            double nearestWallPointDistance =
                std::numeric_limits<double>::max();

            for (const pcl::PointXYZRGBA &wallPointPcl : wallCloud->points)
            {
                if (!pcl::isFinite(wallPointPcl))
                {
                    continue;
                }

                const Eigen::Vector3d wallPoint(
                    static_cast<double>(wallPointPcl.x),
                    static_cast<double>(wallPointPcl.y),
                    static_cast<double>(wallPointPcl.z));

                const Eigen::Vector3d difference = wallPoint - crossingPoint;

                const Eigen::Vector3d inPlaneDifference =
                    difference - difference.dot(wallNormal) * wallNormal;

                nearestWallPointDistance = std::min(nearestWallPointDistance,
                                                    inPlaneDifference.norm());
            }

            /* Reject crossings through populated wall regions */
            if (!std::isfinite(nearestWallPointDistance) ||
                nearestWallPointDistance < minimumOpeningRadius)
            {
                continue;
            }

            acceptedCrossings.push_back(
                {crossingPoint, nearestWallPointDistance});
        }

        if (acceptedCrossings.empty())
        {
            continue;
        }

        /* ------------------------------------------------------------------ *
         * SEPARATE AND CENTRE DISTINCT OPENINGS ON THIS WALL
         * ------------------------------------------------------------------ */

        std::vector<std::vector<AcceptedCrossing>> crossingClusters;

        for (const AcceptedCrossing &acceptedCrossing : acceptedCrossings)
        {
            std::size_t selectedClusterIndex = crossingClusters.size();
            double      nearestClusterDistance_m =
                std::numeric_limits<double>::infinity();

            for (std::size_t clusterIndex = 0U;
                 clusterIndex < crossingClusters.size();
                 clusterIndex++)
            {
                Eigen::Vector3d clusterCentroid_World_m =
                    Eigen::Vector3d::Zero();

                for (const AcceptedCrossing &clusterCrossing :
                     crossingClusters[clusterIndex])
                {
                    clusterCentroid_World_m += clusterCrossing.point_World_m;
                }

                clusterCentroid_World_m /=
                    static_cast<double>(crossingClusters[clusterIndex].size());

                Eigen::Vector3d separation_World_m =
                    acceptedCrossing.point_World_m - clusterCentroid_World_m;

                if (hasValidGroundEquation)
                {
                    separation_World_m -=
                        separation_World_m.dot(groundNormal) * groundNormal;
                }

                const double clusterDistance_m = separation_World_m.norm();

                if (clusterDistance_m < crossingClusterDistance &&
                    clusterDistance_m < nearestClusterDistance_m)
                {
                    selectedClusterIndex     = clusterIndex;
                    nearestClusterDistance_m = clusterDistance_m;
                }
            }

            if (selectedClusterIndex == crossingClusters.size())
            {
                crossingClusters.push_back({acceptedCrossing});
            }
            else
            {
                crossingClusters[selectedClusterIndex].push_back(
                    acceptedCrossing);
            }
        }

        for (const std::vector<AcceptedCrossing> &crossingCluster :
             crossingClusters)
        {
            Eigen::Vector3d passageCentre_World_m  = Eigen::Vector3d::Zero();
            double          maximumOpeningRadius_m = 0.0;

            for (const AcceptedCrossing &crossing : crossingCluster)
            {
                passageCentre_World_m += crossing.point_World_m;
                maximumOpeningRadius_m =
                    std::max(maximumOpeningRadius_m, crossing.openingRadius_m);
            }

            passageCentre_World_m /=
                static_cast<double>(crossingCluster.size());

            double selectedPassageHeight_m = preferredPassageHeight;
            double measuredHeightSpan_m    = 0.0;

            if (hasValidGroundEquation)
            {
                double minimumMeasuredHeight_m =
                    std::numeric_limits<double>::max();
                double maximumMeasuredHeight_m =
                    std::numeric_limits<double>::lowest();

                for (const AcceptedCrossing &crossing : crossingCluster)
                {
                    const double measuredHeight_m =
                        std::abs(groundNormal.dot(crossing.point_World_m) +
                                 groundEquation(3));

                    minimumMeasuredHeight_m =
                        std::min(minimumMeasuredHeight_m, measuredHeight_m);
                    maximumMeasuredHeight_m =
                        std::max(maximumMeasuredHeight_m, measuredHeight_m);
                }

                measuredHeightSpan_m =
                    maximumMeasuredHeight_m - minimumMeasuredHeight_m;

                if (crossingCluster.size() > 1U &&
                    measuredHeightSpan_m >= minimumMeasuredHeightSpan)
                {
                    selectedPassageHeight_m = 0.5 * (minimumMeasuredHeight_m +
                                                     maximumMeasuredHeight_m);
                }

                selectedPassageHeight_m =
                    std::clamp(selectedPassageHeight_m,
                               minimumPassageCentreHeight,
                               maximumPassageCentreHeight);

                double aboveGroundSign =
                    groundNormal.dot(wallCentroid) + groundEquation(3);

                if (std::abs(aboveGroundSign) < 1e-6)
                {
                    aboveGroundSign = groundNormal.dot(passageCentre_World_m) +
                                      groundEquation(3);
                }

                aboveGroundSign = aboveGroundSign >= 0.0 ? 1.0 : -1.0;

                const double currentSignedHeight_m =
                    groundNormal.dot(passageCentre_World_m) + groundEquation(3);
                const double desiredSignedHeight_m =
                    aboveGroundSign * selectedPassageHeight_m;

                passageCentre_World_m +=
                    (desiredSignedHeight_m - currentSignedHeight_m) *
                    groundNormal;
            }

            const double finalPlaneResidual_m =
                wallNormal.dot(passageCentre_World_m) + wallEquation(3);

            passageCentre_World_m -= finalPlaneResidual_m * wallNormal;

            PassageCandidate candidate;
            candidate.wall          = wall;
            candidate.crossingPoint = passageCentre_World_m;
            candidate.openingRadius = maximumOpeningRadius_m;
            candidate.heightSpan_m  = measuredHeightSpan_m;
            candidate.crossingCount = crossingCluster.size();

            passageCandidates.push_back(candidate);
        }
    }

    /* ---------------------------------------------------------------------- *
     * TEMPORAL EVIDENCE ASSOCIATION
     * ---------------------------------------------------------------------- */

    for (OpenPassageEvidence &evidence : openPassageEvidence_)
    {
        evidence.missedUpdateCount++;
    }

    for (PassageCandidate &candidate : passageCandidates)
    {
        OpenPassageEvidence *p_matchingEvidence = nullptr;
        double               nearestEvidenceDistance_m =
            std::numeric_limits<double>::infinity();

        for (OpenPassageEvidence &evidence : openPassageEvidence_)
        {
            if (evidence.p_supportingWall == nullptr ||
                evidence.p_supportingWall->isBad())
            {
                continue;
            }

            Eigen::Vector3d separation_World_m =
                evidence.centroid_World_m - candidate.crossingPoint;

            if (hasValidGroundEquation)
            {
                separation_World_m -=
                    separation_World_m.dot(groundNormal) * groundNormal;
            }

            const double evidenceDistance_m = separation_World_m.norm();

            if (evidenceDistance_m > duplicatePassageDistance ||
                evidenceDistance_m >= nearestEvidenceDistance_m)
            {
                continue;
            }

            Eigen::Vector3d evidenceWallNormal =
                evidence.p_supportingWall->getGlobalEquation().normal();
            Eigen::Vector3d candidateWallNormal =
                candidate.wall->getGlobalEquation().normal();

            if (!evidenceWallNormal.allFinite() ||
                !candidateWallNormal.allFinite() ||
                evidenceWallNormal.norm() < 1e-8 ||
                candidateWallNormal.norm() < 1e-8)
            {
                continue;
            }

            evidenceWallNormal.normalize();
            candidateWallNormal.normalize();

            if (std::abs(evidenceWallNormal.dot(candidateWallNormal)) <
                duplicateNormalAlignment)
            {
                continue;
            }

            Eigen::Vector4d candidateWallEquation =
                candidate.wall->getGlobalEquation().coeffs();
            const double candidateWallNormalNorm =
                candidateWallEquation.head<3>().norm();

            if (candidateWallNormalNorm < 1e-8)
            {
                continue;
            }

            candidateWallEquation /= candidateWallNormalNorm;

            const double supportingWallSeparation_m = std::abs(
                candidateWallEquation.head<3>().dot(evidence.centroid_World_m) +
                candidateWallEquation(3));

            if (supportingWallSeparation_m > 0.30)
            {
                continue;
            }

            p_matchingEvidence        = &evidence;
            nearestEvidenceDistance_m = evidenceDistance_m;
        }

        if (p_matchingEvidence == nullptr)
        {
            openPassageEvidence_.push_back({candidate.wall,
                                            candidate.crossingPoint,
                                            1U,
                                            0U,
                                            skeletonFingerprint,
                                            candidate.openingRadius,
                                            candidate.heightSpan_m});
            candidate.confirmationCount = 1U;
            continue;
        }

        const std::size_t previousConfirmationCount =
            p_matchingEvidence->confirmationCount;
        const double previousWeight = static_cast<double>(
            std::min<std::size_t>(previousConfirmationCount, 10U));

        p_matchingEvidence->centroid_World_m =
            (previousWeight * p_matchingEvidence->centroid_World_m +
             candidate.crossingPoint) /
            (previousWeight + 1.0);
        p_matchingEvidence->p_supportingWall  = candidate.wall;
        p_matchingEvidence->missedUpdateCount = 0U;
        /* The true opening only gets more of it confirmed over time as the
         * crossing evidence accumulates -- never shrinks a size estimate
         * once confirmed. */
        p_matchingEvidence->openingRadius_m =
            std::max(p_matchingEvidence->openingRadius_m,
                     candidate.openingRadius);
        p_matchingEvidence->heightSpan_m =
            std::max(p_matchingEvidence->heightSpan_m, candidate.heightSpan_m);

        if (p_matchingEvidence->lastConfirmedSkeletonFingerprint !=
            skeletonFingerprint)
        {
            p_matchingEvidence->confirmationCount++;
            p_matchingEvidence->lastConfirmedSkeletonFingerprint =
                skeletonFingerprint;
        }

        candidate.crossingPoint     = p_matchingEvidence->centroid_World_m;
        candidate.confirmationCount = p_matchingEvidence->confirmationCount;
        candidate.openingRadius     = p_matchingEvidence->openingRadius_m;
        candidate.heightSpan_m      = p_matchingEvidence->heightSpan_m;

        Eigen::Vector4d supportingWallEquation =
            candidate.wall->getGlobalEquation().coeffs();
        const double supportingWallNormalNorm =
            supportingWallEquation.head<3>().norm();

        if (supportingWallNormalNorm > 1e-8)
        {
            supportingWallEquation /= supportingWallNormalNorm;

            const double passagePlaneResidual_m =
                supportingWallEquation.head<3>().dot(candidate.crossingPoint) +
                supportingWallEquation(3);

            candidate.crossingPoint -=
                passagePlaneResidual_m * supportingWallEquation.head<3>();
            p_matchingEvidence->centroid_World_m = candidate.crossingPoint;
        }
    }

    openPassageEvidence_.erase(
        std::remove_if(
            openPassageEvidence_.begin(),
            openPassageEvidence_.end(),
            [maximumMissedUpdateCount](const OpenPassageEvidence &evidence)
            {
                return evidence.p_supportingWall == nullptr ||
                       evidence.p_supportingWall->isBad() ||
                       evidence.missedUpdateCount > maximumMissedUpdateCount;
            }),
        openPassageEvidence_.end());

    if (passageCandidates.empty())
    {
        return;
    }

    /*
     * Process the clearest candidates first.
     */
    std::sort(passageCandidates.begin(),
              passageCandidates.end(),
              [](const PassageCandidate &first, const PassageCandidate &second)
              { return first.openingRadius > second.openingRadius; });

    /* ---------------------------------------------------------------------- *
     * UPDATE EXISTING PASSAGES OR CREATE NEW ONES
     * ---------------------------------------------------------------------- */

    for (const PassageCandidate &candidate : passageCandidates)
    {
        if (candidate.wall == nullptr || candidate.wall->isBad())
        {
            continue;
        }

        Eigen::Vector3d candidateNormal =
            candidate.wall->getGlobalEquation().normal();

        if (!candidateNormal.allFinite() || candidateNormal.norm() < 1e-8)
        {
            continue;
        }

        candidateNormal.normalize();

        ORB_SLAM3::Passage *matchingPassage           = nullptr;
        bool                hasAmbiguousNearbyPassage = false;

        Eigen::Vector4d candidateWallEquation =
            candidate.wall->getGlobalEquation().coeffs();
        const double candidateWallNormalNorm =
            candidateWallEquation.head<3>().norm();

        if (!candidateWallEquation.allFinite() ||
            candidateWallNormalNorm < 1e-8)
        {
            continue;
        }

        candidateWallEquation /= candidateWallNormalNorm;

        const std::vector<ORB_SLAM3::Passage *> existingPassages =
            mpAtlas->GetAllPassages();

        for (ORB_SLAM3::Passage *existingPassage : existingPassages)
        {
            if (existingPassage == nullptr)
            {
                continue;
            }

            const Eigen::Vector3d existingCentroid =
                existingPassage->getCentroid().cast<double>();

            if (!existingCentroid.allFinite())
            {
                continue;
            }

            Eigen::Vector3d centroidDifference =
                existingCentroid - candidate.crossingPoint;

            /* Ignore vertical displacement during duplicate comparison */
            if (hasValidGroundEquation)
            {
                centroidDifference -=
                    centroidDifference.dot(groundNormal) * groundNormal;
            }

            const double horizontalDistance = centroidDifference.norm();

            if (horizontalDistance > duplicatePassageDistance)
            {
                continue;
            }

            Eigen::Vector3d existingNormal =
                existingPassage->getGlobalEquation().normal();

            if (!existingNormal.allFinite() || existingNormal.norm() < 1e-8)
            {
                continue;
            }

            existingNormal.normalize();

            const double normalAlignment =
                std::abs(existingNormal.dot(candidateNormal));

            const double supportingWallSeparation_m =
                std::abs(candidateWallEquation.head<3>().dot(existingCentroid) +
                         candidateWallEquation(3));

            if (normalAlignment >= duplicateNormalAlignment &&
                supportingWallSeparation_m <= 0.30)
            {
                matchingPassage = existingPassage;
                break;
            }

            /*
             * Do not turn a nearby, overlapping but geometrically degraded
             * estimate into another permanent passage. It remains temporal
             * evidence and can be accepted after plane fusion makes its
             * identity unambiguous.
             */
            if (normalAlignment >= ambiguousDuplicateNormalAlignment &&
                supportingWallSeparation_m <=
                    ambiguousDuplicatePlaneSeparation_m)
            {
                hasAmbiguousNearbyPassage = true;
            }
        }

        /*!
         * Update an existing passage so a previously created low marker moves
         * to the corrected doorway centre.
         */
        if (matchingPassage != nullptr)
        {
            /*
             * Connected ESDF free space through the wall is stronger evidence
             * than a stale blocked-door classification at the same opening.
             */
            matchingPassage->setPassable(true);
            matchingPassage->setCentroid(candidate.crossingPoint);

            /*
             * A passage is framed by the wall face that first produced it.
             * The opposite face of the same physical wall is separate evidence
             * (offset by the wall thickness): adding it must NOT rewrite the
             * passage plane, otherwise the passage normal flips between the
             * two faces every cycle and the far-side/prospective decisions
             * flip with it. Only refresh the plane when this face already
             * anchors the passage; otherwise just pair the face.
             */
            const std::vector<Plane *> matchingSupportingWalls =
                matchingPassage->getAssociateWalls();
            const bool isKnownSupportingFace =
                std::find(matchingSupportingWalls.begin(),
                          matchingSupportingWalls.end(),
                          candidate.wall) != matchingSupportingWalls.end();

            if (isKnownSupportingFace)
            {
                matchingPassage->setGlobalEquation(
                    candidate.wall->getGlobalEquation());
            }

            matchingPassage->addAssociateWall(candidate.wall);

            /* Open passages previously carried no size estimate at all
             * (only door-typed/blocked passages did) -- diameter from the
             * best-confirmed opening radius, height from the best-confirmed
             * vertical crossing span, floored at a typical-door default
             * while that span is still unmeasured. Never shrinks once a
             * larger estimate has been confirmed (candidate.openingRadius/
             * heightSpan_m already hold the running max -- see the
             * temporal evidence merge above). */
            constexpr double defaultOpenPassageHeight_m = 2.0;
            matchingPassage->setWidth(2.0 * candidate.openingRadius);
            matchingPassage->setHeight(
                std::max(candidate.heightSpan_m, defaultOpenPassageHeight_m));

            continue;
        }

        if (hasAmbiguousNearbyPassage)
        {
            continue;
        }

        /* Same-cycle evidence-quantity gate, same semantic level/priority as
         * a wall's own admission (cluster point count / connectivity
         * ratio): enough of the free-space skeleton clustering through the
         * wall THIS cycle is sufficient on its own to create the passage,
         * no separate multi-cycle waiting period. Previously gated on
         * candidate.confirmationCount (minimumConfirmationSnapshots
         * genuinely distinct skeleton topology snapshots), which could take
         * arbitrarily long real exploration time to accumulate -- during
         * that entire window no Passage existed for anything querying
         * mpAtlas->GetAllPassages() to see (the far-side wall-admission gap
         * fixed elsewhere this session). */
        if (candidate.crossingCount < minimumCrossingClusterSize)
        {
            continue;
        }

        /* Create a new passage.
         *
         * NOTE: No prospective room is created here. A prospective room is only
         * legitimate when a passage has exactly ONE associated room
         * (representing unexplored traversable space on the far side). That
         * decision is made by associatePassagesToRooms() after room<->passage
         * association is current, where the associated-room count is known.
         * Creating a prospective room eagerly at detection time (when the
         * passage has 0 associated rooms) leaves a spurious prospective room on
         * passages that later acquire two associated rooms. */
        GeoSemHelpers::createMapPassage(mpAtlas,
                                        nullptr,
                                        candidate.wall,
                                        true,
                                        candidate.crossingPoint);

        /* createMapPassage() returns void; find the passage it just
         * registered (freshly created, so its centroid matches this
         * candidate's crossing point exactly) to size it. Open passages
         * previously carried no size estimate at all -- see the matching
         * branch above for the same estimate's derivation. */
        for (Passage *p_created : mpAtlas->GetAllPassages())
        {
            if (p_created == nullptr ||
                !p_created->getCentroid().isApprox(candidate.crossingPoint,
                                                   1e-6))
            {
                continue;
            }
            constexpr double defaultOpenPassageHeight_m = 2.0;
            p_created->setWidth(2.0 * candidate.openingRadius);
            p_created->setHeight(
                std::max(candidate.heightSpan_m, defaultOpenPassageHeight_m));
            break;
        }
    }
}

void SemanticsManager::detectDoorsAndDoorways(ORB_SLAM3::Atlas *pAtlas)
{
    /* Confirm that the Atlas is valid */
    if (pAtlas == nullptr)
    {
        return;
    }

    /* Extract all planes from the current map */
    const std::vector<ORB_SLAM3::Plane *> allPlanes = pAtlas->GetAllPlanes();

    /* Initialise lists of valid wall and door planes */
    std::vector<ORB_SLAM3::Plane *> wallPlanes;
    std::vector<ORB_SLAM3::Plane *> doorPlanes;

    wallPlanes.reserve(allPlanes.size());
    doorPlanes.reserve(allPlanes.size());

    /* Separate mapped planes according to their confirmed semantic type */
    for (ORB_SLAM3::Plane *plane : allPlanes)
    {
        /* Skip invalid mapped planes */
        if (plane == nullptr || plane->isBad())
        {
            continue;
        }

        /* Store confirmed wall planes */
        if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::WALL)
        {
            wallPlanes.push_back(plane);
            continue;
        }

        /* Store confirmed door planes */
        if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::DOOR)
        {
            doorPlanes.push_back(plane);
        }
    }

    /* Filter wall planes to those with sufficient observations.
     * Provisional rooms are valid evidence - don't require CONFIRMED room
     * association. */
    std::vector<ORB_SLAM3::Plane *> confirmedWallPlanes;
    confirmedWallPlanes.reserve(wallPlanes.size());

    for (ORB_SLAM3::Plane *wall : wallPlanes)
    {
        if (wall == nullptr || wall->isBad())
        {
            continue;
        }

        // Quality gate: minimum observations, not room confirmation status
        if (wall->getObservationCount() >=
            sysParams->room_seg.minimumWallObservationCount)
        {
            confirmedWallPlanes.push_back(wall);
        }
        else
        {
            std::cout << "[SemMgr] Skipping wall " << wall->getId()
                      << " for passage detection: insufficient observations ("
                      << wall->getObservationCount() << " < "
                      << sysParams->room_seg.minimumWallObservationCount << ")."
                      << std::endl;
        }
    }

    /*  Detect blocked passages represented by closed semantic door planes */
    for (ORB_SLAM3::Plane *door : doorPlanes)
    {
        /* Skip invalid door planes */
        if (door == nullptr || door->isBad())
        {
            continue;
        }

        for (ORB_SLAM3::Plane *wall : confirmedWallPlanes)
        {
            /* Skip invalid wall planes */
            if (wall == nullptr || wall->isBad())
            {
                continue;
            }

            /* Door and wall must be parallel */
            if (!Utils::arePlanesParallel(door, wall))
            {
                continue;
            }

            /* Door must lie close to the supporting wall */
            if (Utils::arePlanesApartEnough(
                    door,
                    wall,
                    sysParams->sem_seg.max_wall_door_distance))
            {
                continue;
            }

            /* Create a blocked passage associated with the supporting wall */
            GeoSemHelpers::createMapPassage(mpAtlas, door, wall, false);
        }
    }

    /*!
     * Detect open passages from connected Voxblox skeleton edges which breach
     * finite mapped wall surfaces.
     *
     * @note        Camera trajectory crossings are deliberately not used.
     */
    detectOpenPassagesFromSkeletonEdges(confirmedWallPlanes);
}

void SemanticsManager::updatePassages(ORB_SLAM3::Atlas *pAtlas)
{
    // Get the ground plane
    ORB_SLAM3::Plane *groundPlane = pAtlas->GetBiggestGroundPlane();
    if (groundPlane == nullptr)
        return;

    // Get all passages and update their global pose to be consistent with the
    // ground plane
    std::vector<ORB_SLAM3::Passage *> allPassages = pAtlas->GetAllPassages();

    for (const auto &passage : allPassages)
    {
        if (passage == nullptr || passage->isBad())
        {
            continue;
        }

        // Updating the dimensions of the passage based on the associated door
        // plane
        ORB_SLAM3::Plane *doorPlane = passage->getAssociateDoor();

        // Blocked passages (closed doors) should be aligned with the ground
        // plane normal
        if (!passage->isPassable())
        {
            if (doorPlane == nullptr)
            {
                continue;
            }

            /* Extract width height supple of door */
            std::pair<double, double> widthHeight =
                Utils::computePlaneWidthHeight(
                    doorPlane->getGeometrySnapshot().supportCloud);

            /* Extract the measured height and width */
            const double measuredWidth  = widthHeight.first;
            const double measuredHeight = widthHeight.second;

            /* Extract the max width */
            const double maxWidth =
                static_cast<double>(sysParams->sem_seg.max_door_width);

            /* Extract the max height */
            const double maxHeight =
                static_cast<double>(sysParams->sem_seg.max_door_height);

            /* Determine if dimensions are valid */
            const bool validDimensions =
                std::isfinite(measuredWidth) && std::isfinite(measuredHeight) &&
                measuredWidth > 0.0 && measuredHeight > 0.0 &&
                measuredWidth <= maxWidth && measuredHeight <= maxHeight;

            if (!validDimensions)
            {
                std::cout << "[SemanticsManager] Rejecting door plane "
                          << doorPlane->getId() << " for passage "
                          << passage->getId() << ": measured dimensions "
                          << measuredWidth << "x" << measuredHeight
                          << " m exceed limits " << maxWidth << "x" << maxHeight
                          << " m." << std::endl;

                continue;
            }

            /* Set centroid of the door plane */
            passage->setCentroid(doorPlane->getCentroid());

            /* Get the plane global equation */
            passage->setGlobalEquation(doorPlane->getGlobalEquation());

            /* Set width & height of the passage */
            passage->setWidth(measuredWidth);
            passage->setHeight(measuredHeight);
        }
        else
        {
            /*
             * Open passages are derived from a crossing of their supporting
             * wall. Re-anchor them after every optimization so an independently
             * corrected plane cannot leave the passage or graph edge behind.
             *
             * A passage is expected to be framed by TWO observed wall faces
             * (the two surfaces of the same physical wall, offset by the wall
             * thickness). The stable aperture plane for side discrimination is
             * the MID-PLANE between the two faces, so that a wall on either
             * side of the opening always lies the wall-thickness away from the
             * passage plane (never ON it). Anchoring to only the closest face
             * - or switching between nearest and farthest face across cycles -
             * makes the passage normal flip and breaks the far-side holds.
             */
            const std::vector<ORB_SLAM3::Plane *> supportingFaces =
                passage->getAssociateWalls();

            std::vector<Eigen::Vector4d> validFaceEquations;
            validFaceEquations.reserve(2);

            for (ORB_SLAM3::Plane *p_candidateWall : supportingFaces)
            {
                if (p_candidateWall == nullptr || p_candidateWall->isBad())
                {
                    continue;
                }

                Eigen::Vector4d candidateWallEquation =
                    p_candidateWall->getGlobalEquation().coeffs();

                if (!candidateWallEquation.allFinite())
                {
                    continue;
                }

                const double candidateNormalNorm =
                    candidateWallEquation.head<3>().norm();

                if (!std::isfinite(candidateNormalNorm) ||
                    candidateNormalNorm < 1e-8)
                {
                    continue;
                }

                candidateWallEquation /= candidateNormalNorm;

                if (validFaceEquations.size() < 2)
                {
                    validFaceEquations.push_back(candidateWallEquation);
                }
            }

            const Eigen::Vector3d passageCentroid_World_m =
                passage->getCentroid();

            if (validFaceEquations.size() == 2)
            {
                /*
                 * Two faces of the same physical wall have normals that are
                 * antiparallel (each oriented away from its own room). Orient
                 * them consistently as (n, d) along a shared unit normal and
                 * take the mid-plane. The lower face is used as the reference
                 * orientation so the resulting aperture plane does not depend
                 * on which face happened to produce the passage first.
                 */
                Eigen::Vector4d firstFace  = validFaceEquations[0];
                Eigen::Vector4d secondFace = validFaceEquations[1];

                /*
                 * Orient both faces to the same unit normal: use the normal of
                 * the first face as the shared reference orientation.
                 */
                const double alignmentSecondFace =
                    firstFace.head<3>().dot(secondFace.head<3>());

                if (alignmentSecondFace < 0.0)
                {
                    secondFace *= -1.0;
                }

                /* The mid-plane offset keeps the face centroids symmetric. */
                const double nearFaceDistance_m =
                    firstFace.head<3>().dot(passageCentroid_World_m) +
                    firstFace(3);
                const double farFaceDistance_m =
                    secondFace.head<3>().dot(passageCentroid_World_m) +
                    secondFace(3);

                const double midPlaneDistance_m =
                    0.5 * (nearFaceDistance_m + farFaceDistance_m);

                Eigen::Vector4d midPlaneEquation;
                midPlaneEquation.head<3>() = firstFace.head<3>();
                midPlaneEquation(3) =
                    midPlaneDistance_m -
                    firstFace.head<3>().dot(passageCentroid_World_m);

                ORB_SLAM3::Plane passagePlane;
                passagePlane.setGlobalEquation(g2o::Plane3D(midPlaneEquation));

                if (Utils::arePlanesPerpendicular(&passagePlane, groundPlane))
                {
                    passage->setGlobalEquation(g2o::Plane3D(midPlaneEquation));
                }
                else
                {
                    /* Mid-plane deviates from vertical; fall back to the
                     * single-face anchoring for this cycle. */
                    Eigen::Vector4d referenceEquation = validFaceEquations[0];
                    Eigen::Vector3d anchoredCentroid_World_m =
                        passageCentroid_World_m;
                    const double wallResidual_m =
                        referenceEquation.head<3>().dot(
                            anchoredCentroid_World_m) +
                        referenceEquation(3);

                    anchoredCentroid_World_m -=
                        wallResidual_m * referenceEquation.head<3>();

                    passage->setCentroid(anchoredCentroid_World_m);
                    passage->setGlobalEquation(g2o::Plane3D(referenceEquation));
                }
            }
            else if (validFaceEquations.size() == 1)
            {
                /*
                 * Only one face of the physical wall has been observed so far.
                 * Anchor the passage plane to that face.
                 */
                Eigen::Vector4d referenceEquation = validFaceEquations[0];
                Eigen::Vector3d anchoredCentroid_World_m =
                    passageCentroid_World_m;
                const double wallResidual_m =
                    referenceEquation.head<3>().dot(anchoredCentroid_World_m) +
                    referenceEquation(3);

                anchoredCentroid_World_m -=
                    wallResidual_m * referenceEquation.head<3>();

                passage->setCentroid(anchoredCentroid_World_m);
                passage->setGlobalEquation(g2o::Plane3D(referenceEquation));
            }

            ORB_SLAM3::Plane passagePlane;
            passagePlane.setGlobalEquation(passage->getGlobalEquation());
            if (!Utils::arePlanesPerpendicular(&passagePlane, groundPlane))
            {
                // Project the passage normal onto the horizontal plane to
                // remove tilt
                const Eigen::Vector3d groundNormal =
                    groundPlane->getGlobalEquation()
                        .coeffs()
                        .head<3>()
                        .normalized();
                Eigen::Vector4d globalEq =
                    passage->getGlobalEquation().coeffs();
                Eigen::Vector3d passageNormal = globalEq.head<3>().normalized();

                Eigen::Vector3d correctedNormal =
                    (passageNormal -
                     passageNormal.dot(groundNormal) * groundNormal)
                        .normalized();
                if (correctedNormal.norm() < 1e-6)
                    continue;
                correctedNormal.normalize();

                // Recompute d so the plane still passes through the centroid
                const Eigen::Vector3d centroid =
                    passage->getCentroid().cast<double>();
                const double d = -correctedNormal.dot(centroid);

                Eigen::Vector4d correctedCoeffs;
                correctedCoeffs.head<3>() = correctedNormal;
                correctedCoeffs(3)        = d;

                passage->setGlobalEquation(g2o::Plane3D(correctedCoeffs));
            }
        }
    }
}

void SemanticsManager::mergeOverlappingPassages(void)
{
    Plane *p_groundPlane = mpAtlas->GetBiggestGroundPlane();
    if (p_groundPlane == nullptr || p_groundPlane->isBad())
    {
        return;
    }
    const Eigen::Vector4d groundEq =
        p_groundPlane->getGlobalEquation().coeffs();
    const double groundNorm = groundEq.head<3>().norm();
    if (!groundEq.allFinite() || groundNorm < 1e-8)
    {
        return;
    }
    const Eigen::Vector3d groundNormal_World = groundEq.head<3>() / groundNorm;

    const std::vector<Passage *> allPassages = mpAtlas->GetAllPassages();

    for (std::size_t i = 0U; i < allPassages.size(); ++i)
    {
        Passage *p_first = allPassages[i];
        if (p_first == nullptr || p_first->isBad())
        {
            continue;
        }

        for (std::size_t j = i + 1U; j < allPassages.size(); ++j)
        {
            Passage *p_second = allPassages[j];
            if (p_second == nullptr || p_second->isBad())
            {
                continue;
            }

            /* Must be the same physical wall's opening: near-coplanar
             * passage equations (parallel normals, matching offset once
             * consistently oriented). This deliberately reuses the same
             * kind of alignment/offset gates updatePassages()'s own
             * duplicate-detection uses, applied here across ALL existing
             * passages rather than only against fresh detection candidates. */
            Eigen::Vector4d firstEquation_World =
                p_first->getGlobalEquation().coeffs();
            Eigen::Vector4d secondEquation_World =
                p_second->getGlobalEquation().coeffs();
            const double firstNormalNorm = firstEquation_World.head<3>().norm();
            const double secondNormalNorm =
                secondEquation_World.head<3>().norm();
            if (!firstEquation_World.allFinite() ||
                !secondEquation_World.allFinite() || firstNormalNorm < 1e-8 ||
                secondNormalNorm < 1e-8)
            {
                continue;
            }
            firstEquation_World /= firstNormalNorm;
            secondEquation_World /= secondNormalNorm;

            constexpr double minimumCoplanarNormalAlignment = 0.90;
            constexpr double maximumCoplanarOffset_m        = 0.30;
            const double normalAlignment = firstEquation_World.head<3>().dot(
                secondEquation_World.head<3>());
            if (std::abs(normalAlignment) < minimumCoplanarNormalAlignment)
            {
                continue;
            }
            Eigen::Vector4d orientedSecondEquation_World = secondEquation_World;
            if (normalAlignment < 0.0)
            {
                orientedSecondEquation_World = -orientedSecondEquation_World;
            }
            if (std::abs(firstEquation_World(3) -
                         orientedSecondEquation_World(3)) >
                maximumCoplanarOffset_m)
            {
                continue;
            }

            /* Shared 2D basis in the wall's own plane: horizontal tangent
             * (ground normal x wall normal) for width, ground normal for
             * height -- same construction used for the ground-aligned wall
             * admission gate. */
            Eigen::Vector3d axisU_World =
                groundNormal_World.cross(firstEquation_World.head<3>());
            const double axisUNorm = axisU_World.norm();
            if (axisUNorm < 1e-3)
            {
                continue;
            }
            axisU_World /= axisUNorm;
            const Eigen::Vector3d &axisV_World = groundNormal_World;

            const Eigen::Vector3d firstCentroid_World = p_first->getCentroid();
            const Eigen::Vector3d secondCentroid_World =
                p_second->getCentroid();
            if (!firstCentroid_World.allFinite() ||
                !secondCentroid_World.allFinite())
            {
                continue;
            }

            const double firstU  = firstCentroid_World.dot(axisU_World);
            const double firstV  = firstCentroid_World.dot(axisV_World);
            const double secondU = secondCentroid_World.dot(axisU_World);
            const double secondV = secondCentroid_World.dot(axisV_World);

            const double combinedHalfWidth_m =
                0.5 * (p_first->getWidth() + p_second->getWidth());
            const double combinedHalfHeight_m =
                0.5 * (p_first->getHeight() + p_second->getHeight());

            const bool overlapsInWidth =
                std::abs(firstU - secondU) < combinedHalfWidth_m;
            const bool overlapsInHeight =
                std::abs(firstV - secondV) < combinedHalfHeight_m;
            if (!overlapsInWidth || !overlapsInHeight)
            {
                continue;
            }

            Passage *p_survivor =
                (p_first->getId() <= p_second->getId()) ? p_first : p_second;
            Passage *p_absorbed = (p_survivor == p_first) ? p_second : p_first;

            for (Plane *p_wall : p_absorbed->getAssociateWalls())
            {
                p_survivor->addAssociateWall(p_wall);
            }
            p_survivor->setWidth(
                std::max(p_survivor->getWidth(), p_absorbed->getWidth()));
            p_survivor->setHeight(
                std::max(p_survivor->getHeight(), p_absorbed->getHeight()));
            p_survivor->setPassable(p_survivor->isPassable() ||
                                    p_absorbed->isPassable());
            p_survivor->mergeKnownSideProvenance(
                p_absorbed->getKnownSideProvenance());
            if (!p_survivor->hasProspectiveRoom() &&
                p_absorbed->hasProspectiveRoom())
            {
                p_survivor->setProspectiveRoom(
                    p_absorbed->getProspectiveRoom());
            }

            if (loggedPassageMergeIds_
                    .insert({p_survivor->getId(), p_absorbed->getId()})
                    .second)
            {
                std::cout << "[SemMgr] Passage#" << p_absorbed->getId()
                          << " overlaps Passage#" << p_survivor->getId()
                          << " in their shared wall's 2D plane; merged "
                             "evidence into Passage#"
                          << p_survivor->getId() << "." << std::endl;
            }
        }
    }
}

void SemanticsManager::updateTraversalEvidence(ORB_SLAM3::Atlas *pAtlas)
{
    if (pAtlas == nullptr)
    {
        return;
    }

    Map *p_activeMap = pAtlas->GetCurrentMap();

    if (p_activeMap == nullptr)
    {
        return;
    }

    resetTemporalStateForMap(p_activeMap);

    seedCurrentRoomFromActiveMap(p_activeMap);

    std::vector<KeyFrame *> orderedKeyFrames = p_activeMap->GetAllKeyFrames();
    orderedKeyFrames.erase(std::remove_if(orderedKeyFrames.begin(),
                                          orderedKeyFrames.end(),
                                          [](KeyFrame *p_keyFrame) {
                                              return p_keyFrame == nullptr ||
                                                     p_keyFrame->isBad();
                                          }),
                           orderedKeyFrames.end());
    std::sort(orderedKeyFrames.begin(),
              orderedKeyFrames.end(),
              [](const KeyFrame *p_first, const KeyFrame *p_second)
              {
                  if (p_first->mnFrameId != p_second->mnFrameId)
                  {
                      return p_first->mnFrameId < p_second->mnFrameId;
                  }
                  return p_first->mnId < p_second->mnId;
              });

    if (orderedKeyFrames.empty())
    {
        return;
    }

    /* Prepare the ground normal: aperture height/width tests need the vertical
     * axis. Without it there is no reliable opening bounds test. */
    ORB_SLAM3::Plane *groundPlane = pAtlas->GetBiggestGroundPlane();

    Eigen::Vector3d groundNormal_World = Eigen::Vector3d::Zero();

    bool hasValidGroundNormal = false;

    if (groundPlane != nullptr && !groundPlane->isBad())
    {
        Eigen::Vector4d groundEquation =
            groundPlane->getGlobalEquation().coeffs();
        const double groundNormalNorm = groundEquation.head<3>().norm();

        if (groundEquation.allFinite() && std::isfinite(groundNormalNorm) &&
            groundNormalNorm > 1e-8)
        {
            groundEquation /= groundNormalNorm;
            groundNormal_World   = groundEquation.head<3>();
            hasValidGroundNormal = true;
        }
    }

    if (!hasValidGroundNormal)
    {
        return;
    }

    const double openingMargin_m = static_cast<double>(
        sysParams->room_seg.passagePartition.openingMargin_m);
    const double minimumSideDistance_m = static_cast<double>(
        sysParams->room_seg.passagePartition.minimumSideDistance_m);
    const std::vector<Passage *> passages = p_activeMap->GetAllPassages();

    /* Passage confirmation is delayed relative to flight. Replay a bounded
     * recent trajectory on every semantic cycle; Passage deduplicates segment
     * IDs, so a crossing observed before the passage existed is retained once
     * the persistent passage appears. */
    constexpr std::size_t maximumTraversalHistoryKeyFrames = 128U;
    const std::size_t     historyStartIndex =
        orderedKeyFrames.size() > maximumTraversalHistoryKeyFrames
                ? orderedKeyFrames.size() - maximumTraversalHistoryKeyFrames
                : 0U;
    KeyFrame *p_seedKeyFrame = orderedKeyFrames[historyStartIndex];
    currentCameraCenter_World_m =
        p_seedKeyFrame->GetCameraCenter().cast<double>();
    if (!currentCameraCenter_World_m.allFinite())
    {
        return;
    }
    hasCameraCenter_  = true;
    pCameraCenterMap_ = p_activeMap;

    for (std::size_t keyFrameIndex = historyStartIndex + 1U;
         keyFrameIndex < orderedKeyFrames.size();
         ++keyFrameIndex)
    {
        KeyFrame *p_keyFrame = orderedKeyFrames[keyFrameIndex];

        const Eigen::Vector3d nextCameraCenter_World_m =
            p_keyFrame->GetCameraCenter().cast<double>();

        lastTraversalFrameId_    = p_keyFrame->mnFrameId;
        lastTraversalKeyFrameId_ = p_keyFrame->mnId;

        if (!nextCameraCenter_World_m.allFinite())
        {
            continue;
        }

        previousCameraCenter_World_m = currentCameraCenter_World_m;
        currentCameraCenter_World_m  = nextCameraCenter_World_m;

        for (ORB_SLAM3::Passage *p_passage : passages)
        {
            if (p_passage == nullptr || !p_passage->isPassable())
            {
                continue;
            }

            /* The segment joining the last two camera centres approximates the
             * UAV trajectory. When it crosses inside the finite aperture while
             * the passage is passable, the UAV has flown through the opening:
             * record traversal evidence. */
            if (segmentCrossesPassageOpening(previousCameraCenter_World_m,
                                             currentCameraCenter_World_m,
                                             p_passage,
                                             groundNormal_World,
                                             openingMargin_m,
                                             minimumSideDistance_m,
                                             true))
            {
                const bool wasSettled = p_passage->getTraversalEvidence();

                Passage::TraversalDirection traversalDirection =
                    Passage::TraversalDirection::UNKNOWN;
                Passage::KnownSideProvenance knownSide =
                    p_passage->getKnownSideProvenance();
                if (!knownSide.hasDirection())
                {
                    Eigen::Vector4d passageEquation =
                        p_passage->getGlobalEquation().coeffs();
                    const double normalNorm = passageEquation.head<3>().norm();
                    if (passageEquation.allFinite() && normalNorm > 1e-8)
                    {
                        passageEquation /= normalNorm;
                        const double observationSide =
                            passageEquation.head<3>().dot(
                                previousCameraCenter_World_m) +
                            passageEquation(3);
                        if (std::abs(observationSide) > minimumSideDistance_m)
                        {
                            p_passage->setKnownSideDirection(
                                (observationSide > 0.0 ? 1.0 : -1.0) *
                                passageEquation.head<3>());
                            knownSide = p_passage->getKnownSideProvenance();
                        }
                    }
                }
                if (knownSide.hasDirection())
                {
                    const Eigen::Vector3d startFromPassage_World_m =
                        previousCameraCenter_World_m - p_passage->getCentroid();
                    traversalDirection =
                        startFromPassage_World_m.dot(
                            knownSide.direction_World) >= 0.0
                            ? Passage::TraversalDirection::KNOWN_TO_FAR
                            : Passage::TraversalDirection::FAR_TO_KNOWN;
                }

                /* Resolve the room entered after crossing this passage: a
                 * KNOWN_TO_FAR crossing reaches the far side, a FAR_TO_KNOWN
                 * crossing returns to the known side. */
                ORB_SLAM3::Room *p_reachedRoom = nullptr;
                if (traversalDirection ==
                    Passage::TraversalDirection::KNOWN_TO_FAR)
                {
                    p_reachedRoom = p_passage->getProspectiveRoom();
                }
                else if (traversalDirection ==
                         Passage::TraversalDirection::FAR_TO_KNOWN)
                {
                    p_reachedRoom = knownSide.pRoom;
                }
                const std::vector<Room *> activeRooms =
                    p_activeMap->GetAllRooms();
                const bool reachedRoomIsLive =
                    p_reachedRoom != nullptr && !p_reachedRoom->isBad() &&
                    p_reachedRoom->getMap() == p_activeMap &&
                    std::find(activeRooms.begin(),
                              activeRooms.end(),
                              p_reachedRoom) != activeRooms.end();
                if (reachedRoomIsLive)
                {
                    if (p_reachedRoom->getRoomVariant() ==
                        Room::roomVariant::UNDEFINED)
                    {
                        p_activeMap->PromoteCandidateMapRoom(p_reachedRoom);
                        p_reachedRoom->setRoomVariant(Room::roomVariant::ROOM);
                        p_reachedRoom->setName(
                            "Room#" + std::to_string(p_reachedRoom->getId()));
                        p_reachedRoom->setBoundaryStatus(
                            Room::BoundaryStatus::UNOBSERVED);
                        prospectiveRoomCycles_.erase(p_reachedRoom->getId());

                        Floor *p_floor = Floor::selectBestObservedFloor(
                            p_activeMap->GetAllFloors());
                        if (p_floor != nullptr)
                        {
                            p_floor->addRoom(p_reachedRoom);
                        }
                        std::cout
                            << "SG_PIPELINE {\"event\":\"room_promotion\","
                               "\"map_id\":"
                            << p_activeMap->GetId()
                            << ",\"room_id\":" << p_reachedRoom->getId()
                            << ",\"passage_id\":" << p_passage->getId()
                            << ",\"reason\":\"PASSAGE_TRAVERSAL\","
                               "\"semantic_cycle\":"
                            << pipelineSemanticCycle_ << "}" << std::endl;
                    }
                    const int reachedRoomId = p_reachedRoom->getId();
                    {
                        std::lock_guard<std::mutex> currentRoomLock(
                            mMutexCurrentRoom);
                        currentRoomId_ = reachedRoomId;
                    }
                    mpAtlas->setCurrentSemanticRoomIdentity(reachedRoomId);
                    /* Completed passage traversal into this room: entry
                     * evidence marks it visited. */
                    p_reachedRoom->setPreviouslyVisited(true);
                }

                const bool addedTraversal =
                    p_passage->addTraversalObservation(traversalDirection,
                                                       p_keyFrame->mnFrameId,
                                                       p_keyFrame->mnId);

                /* Only newly accepted segment evidence is a new tracker event.
                 * The passage owns segment deduplication, so replayed history
                 * must not republish crossing evidence. */
                if (addedTraversal)
                {
                    std::lock_guard<std::mutex> currentRoomLock(
                        mMutexCurrentRoom);
                    crossingEventPending_ = true;
                    crossingBothSidesPending_ =
                        crossingBothSidesPending_ ||
                        p_passage->hasBidirectionalTraversalEvidence();
#ifdef VS_GRAPHS_ENABLE_ROOM_TRACKER_TEST_HOOK
                    std::function<void()> publishHook =
                        std::move(roomTrackerPendingPublishHook_);
                    if (publishHook)
                    {
                        publishHook();
                    }
#endif
                }

                if (addedTraversal && !wasSettled)
                {
                    std::cout << "[SemMgr] Passage#" << p_passage->getId()
                              << " traversed (traversal evidence settled)."
                              << std::endl;
                }
            }
        }
    }

    KeyFrame *p_latestKeyFrame  = orderedKeyFrames.back();
    lastTraversalFrameId_       = p_latestKeyFrame->mnFrameId;
    lastTraversalKeyFrameId_    = p_latestKeyFrame->mnId;
    hasTraversalKeyFrameCursor_ = true;
}

void SemanticsManager::seedCurrentRoomFromActiveMap(Map *p_activeMap_in)
{
    if (p_activeMap_in == nullptr)
    {
        return;
    }

    std::lock_guard<std::mutex> currentRoomLock(mMutexCurrentRoom);
    if (currentRoomId_ != -1)
    {
        return;
    }

    const std::vector<Room *> rooms = p_activeMap_in->GetAllRooms();
    for (Room *p_room : rooms)
    {
        if (p_room != nullptr && !p_room->isBad() &&
            p_room->getRoomVariant() == Room::roomVariant::ROOM)
        {
            currentRoomId_ = p_room->getId();
            mpAtlas->setCurrentSemanticRoomIdentity(currentRoomId_);
            return;
        }
    }
}

int SemanticsManager::getCurrentRoomId() const
{
    std::lock_guard<std::mutex> currentRoomLock(mMutexCurrentRoom);
    return currentRoomId_;
}

int SemanticsManager::getLastKnownRoomId() const
{
    std::lock_guard<std::mutex> currentRoomLock(mMutexCurrentRoom);
    return lastKnownRoomId_;
}

void SemanticsManager::onTrackingLost(void)
{
    std::lock_guard<std::mutex> currentRoomLock(mMutexCurrentRoom);
    if (!trackingLossEpisodeActive_)
    {
        lastKnownRoomId_ = currentRoomId_ >= 0 || mpAtlas == nullptr
                               ? currentRoomId_
                               : mpAtlas->getCurrentSemanticRoomIdentity();
        if (mpAtlas != nullptr && lastKnownRoomId_ >= 0)
        {
            mpAtlas->setCurrentSemanticRoomIdentity(lastKnownRoomId_);
        }
        trackingLostPending_       = true;
        trackingLossEpisodeActive_ = true;
    }
}

void SemanticsManager::onTrackingRecovered(void)
{
    std::lock_guard<std::mutex> currentRoomLock(mMutexCurrentRoom);
    trackingLossEpisodeActive_ = false;
}

void SemanticsManager::submitVerificationVerdict(
    const VerificationVerdict &verdict_in)
{
    std::lock_guard<std::mutex> currentRoomLock(mMutexCurrentRoom);
    verificationVerdict_        = verdict_in;
    verificationVerdictPending_ = true;
}

void SemanticsManager::updateRoomTrackerState(double now_s)
{
    /* Consume the per-cycle signals. trackingLostPending_ is set on another
     * thread (System::TrackRGBD's real per-frame tracking state, and also
     * reachable via the on-demand System::GetMissionHealthSnapshot RPC), so
     * it is read and cleared under mMutexCurrentRoom. */
    bool                crossingPending     = false;
    bool                bothSidesPending    = false;
    bool                trackingLostPending = false;
    VerificationVerdict verification;
    {
        std::lock_guard<std::mutex> currentRoomLock(mMutexCurrentRoom);
        crossingPending           = crossingEventPending_;
        bothSidesPending          = crossingBothSidesPending_;
        trackingLostPending       = trackingLostPending_;
        crossingEventPending_     = false;
        crossingBothSidesPending_ = false;
        trackingLostPending_      = false;
        if (verificationVerdictPending_)
        {
            verification                = verificationVerdict_;
            verificationVerdict_        = VerificationVerdict();
            verificationVerdictPending_ = false;
        }
    }

    /* Passage crossing evidence. segmentCrossesPassageOpening() already
     * required a passable passage; a detected crossing is therefore direct
     * geometric evidence and carries full traversal confidence until the
     * Phase 4 verifier supplies a calibrated value. */
    TraversalGuardValues crossing;
    crossing.passageDetected   = crossingPending;
    crossing.passable          = crossingPending;
    crossing.confidence        = crossingPending ? 1.0 : 0.0;
    crossing.bothSidesObserved = bothSidesPending;

    TrackingStatusInput tracking;
    tracking.lost = trackingLostPending;
    pendingNewMapCreated_ =
        pendingNewMapCreated_ || mpAtlas->consumeNewMapCreatedEvent();
    tracking.newMapCreated = pendingNewMapCreated_;
    if (tracking.lost && tracking.newMapCreated)
    {
        /* RoomTracker commits at most one row per cycle. Preserve the map event
         * for the following cycle instead of losing it behind TRACKING_LOST. */
        newMapCreatedDeferred_ = true;
        tracking.newMapCreated = false;
    }
    else
    {
        newMapCreatedDeferred_ = false;
    }

    roomTracker_.step(now_s, crossing, verification, tracking);
    const TransitionEvent &lastEvent = roomTracker_.getLastEvent();
    if (lastEvent.accepted &&
        (lastEvent.event == RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH ||
         lastEvent.event == RoomTrackingEvent::LOST_TIMEOUT ||
         lastEvent.event == RoomTrackingEvent::REACQUIRE_TIMEOUT))
    {
        pendingNewMapCreated_ = false;
    }
}

Room *SemanticsManager::findRoomByMapAndId(long unsigned int mapId_in,
                                           int               roomId_in) const
{
    for (Map *p_map : mpAtlas->GetAllMaps())
    {
        if (p_map == nullptr || p_map->GetId() != mapId_in)
        {
            continue;
        }
        for (Room *p_room : p_map->GetAllRooms())
        {
            if (p_room != nullptr && !p_room->isBad() &&
                p_room->getRoomVariant() == Room::roomVariant::ROOM &&
                p_room->getId() == roomId_in)
            {
                return p_room;
            }
        }
        break;
    }
    return nullptr;
}

void SemanticsManager::evaluateTopCandidateVerification(
    const std::vector<SemanticCandidate> &candidates_in)
{
    if (candidates_in.empty())
    {
        return;
    }

    const SemanticCandidate &topCandidate = candidates_in.front();
    if (!topCandidate.minimumEvidenceSatisfied)
    {
        return;
    }

    /* SemanticCandidates::generateWithStatus() marks every candidate within
     * ambiguityMargin of the best distance as `ambiguous`, including the
     * best candidate itself -- its own distance trivially satisfies
     * "<= best distance + margin", so `topCandidate.ambiguous` is always
     * true and can never be read as a gate on its own. A genuine tie instead
     * shows up as a SECOND candidate also carrying `ambiguous == true`. */
    const bool topCandidateIsUniqueLeader =
        candidates_in.size() == 1U || !candidates_in[1].ambiguous;
    if (!topCandidateIsUniqueLeader)
    {
        return;
    }

    Room *p_roomA =
        findRoomByMapAndId(topCandidate.mapAId, topCandidate.roomAId);
    Room *p_roomB =
        findRoomByMapAndId(topCandidate.mapBId, topCandidate.roomBId);
    if (p_roomA == nullptr || p_roomB == nullptr)
    {
        return;
    }

    const SemanticVerifyConfig verifyConfig =
        SemanticVerify::configFromSystemParams();
    const std::vector<VerifyWallObservation> wallsA =
        SemanticVerify::collectWallObservations(p_roomA, verifyConfig);
    const std::vector<VerifyWallObservation> wallsB =
        SemanticVerify::collectWallObservations(p_roomB, verifyConfig);

    SemanticVerifyResult result =
        SemanticVerify::verify(wallsA, wallsB, verifyConfig);

    /* The floor gate can only turn a geometric PASS into a final rejection
     * (toVerificationVerdict() ANDs pass with floorGatePassed), so only run
     * it -- and only when both rooms actually carry a floor identity to
     * compare -- when that outcome is in play; skipping it here (as opposed
     * to skipping it when the two rooms could plausibly share a floor) would
     * be what SemanticVerify.h's runFloorGate() doc warns under-reports a
     * real pass as a false negative. */
    Floor *p_floorA = p_roomA->getFloor();
    Floor *p_floorB = p_roomB->getFloor();
    if (result.pass && p_floorA != nullptr && p_floorB != nullptr &&
        p_floorA->hasPlaneIdentity() && p_floorB->hasPlaneIdentity())
    {
        /* verify()'s transform_AToB maps room-A points into room B's frame,
         * i.e. A is absorbed into B -- matches runFloorGate's
         * absorbed->surviving convention. */
        SemanticVerify::runFloorGate(result,
                                     p_roomB->getMap(),
                                     p_roomA->getMap(),
                                     result.transform_AToB);
    }

    const auto rejectReasonName = [](VerifyRejectReason reason)
    {
        switch (reason)
        {
        case VerifyRejectReason::NONE:
            return "NONE";
        case VerifyRejectReason::TOO_FEW_WALLS:
            return "TOO_FEW_WALLS";
        case VerifyRejectReason::NO_VALID_HYPOTHESIS:
            return "NO_VALID_HYPOTHESIS";
        case VerifyRejectReason::AMBIGUOUS_TOP_HYPOTHESES:
            return "AMBIGUOUS_TOP_HYPOTHESES";
        case VerifyRejectReason::BELOW_MIN_INLIER_RATIO:
            return "BELOW_MIN_INLIER_RATIO";
        case VerifyRejectReason::REFINED_FIT_NOT_OBSERVABLE:
            return "REFINED_FIT_NOT_OBSERVABLE";
        }
        return "unknown";
    };

    std::cout << "[SemMgr] verification_evaluated mapA=" << topCandidate.mapAId
              << " roomA=" << topCandidate.roomAId
              << " mapB=" << topCandidate.mapBId
              << " roomB=" << topCandidate.roomBId
              << " status=" << static_cast<int>(result.status)
              << " rejectReason=" << rejectReasonName(result.rejectReason)
              << " wallCountA=" << wallsA.size()
              << " wallCountB=" << wallsB.size()
              << " topInlierCount=" << result.topInlierCount
              << " runnerUpInlierCount=" << result.runnerUpInlierCount
              << " inlierRatio=" << result.inlierRatio
              << " floorGateRan=" << result.floorGateRan
              << " floorGateResult=\""
              << (result.floorGateResult.empty() ? "-" : result.floorGateResult)
              << "\"" << std::endl;

    /* Verification only: no Atlas mutation here. This makes RoomTracker's
     * VerificationVerdict input real; Milestone 3's shared merge trigger is
     * a separate, deliberately gated step (see the comment above this
     * method's call site in Run()). */
    submitVerificationVerdict(result.toVerificationVerdict());
}

#ifdef VS_GRAPHS_ENABLE_ROOM_TRACKER_TEST_HOOK
void SemanticsManager::processRoomTrackerPendingForTest(double now_s)
{
    updateRoomTrackerState(now_s);
}

const std::vector<TransitionEvent> &
    SemanticsManager::getRoomTrackerEventHistoryForTest() const
{
    return roomTracker_.getEventHistory();
}

std::pair<bool, bool> SemanticsManager::getRoomTrackerPendingForTest() const
{
    std::lock_guard<std::mutex> currentRoomLock(mMutexCurrentRoom);
    return {crossingEventPending_, crossingBothSidesPending_};
}

RoomTrackingState SemanticsManager::getRoomTrackerStateForTest() const
{
    return roomTracker_.getState();
}

bool SemanticsManager::tryLockRoomTrackerPendingMutexForTest() const
{
    std::unique_lock<std::mutex> currentRoomLock(mMutexCurrentRoom,
                                                 std::try_to_lock);
    return currentRoomLock.owns_lock();
}

void SemanticsManager::setRoomTrackerPendingPublishHookForTest(
    std::function<void()> hook_in)
{
    std::lock_guard<std::mutex> currentRoomLock(mMutexCurrentRoom);
    roomTrackerPendingPublishHook_ = std::move(hook_in);
}

void SemanticsManager::evaluateTopCandidateVerificationForTest(
    const std::vector<SemanticCandidate> &candidates_in)
{
    evaluateTopCandidateVerification(candidates_in);
}

bool SemanticsManager::evaluateWallAdmissionEvidenceAdmissibleForTest(
    Plane                 *p_wall_in,
    const Eigen::Vector3d &groundNormal_World_in) const
{
    return evaluateWallAdmissionEvidence(p_wall_in,
                                         sysParams,
                                         groundNormal_World_in)
        .admissible;
}

bool SemanticsManager::admitWallToRoomForTest(Room  *p_room_inout,
                                              Plane *p_candidateWall_in)
{
    return admitWallToRoom(p_room_inout, p_candidateWall_in);
}

void SemanticsManager::enforcePassageSideInvariantForTest(void)
{
    enforcePassageSideInvariant();
}

void SemanticsManager::seedCurrentRoomFromActiveMapForTest(Map *p_activeMap_in)
{
    seedCurrentRoomFromActiveMap(p_activeMap_in);
}

std::optional<float>
    SemanticsManager::computeGroundPlaneHeightForTest(Plane *p_groundPlane_in)
{
    return computeGroundPlaneHeight(p_groundPlane_in);
}

void SemanticsManager::reconcileWallFacePairsForTest(void)
{
    reconcileWallFacePairs();
}

void SemanticsManager::getUpdatedFloorsForTest(void)
{
    getUpdatedFloors();
}

int SemanticsManager::ensureActiveMapBootstrapHierarchyForTest(
    const Eigen::Vector3d &cameraPosition_World_m_in)
{
    pipelineSemanticCycle_++;
    return static_cast<int>(
        ensureActiveMapBootstrapHierarchy(cameraPosition_World_m_in));
}

void SemanticsManager::setCurrentRoomIdForTest(const int roomId_in)
{
    std::lock_guard<std::mutex> currentRoomLock(mMutexCurrentRoom);
    currentRoomId_ = roomId_in;
    mpAtlas->setCurrentSemanticRoomIdentity(roomId_in);
}

void SemanticsManager::associateAllWallsToRoomsForTest(void)
{
    associateAllWallsToRooms();
}

void SemanticsManager::associatePassagesToRoomsForTest(void)
{
    associatePassagesToRooms();
}

void SemanticsManager::suppressUndefendedWallsForTest(void)
{
    suppressUndefendedWalls();
}

int SemanticsManager::getPendingWallAgeForTest(int wallId_in) const
{
    const std::unordered_map<int, UndefendedWallState>::const_iterator found =
        undefendedWalls_.find(wallId_in);
    return found == undefendedWalls_.end()
               ? -1
               : static_cast<int>(found->second.unresolvedCycles);
}

void SemanticsManager::reconcileRoomGroundPlanesForTest(void)
{
    reconcileRoomGroundPlanes();
}

void SemanticsManager::validateRoomBoundariesForTest(void)
{
    validateRoomBoundaries();
}
#endif

Eigen::Vector3f SemanticsManager::transformPlaneEqToGroundReference(
    const Eigen::Vector4d &planeEq)
{
    /* extract the rotation matrix from the transformation matrix */
    Eigen::Matrix3f rotationMatrix = mPlanePoseMat.block<3, 3>(0, 0);

    /* Compute the inverse transpose of the rotation matrix */
    Eigen::Matrix3f inverseTransposeRotationMatrix =
        rotationMatrix.inverse().transpose();

    /* Transform the coefficients of the plane equation */
    Eigen::Vector3f transformedPlaneCoefficients =
        inverseTransposeRotationMatrix * planeEq.head<3>().cast<float>();

    /* Find the normalized coefficients */
    transformedPlaneCoefficients.normalize();

    return transformedPlaneCoefficients;
}

std::optional<float>
    SemanticsManager::computeGroundPlaneHeight(Plane *groundPlane)
{
    /* Transform the planeCloud according to the planePose */
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr planeCloud =
        groundPlane->getGeometrySnapshot().supportCloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedCloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud(*planeCloud, *transformedCloud, mPlanePoseMat);

    /* Not a median: partial_sort with std::greater keeps the lower half in
       descending order, so [numPoint-1] is the upper edge of that half. */
    std::vector<float> yVals;
    for (const auto &point : transformedCloud->points)
    {
        yVals.push_back(point.y);
    }

    size_t numPoint = yVals.size() / 2;

    /* An empty (or single-point) support cloud -- plane created before its
       first refit, or cleared during replaceMapClouds -- makes numPoint == 0,
       leaving nothing for [numPoint - 1] to address. Report "unknown" rather
       than substituting 0.0, which is a valid real height and would silently
       corrupt filterGroundPlanes' threshold. */
    if (numPoint == 0)
    {
        return std::nullopt;
    }

    std::partial_sort(yVals.begin(),
                      yVals.begin() + numPoint,
                      yVals.end(),
                      std::greater<float>());

    return yVals[numPoint - 1];
}

Eigen::Matrix4f SemanticsManager::computePlaneToHorizontal(const Plane *plane)
{
    // initialize the transformation with translation set to a zero vector
    Eigen::Isometry3d planePose;
    planePose.translation() = Eigen::Vector3d(0, 0, 0);

    // normalize the normal vector
    Eigen::Vector3d normal = plane->getGlobalEquation().coeffs().head<3>();

    // get the rotation from the ground plane to the plane with y-facing
    // vertical downwards
    Eigen::Vector3d    verticalAxis = Eigen::Vector3d(0, -1, 0);
    Eigen::Quaterniond q;
    q.setFromTwoVectors(normal, verticalAxis);
    planePose.linear() = q.toRotationMatrix();

    // form homogenous transformation matrix
    Eigen::Matrix4f planePoseMat = planePose.matrix().cast<float>();
    planePoseMat(3, 3)           = 1.0;

    return planePoseMat;
}

std::vector<std::vector<Eigen::Vector3d>>
    SemanticsManager::partitionFreeSpaceAtPassages(
        const std::vector<std::vector<Eigen::Vector3d>>
            &freeSpaceClusters_World_m_in) const
{
    const SystemParams::room_seg::PassagePartition &partitionParameters =
        sysParams->room_seg.passagePartition;

    if (!partitionParameters.enabled || freeSpaceClusters_World_m_in.empty())
    {
        return freeSpaceClusters_World_m_in;
    }

    const std::vector<Passage *> allPassages = mpAtlas->GetAllPassages();
    std::vector<Passage *>       confirmedOpenPassages;

    for (Passage *p_passage : allPassages)
    {
        if (p_passage != nullptr && !p_passage->isBad() &&
            p_passage->isPassable())
        {
            confirmedOpenPassages.push_back(p_passage);
        }
    }

    Plane *p_groundPlane = mpAtlas->GetBiggestGroundPlane();

    if (confirmedOpenPassages.empty() || p_groundPlane == nullptr ||
        p_groundPlane->isBad())
    {
        return freeSpaceClusters_World_m_in;
    }

    Eigen::Vector4d groundEquation_World =
        p_groundPlane->getGlobalEquation().coeffs();
    const double groundNormalNorm = groundEquation_World.head<3>().norm();

    if (!groundEquation_World.allFinite() || groundNormalNorm < 1e-8)
    {
        return freeSpaceClusters_World_m_in;
    }

    const Eigen::Vector3d groundNormal_World =
        groundEquation_World.head<3>() / groundNormalNorm;
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        skeletonEdges_World_m = mpAtlas->GetSkeletonEdges();

    if (skeletonEdges_World_m.empty())
    {
        return freeSpaceClusters_World_m_in;
    }

    const double associationDistance_m = static_cast<double>(
        partitionParameters.edgeVertexAssociationDistance_m);
    const double maximumAssociationSquaredDistance_m2 =
        associationDistance_m * associationDistance_m;
    const double minimumGraphCoverageRatio =
        static_cast<double>(partitionParameters.minimumGraphCoverageRatio);
    const double openingMargin_m =
        static_cast<double>(partitionParameters.openingMargin_m);
    const double minimumSideDistance_m =
        static_cast<double>(partitionParameters.minimumSideDistance_m);
    const std::size_t minimumClusterVertexCount =
        std::max<std::size_t>(sysParams->room_seg.min_cluster_vertices, 1U);
    std::vector<std::vector<Eigen::Vector3d>> partitionedClusters_World_m;

    for (const std::vector<Eigen::Vector3d> &cluster_World_m :
         freeSpaceClusters_World_m_in)
    {
        if (cluster_World_m.size() < minimumClusterVertexCount)
        {
            continue;
        }

        std::vector<std::vector<std::size_t>> adjacency(cluster_World_m.size());
        std::vector<bool> graphVertexWasObserved(cluster_World_m.size(), false);
        std::size_t       cutEdgeCount = 0U;

        const auto findNearestClusterVertex =
            [&cluster_World_m, maximumAssociationSquaredDistance_m2](
                const Eigen::Vector3d &edgePoint_World_m_in) -> std::size_t
        {
            std::size_t nearestVertexIndex = cluster_World_m.size();
            double      nearestSquaredDistance_m2 =
                maximumAssociationSquaredDistance_m2;

            for (std::size_t vertexIndex = 0U;
                 vertexIndex < cluster_World_m.size();
                 ++vertexIndex)
            {
                const double squaredDistance_m2 =
                    (cluster_World_m[vertexIndex] - edgePoint_World_m_in)
                        .squaredNorm();

                if (squaredDistance_m2 <= nearestSquaredDistance_m2)
                {
                    nearestSquaredDistance_m2 = squaredDistance_m2;
                    nearestVertexIndex        = vertexIndex;
                }
            }

            return nearestVertexIndex;
        };

        for (const auto &skeletonEdge_World_m : skeletonEdges_World_m)
        {
            const std::size_t startVertexIndex =
                findNearestClusterVertex(skeletonEdge_World_m.first);
            const std::size_t endVertexIndex =
                findNearestClusterVertex(skeletonEdge_World_m.second);

            if (startVertexIndex >= cluster_World_m.size() ||
                endVertexIndex >= cluster_World_m.size() ||
                startVertexIndex == endVertexIndex)
            {
                continue;
            }

            graphVertexWasObserved[startVertexIndex] = true;
            graphVertexWasObserved[endVertexIndex]   = true;

            const bool crossesConfirmedPassage =
                std::any_of(confirmedOpenPassages.begin(),
                            confirmedOpenPassages.end(),
                            [&skeletonEdge_World_m,
                             &groundNormal_World,
                             openingMargin_m,
                             minimumSideDistance_m](Passage *p_passage)
                            {
                                return segmentCrossesPassageOpening(
                                    skeletonEdge_World_m.first,
                                    skeletonEdge_World_m.second,
                                    p_passage,
                                    groundNormal_World,
                                    openingMargin_m,
                                    minimumSideDistance_m);
                            });

            if (crossesConfirmedPassage)
            {
                cutEdgeCount++;
                continue;
            }

            adjacency[startVertexIndex].push_back(endVertexIndex);
            adjacency[endVertexIndex].push_back(startVertexIndex);
        }

        const std::size_t observedGraphVertexCount =
            static_cast<std::size_t>(std::count(graphVertexWasObserved.begin(),
                                                graphVertexWasObserved.end(),
                                                true));
        const double graphCoverageRatio =
            static_cast<double>(observedGraphVertexCount) /
            static_cast<double>(cluster_World_m.size());

        /*
         * Preserve the upstream connected component when edge-to-vertex
         * reconstruction is underconstrained or no passage edge was cut.
         */
        if (graphCoverageRatio < minimumGraphCoverageRatio ||
            cutEdgeCount == 0U)
        {
            partitionedClusters_World_m.push_back(cluster_World_m);
            continue;
        }

        std::vector<bool> vertexWasVisited(cluster_World_m.size(), false);
        const std::size_t outputClusterCountBeforePartition =
            partitionedClusters_World_m.size();

        for (std::size_t seedVertexIndex = 0U;
             seedVertexIndex < cluster_World_m.size();
             ++seedVertexIndex)
        {
            if (vertexWasVisited[seedVertexIndex] ||
                !graphVertexWasObserved[seedVertexIndex])
            {
                continue;
            }

            std::vector<std::size_t>     pendingVertexIndices{seedVertexIndex};
            std::vector<Eigen::Vector3d> component_World_m;
            vertexWasVisited[seedVertexIndex] = true;

            while (!pendingVertexIndices.empty())
            {
                const std::size_t vertexIndex = pendingVertexIndices.back();
                pendingVertexIndices.pop_back();
                component_World_m.push_back(cluster_World_m[vertexIndex]);

                for (const std::size_t neighbourIndex : adjacency[vertexIndex])
                {
                    if (!vertexWasVisited[neighbourIndex])
                    {
                        vertexWasVisited[neighbourIndex] = true;
                        pendingVertexIndices.push_back(neighbourIndex);
                    }
                }
            }

            if (component_World_m.size() >= minimumClusterVertexCount)
            {
                partitionedClusters_World_m.push_back(
                    std::move(component_World_m));
            }
        }

        /*
         * Do not lose a valid upstream component when every reconstructed
         * child is below the room detector's minimum size. The passage still
         * remains in the semantic graph and will be retried as Voxblox gains
         * more observed free-space vertices.
         */
        if (partitionedClusters_World_m.size() ==
            outputClusterCountBeforePartition)
        {
            partitionedClusters_World_m.push_back(cluster_World_m);
        }
    }

    return partitionedClusters_World_m;
}

void SemanticsManager::detachWallsBeyondConfirmedPassages(void)
{
    const SystemParams::room_seg::PassagePartition &partitionParameters =
        sysParams->room_seg.passagePartition;

    if (!partitionParameters.enabled ||
        !partitionParameters.detachWallsBeyondPassages)
    {
        return;
    }

    Plane *p_groundPlane = mpAtlas->GetBiggestGroundPlane();

    if (p_groundPlane == nullptr || p_groundPlane->isBad())
    {
        return;
    }

    const Eigen::Vector4d groundEquation_World =
        p_groundPlane->getGlobalEquation().coeffs();
    const double groundNormalNorm = groundEquation_World.head<3>().norm();

    if (!groundEquation_World.allFinite() || groundNormalNorm < 1e-8)
    {
        return;
    }

    const Eigen::Vector3d groundNormal_World =
        groundEquation_World.head<3>() / groundNormalNorm;
    const double openingMargin_m =
        static_cast<double>(partitionParameters.openingMargin_m);
    const double minimumSideDistance_m = static_cast<double>(
        partitionParameters.wallCentroidMinimumSideDistance_m);

    std::vector<Passage *> confirmedOpenPassages;

    for (Passage *p_passage : mpAtlas->GetAllPassages())
    {
        if (p_passage != nullptr && p_passage->isPassable())
        {
            confirmedOpenPassages.push_back(p_passage);
        }
    }

    std::sort(confirmedOpenPassages.begin(),
              confirmedOpenPassages.end(),
              [](const Passage *p_first, const Passage *p_second)
              { return p_first->getId() < p_second->getId(); });

    if (confirmedOpenPassages.empty())
    {
        return;
    }

    const std::vector<Room *> allRooms = mpAtlas->GetAllRooms();

    for (Room *p_room : allRooms)
    {
        if (p_room == nullptr || p_room->isBad())
        {
            continue;
        }

        const Eigen::Vector3d roomCentroid_World_m = p_room->getCentroid();

        if (!roomCentroid_World_m.allFinite())
        {
            continue;
        }

        for (Plane *p_wall : p_room->getWalls())
        {
            if (p_wall == nullptr || p_wall->isBad())
            {
                continue;
            }

            const Eigen::Vector3d wallCentroid_World_m =
                p_wall->getCentroid().cast<double>();

            if (!wallCentroid_World_m.allFinite())
            {
                continue;
            }

            Passage *p_separatingPassage = nullptr;

            for (Passage *p_passage : confirmedOpenPassages)
            {
                /*
                 * A passage's own supporting wall lies on its aperture plane,
                 * so it cannot satisfy the opposite-side distance test. This
                 * preserves the wall-passage relationship while rejecting a
                 * different wall reached only through that opening.
                 */
                if (segmentCrossesPassageOpening(roomCentroid_World_m,
                                                 wallCentroid_World_m,
                                                 p_passage,
                                                 groundNormal_World,
                                                 openingMargin_m,
                                                 minimumSideDistance_m))
                {
                    p_separatingPassage = p_passage;
                    break;
                }
            }

            if (p_separatingPassage == nullptr)
            {
                continue;
            }

            Room *p_confirmedOwner = nullptr;
            for (Room *p_otherRoom : allRooms)
            {
                if (p_otherRoom == nullptr || p_otherRoom == p_room ||
                    p_otherRoom->isBad() ||
                    p_otherRoom->getRoomVariant() ==
                        Room::roomVariant::UNDEFINED)
                {
                    continue;
                }

                const std::vector<Plane *> otherWalls = p_otherRoom->getWalls();
                if (std::find(otherWalls.begin(), otherWalls.end(), p_wall) !=
                    otherWalls.end())
                {
                    p_confirmedOwner = p_otherRoom;
                    break;
                }
            }

            Room *p_farSideRoom = p_separatingPassage->getProspectiveRoom();
            if (p_farSideRoom != nullptr &&
                (p_farSideRoom->isBad() || p_farSideRoom == p_room))
            {
                p_farSideRoom = nullptr;
            }

            if (!p_room->removeWall(p_wall))
            {
                continue;
            }

            if (p_confirmedOwner != nullptr &&
                p_confirmedOwner != p_farSideRoom)
            {
                std::cout << "[SemMgr] Detached far-side Wall#"
                          << p_wall->getId() << " from Room#" << p_room->getId()
                          << "; retained distinct confirmed owner Room#"
                          << p_confirmedOwner->getId() << "." << std::endl;
                continue;
            }

            if (p_farSideRoom != nullptr)
            {
                p_farSideRoom->setWalls(p_wall);
                if (mpAtlas->GetRoomWallPlaneById(p_wall->getId()) == nullptr)
                {
                    mpAtlas->AddRoomWallPlane(p_wall);
                }
                std::cout << "[SemMgr] Redirected far-side Wall#"
                          << p_wall->getId() << " from Room#" << p_room->getId()
                          << " through Passage#" << p_separatingPassage->getId()
                          << " to stable Room#" << p_farSideRoom->getId() << "."
                          << std::endl;
                continue;
            }

            std::cout << "[SemMgr] Detached far-side Wall#" << p_wall->getId()
                      << " from Room#" << p_room->getId() << "; Passage#"
                      << p_separatingPassage->getId()
                      << " has no stable far-side room, so the wall remains "
                         "orphaned."
                      << std::endl;
        }
    }
}

SemanticsManager::PassageSideEnforcementOutcome
    SemanticsManager::enforcePassageApertureBackstop(
        Room                         *p_room_inout,
        Plane                        *p_wall_in,
        const std::vector<Passage *> &allPassages_in,
        const Eigen::Vector3d        &groundNormal_World_in)
{
    if (p_room_inout == nullptr || p_room_inout->isBad() ||
        p_wall_in == nullptr || p_wall_in->isBad())
    {
        return PassageSideEnforcementOutcome::NoViolation;
    }

    for (Passage *p_passage : allPassages_in)
    {
        if (p_passage == nullptr || p_passage->isBad())
        {
            continue;
        }

        const double minimumSideDistance_m = static_cast<double>(
            sysParams->room_seg.passagePartition.minimumSideDistance_m);

        /* B2 fix: segmentCrossesPassageOpening silently reports "no crossing"
         * whenever its segment-start point sits within minimumSideDistance_m
         * of the passage plane -- which the room's own centroid commonly
         * does for a sparsely-observed room. Rather than let that ambiguity
         * masquerade as "not crossing" (silently admitting a genuine
         * far-side wall to the near room), substitute a point pushed out
         * along the passage's known near side when the raw centroid is too
         * close to call. Only apply this when a reliable near-side direction
         * is actually available (Passage::KnownSideProvenance, built up from
         * other walls' admission history for this passage): the ambiguous
         * centroid's own residual sign is noise, not a signal, and guessing
         * from it can just as easily push the synthesized point to the
         * WRONG side as the right one -- worse than the original silent
         * no-crossing report, not better. With no known side yet, this
         * degenerate case is left exactly as before the fix. */
        Eigen::Vector3d segmentStart_World_m = p_room_inout->getCentroid();
        Eigen::Vector4d passageEquation_World =
            p_passage->getGlobalEquation().coeffs();
        const double passageNormalNorm = passageEquation_World.head<3>().norm();
        if (passageEquation_World.allFinite() && passageNormalNorm > 1e-8)
        {
            passageEquation_World /= passageNormalNorm;
            const Eigen::Vector3d passageNormal_World =
                passageEquation_World.head<3>();
            const double roomCentroidSide_m =
                passageNormal_World.dot(segmentStart_World_m) +
                passageEquation_World(3);

            if (std::abs(roomCentroidSide_m) < minimumSideDistance_m)
            {
                const Passage::KnownSideProvenance knownSide =
                    p_passage->getKnownSideProvenance();
                if (knownSide.hasDirection())
                {
                    segmentStart_World_m = p_passage->getCentroid() +
                                           (minimumSideDistance_m * 2.0) *
                                               knownSide.direction_World;
                }
            }
        }

        if (!segmentCrossesPassageOpening(
                segmentStart_World_m,
                p_wall_in->getCentroid().cast<double>(),
                p_passage,
                groundNormal_World_in,
                static_cast<double>(
                    sysParams->room_seg.passagePartition.openingMargin_m),
                minimumSideDistance_m))
        {
            continue;
        }

        /* Never steal a wall already claimed by a distinct confirmed room. */
        bool ownedByConfirmedRoom = false;
        for (ORB_SLAM3::Room *p_other : mpAtlas->GetAllRooms())
        {
            if (p_other == nullptr || p_other->isBad() ||
                p_other == p_room_inout ||
                p_other->getRoomVariant() ==
                    ORB_SLAM3::Room::roomVariant::UNDEFINED)
            {
                continue;
            }
            const std::vector<Plane *> otherWalls = p_other->getWalls();
            if (std::find(otherWalls.begin(), otherWalls.end(), p_wall_in) !=
                otherWalls.end())
            {
                ownedByConfirmedRoom = true;
                break;
            }
        }
        if (ownedByConfirmedRoom)
        {
            /* A distinct confirmed room already owns this wall. Leave it on
             * that owner rather than re-binding it to the near room. */
            p_room_inout->removeWall(p_wall_in);
            return PassageSideEnforcementOutcome::RemovedUnbound;
        }

        ORB_SLAM3::Room *p_prospective = p_passage->getProspectiveRoom();

        /* The wall is already sitting in the room this exact aperture
         * crossing would route it to -- there is nothing to enforce. Live-
         * observed 2026-09-04: this branch previously fell through the same
         * eviction as "no prospective room exists at all", so a wall that
         * had ALREADY been correctly rerouted to its far-side prospective
         * kept getting evicted from it every single cycle this sweep re-ran
         * (enforcePassageSideInvariant runs every Run() cycle), leaving it
         * permanently homeless even though Passage#0's own SemMgrSummary
         * line showed a perfectly live prospectiveRoom the whole time. */
        if (p_prospective == p_room_inout)
        {
            continue;
        }

        if (p_prospective == nullptr || p_prospective->isBad())
        {
            p_room_inout->removeWall(p_wall_in);
            std::cout << "[SemMgr] Far-side Wall#" << p_wall_in->getId()
                      << " at Passage#" << p_passage->getId()
                      << " has no opposite stable room; left unbound."
                      << std::endl;
            return PassageSideEnforcementOutcome::RemovedUnbound;
        }

        p_room_inout->removeWall(p_wall_in);
        if (mpAtlas->GetRoomWallPlaneById(p_wall_in->getId()) == nullptr)
        {
            mpAtlas->AddRoomWallPlane(p_wall_in);
        }
        p_prospective->setWalls(p_wall_in);
        std::cout << "[SemMgr] Redirected far-side Wall#" << p_wall_in->getId()
                  << " to prospective Room#" << p_prospective->getId() << "."
                  << std::endl;
        return PassageSideEnforcementOutcome::Rerouted;
    }

    /* No CONFIRMED passage caught this wall -- but confirmation lags real
     * exploration time behind the skeleton-crossing evidence itself (see
     * segmentCrossesOpenPassageEvidence's own comment). Re-run the same
     * aperture test against each pending hypothesis so this continuous
     * re-check sweep (enforcePassageSideInvariant) catches a wall that slips
     * in during that window just as reliably as it catches one that slips in
     * against an already-confirmed passage. */
    for (const OpenPassageEvidence &evidence : openPassageEvidence_)
    {
        if (!segmentCrossesOpenPassageEvidence(
                p_room_inout->getCentroid(),
                p_wall_in->getCentroid().cast<double>(),
                evidence.p_supportingWall,
                evidence.centroid_World_m,
                evidence.openingRadius_m,
                evidence.heightSpan_m,
                groundNormal_World_in,
                static_cast<double>(
                    sysParams->room_seg.passagePartition.openingMargin_m),
                static_cast<double>(sysParams->room_seg.passagePartition
                                        .minimumSideDistance_m)))
        {
            continue;
        }

        bool ownedByConfirmedRoom = false;
        for (ORB_SLAM3::Room *p_other : mpAtlas->GetAllRooms())
        {
            if (p_other == nullptr || p_other->isBad() ||
                p_other == p_room_inout ||
                p_other->getRoomVariant() ==
                    ORB_SLAM3::Room::roomVariant::UNDEFINED)
            {
                continue;
            }
            const std::vector<Plane *> otherWalls = p_other->getWalls();
            if (std::find(otherWalls.begin(), otherWalls.end(), p_wall_in) !=
                otherWalls.end())
            {
                ownedByConfirmedRoom = true;
                break;
            }
        }
        if (ownedByConfirmedRoom)
        {
            continue;
        }

        p_room_inout->removeWall(p_wall_in);
        std::cout << "[SemMgr] Far-side Wall#" << p_wall_in->getId()
                  << " crosses an unconfirmed passage opening (evidence at "
                     "wall "
                  << (evidence.p_supportingWall != nullptr
                          ? evidence.p_supportingWall->getId()
                          : -1)
                  << "); removed from Room#" << p_room_inout->getId()
                  << " pending confirmation." << std::endl;
        return PassageSideEnforcementOutcome::RemovedUnbound;
    }

    return PassageSideEnforcementOutcome::NoViolation;
}

bool SemanticsManager::isWallFaceForeignToRoom(Room  *p_room_in,
                                               Plane *p_wall_in)
{
    if (p_room_in == nullptr || p_room_in->isBad() || p_wall_in == nullptr ||
        p_wall_in->isBad())
    {
        return false;
    }

    /* The face's identity: the camera position it was first observed from.
     * Only the side of a physical surface turned toward a camera can be
     * seen, so this fixes which of the wall's two faces this plane is -- and
     * therefore which room it bounds -- for the plane's whole lifetime. */
    const std::optional<Eigen::Vector3d> observationOrigin_World_m =
        p_wall_in->getObservationOrigin_World();

    if (!observationOrigin_World_m.has_value() ||
        !observationOrigin_World_m->allFinite())
    {
        /* Planes created before the stamp existed carry no face identity;
         * make no claim rather than a wrong one. */
        return false;
    }

    Eigen::Vector4d equation_World = p_wall_in->getGlobalEquation().coeffs();
    const double    normalNorm     = equation_World.head<3>().norm();

    if (!equation_World.allFinite() || normalNorm <= 1e-8)
    {
        return false;
    }

    equation_World /= normalNorm;

    const double observedSide_m =
        equation_World.head<3>().dot(observationOrigin_World_m.value()) +
        equation_World(3);
    const double roomSide_m =
        equation_World.head<3>().dot(p_room_in->getCentroid().cast<double>()) +
        equation_World(3);

    if (!std::isfinite(observedSide_m) || !std::isfinite(roomSide_m))
    {
        return false;
    }

    /* Matches the resolvable-side floor already used by the association path
     * (Utils::associatePlanes) and Plane::getObservationSideSnapshot(): a
     * position essentially ON the plane does not identify a side. */
    constexpr double minimumResolvableSide_m = 0.10;

    if (std::abs(observedSide_m) < minimumResolvableSide_m ||
        std::abs(roomSide_m) < minimumResolvableSide_m)
    {
        return false;
    }

    /* Opposite sides: the camera that produced this face was on the far side
     * of it from this room, so this is the neighbouring room's face. The
     * room's own face of the same physical wall is a separate plane, which
     * it has evidently not observed yet. */
    return observedSide_m * roomSide_m < 0.0;
}

void SemanticsManager::enforcePassageSideInvariant(void)
{
    /* "Continuously checking the current state of the sgraph to make sure
     * the rules are followed" (as opposed to only at the moment a wall is
     * newly admitted): associateAllWallsToRooms() only ever revisits ORPHAN
     * walls (a wall that already has a room is skipped outright), so a wall
     * admitted before a relevant passage's aperture became confidently
     * resolvable would otherwise never be re-examined again. This sweep
     * re-applies both the passage-aperture backstop and the wall-face
     * ownership rule to every wall every room currently owns, every cycle.
     *
     * Note the two are re-checked here for different reasons.
     * isWallFaceForeignToRoom() is itself stable -- face identity is stamped
     * at observation and does not drift -- but the ROOM side of the
     * comparison does move: a FREE_SPACE room's centroid is recomputed every
     * cycle as its wall-centroid mean, so a room that grows walls can
     * migrate across a face it once legitimately sat beside. The aperture
     * backstop is re-checked because passage geometry itself sharpens over
     * time. */
    Plane          *p_groundPlane      = mpAtlas->GetBiggestGroundPlane();
    Eigen::Vector3d groundNormal_World = Eigen::Vector3d::Zero();
    if (p_groundPlane != nullptr && !p_groundPlane->isBad())
    {
        const Eigen::Vector4d groundEq =
            p_groundPlane->getGlobalEquation().coeffs();
        const double groundNorm = groundEq.head<3>().norm();
        if (groundEq.allFinite() && groundNorm > 1e-8)
        {
            groundNormal_World = groundEq.head<3>() / groundNorm;
        }
    }

    const std::vector<Passage *> allPassages = mpAtlas->GetAllPassages();

    for (Room *p_room : mpAtlas->GetAllRooms())
    {
        if (p_room == nullptr || p_room->isBad())
        {
            continue;
        }

        /* Copy: both backstops below may call Room::removeWall(), which
         * would invalidate an in-progress iteration over the room's own
         * live wall vector. */
        const std::vector<Plane *> roomWalls = p_room->getWalls();
        for (Plane *p_wall : roomWalls)
        {
            if (p_wall == nullptr || p_wall->isBad())
            {
                continue;
            }

            /* Prospective-placement exemption: a far-side wall that the
             * aperture backstop deliberately routed into this prospective
             * room must not be evicted from it by the face check below
             * (live-observed churn: remove-then-reroute every cycle). The
             * exemption is earned only when the same aperture test that
             * routes the wall still places it here, synthesized from the
             * passage's known near-side direction exactly as the backstop
             * does. */
            bool wallRoutedToProspective = false;
            for (Passage *p_exemptPassage : allPassages)
            {
                if (p_exemptPassage == nullptr || p_exemptPassage->isBad() ||
                    p_exemptPassage->getProspectiveRoom() != p_room)
                {
                    continue;
                }
                const Passage::KnownSideProvenance knownSide =
                    p_exemptPassage->getKnownSideProvenance();
                if (!knownSide.hasDirection())
                {
                    continue;
                }
                const double minimumSideDistance_m = static_cast<double>(
                    sysParams->room_seg.passagePartition.minimumSideDistance_m);
                const Eigen::Vector3d knownSidePoint_World_m =
                    p_exemptPassage->getCentroid() +
                    (minimumSideDistance_m * 2.0) * knownSide.direction_World;
                if (segmentCrossesPassageOpening(
                        knownSidePoint_World_m,
                        p_wall->getCentroid().cast<double>(),
                        p_exemptPassage,
                        groundNormal_World,
                        static_cast<double>(sysParams->room_seg.passagePartition
                                                .openingMargin_m),
                        minimumSideDistance_m))
                {
                    wallRoutedToProspective = true;
                    break;
                }
            }
            if (wallRoutedToProspective)
            {
                continue;
            }

            if (isWallFaceForeignToRoom(p_room, p_wall))
            {
                p_room->removeWall(p_wall);
                std::cout << "[SemMgr] Wall#" << p_wall->getId()
                          << " removed from Room#" << p_room->getId()
                          << ": this face was observed from the opposite side, "
                             "so it bounds the neighbouring room."
                          << std::endl;
                continue;
            }

            if (!allPassages.empty())
            {
                enforcePassageApertureBackstop(p_room,
                                               p_wall,
                                               allPassages,
                                               groundNormal_World);
            }
        }
    }
}

bool SemanticsManager::admitWallToRoom(Room  *p_room_inout,
                                       Plane *p_candidateWall_in)
{
    if (p_room_inout == nullptr || p_room_inout->isBad() ||
        p_candidateWall_in == nullptr || p_candidateWall_in->isBad())
    {
        return false;
    }

    const std::vector<Plane *> existingWalls = p_room_inout->getWalls();
    const bool                 alreadyPresent =
        std::find(existingWalls.begin(),
                  existingWalls.end(),
                  p_candidateWall_in) != existingWalls.end();

    Plane          *p_farSideGroundPlane = mpAtlas->GetBiggestGroundPlane();
    Eigen::Vector3d farSideGroundNormal_World = Eigen::Vector3d::Zero();
    if (p_farSideGroundPlane != nullptr && !p_farSideGroundPlane->isBad())
    {
        const Eigen::Vector4d groundEq =
            p_farSideGroundPlane->getGlobalEquation().coeffs();
        const double groundNorm = groundEq.head<3>().norm();
        if (groundEq.allFinite() && groundNorm > 1e-8)
        {
            farSideGroundNormal_World = groundEq.head<3>() / groundNorm;
        }
    }

    std::vector<Passage *> allPassages = mpAtlas->GetAllPassages();
    std::sort(allPassages.begin(),
              allPassages.end(),
              [](const Passage *p_first, const Passage *p_second)
              {
                  if (p_first == nullptr)
                  {
                      return false;
                  }
                  if (p_second == nullptr)
                  {
                      return true;
                  }
                  return p_first->getId() < p_second->getId();
              });

    switch (enforcePassageApertureBackstop(p_room_inout,
                                           p_candidateWall_in,
                                           allPassages,
                                           farSideGroundNormal_World))
    {
    case PassageSideEnforcementOutcome::RemovedUnbound:
        return false;
    case PassageSideEnforcementOutcome::Rerouted:
        return true;
    case PassageSideEnforcementOutcome::NoViolation:
        break;
    }

    if (alreadyPresent)
    {
        return true;
    }

    if (!evaluateWallAdmissionEvidence(p_candidateWall_in,
                                       sysParams,
                                       farSideGroundNormal_World)
             .admissible)
    {
        return false;
    }

    if (isWallFaceForeignToRoom(p_room_inout, p_candidateWall_in))
    {
        std::cout << "[SemMgr] Wall#" << p_candidateWall_in->getId()
                  << " rejected from Room#" << p_room_inout->getId()
                  << ": this face was observed from the opposite side, so it "
                     "bounds the neighbouring room."
                  << std::endl;
        return false;
    }

    const SystemParams::room_seg::BoundaryTopology &topologyParameters =
        sysParams->room_seg.boundaryTopology;
    Plane *p_groundPlane = mpAtlas->GetBiggestGroundPlane();

    if (!topologyParameters.enabled || p_groundPlane == nullptr ||
        p_groundPlane->isBad())
    {
        p_room_inout->setWalls(p_candidateWall_in);
        return true;
    }

    Eigen::Vector4d groundEquation_World =
        p_groundPlane->getGlobalEquation().coeffs();
    const double groundNormalNorm = groundEquation_World.head<3>().norm();

    if (!groundEquation_World.allFinite() || groundNormalNorm < 1e-8)
    {
        p_room_inout->setWalls(p_candidateWall_in);
        return true;
    }

    const Eigen::Vector3d groundNormal_World =
        groundEquation_World.head<3>() / groundNormalNorm;
    const Eigen::Vector3d groundAxisU_World =
        groundNormal_World.unitOrthogonal().normalized();
    const Eigen::Vector3d groundAxisV_World =
        groundNormal_World.cross(groundAxisU_World).normalized();
    FiniteWallSegment2d candidateSegment;

    if (!buildFiniteWallSegment2d(p_candidateWall_in,
                                  groundNormal_World,
                                  groundAxisU_World,
                                  groundAxisV_World,
                                  topologyParameters.endpointTrimRatio,
                                  topologyParameters.minimumWallLength_m,
                                  candidateSegment))
    {
        p_room_inout->setWalls(p_candidateWall_in);
        return true;
    }

    std::vector<Plane *> weakerClashingWalls;

    for (Plane *p_existingWall : existingWalls)
    {
        FiniteWallSegment2d existingSegment;

        if (!buildFiniteWallSegment2d(p_existingWall,
                                      groundNormal_World,
                                      groundAxisU_World,
                                      groundAxisV_World,
                                      topologyParameters.endpointTrimRatio,
                                      topologyParameters.minimumWallLength_m,
                                      existingSegment))
        {
            continue;
        }

        Eigen::Vector2d intersection_World_m;
        double          candidateParameter = 0.0;
        double          existingParameter  = 0.0;

        if (!intersectSupportingLines(candidateSegment,
                                      existingSegment,
                                      intersection_World_m,
                                      candidateParameter,
                                      existingParameter) ||
            candidateParameter < 0.0 || candidateParameter > 1.0 ||
            existingParameter < 0.0 || existingParameter > 1.0)
        {
            continue;
        }

        const double candidateInteriorDistance_m =
            std::min(candidateParameter, 1.0 - candidateParameter) *
            candidateSegment.length_m;
        const double existingInteriorDistance_m =
            std::min(existingParameter, 1.0 - existingParameter) *
            existingSegment.length_m;

        if (std::max(candidateInteriorDistance_m, existingInteriorDistance_m) <=
            topologyParameters.maximumInteriorIntersection_m)
        {
            continue;
        }

        const double candidateSupport = candidateSegment.supportScore;
        const double existingSupport  = existingSegment.supportScore;
        const double weakerSupport =
            std::max(std::min(candidateSupport, existingSupport), 1e-8);
        const double supportRatio =
            std::max(candidateSupport, existingSupport) / weakerSupport;

        /*
         * Ambiguous or weaker candidates stay outside this room. Their Plane
         * objects remain valid and the orphan-wall pass can attach them to a
         * different room as additional free-space evidence becomes available.
         */
        if (supportRatio < topologyParameters.decisiveConflictSupportRatio ||
            candidateSupport <= existingSupport)
        {
            return false;
        }

        weakerClashingWalls.push_back(p_existingWall);
    }

    for (Plane *p_weakerWall : weakerClashingWalls)
    {
        if (p_room_inout->removeWall(p_weakerWall))
        {
            std::cout << "[SemMgr] Replaced clashing Wall#"
                      << p_weakerWall->getId() << " in Room#"
                      << p_room_inout->getId() << " with stronger Wall#"
                      << p_candidateWall_in->getId() << "." << std::endl;
        }
    }

    /* Cross-room check: real walls only meet at shared corners -- a
     * candidate whose finite segment decisively crosses the interior of
     * another room's already-admitted wall is a modeling error, not a
     * legitimate admission. (Twin faces from reconcileWallFacePairs() are
     * parallel by construction and cannot trigger this.) Reject outright
     * rather than perturb the foreign room's wall: ownership of an
     * already-admitted wall is never taken by another room's admission
     * attempt, only by enforceUniqueWallOwnership()/validateRoomBoundaries()'
     * own intra-room repair. */
    for (ORB_SLAM3::Room *p_otherRoom : mpAtlas->GetAllRooms())
    {
        if (p_otherRoom == nullptr || p_otherRoom->isBad() ||
            p_otherRoom == p_room_inout)
        {
            continue;
        }

        for (Plane *p_otherWall : p_otherRoom->getWalls())
        {
            if (p_otherWall == nullptr || p_otherWall == p_candidateWall_in)
            {
                continue;
            }

            FiniteWallSegment2d otherRoomSegment;

            if (!buildFiniteWallSegment2d(
                    p_otherWall,
                    groundNormal_World,
                    groundAxisU_World,
                    groundAxisV_World,
                    topologyParameters.endpointTrimRatio,
                    topologyParameters.minimumWallLength_m,
                    otherRoomSegment))
            {
                continue;
            }

            Eigen::Vector2d otherIntersection_World_m;
            double          candidateOtherParameter = 0.0;
            double          otherRoomParameter      = 0.0;

            if (!intersectSupportingLines(candidateSegment,
                                          otherRoomSegment,
                                          otherIntersection_World_m,
                                          candidateOtherParameter,
                                          otherRoomParameter) ||
                candidateOtherParameter < 0.0 ||
                candidateOtherParameter > 1.0 || otherRoomParameter < 0.0 ||
                otherRoomParameter > 1.0)
            {
                continue;
            }

            const double candidateOtherInteriorDistance_m =
                std::min(candidateOtherParameter,
                         1.0 - candidateOtherParameter) *
                candidateSegment.length_m;
            const double otherRoomInteriorDistance_m =
                std::min(otherRoomParameter, 1.0 - otherRoomParameter) *
                otherRoomSegment.length_m;

            if (std::max(candidateOtherInteriorDistance_m,
                         otherRoomInteriorDistance_m) <=
                topologyParameters.maximumInteriorIntersection_m)
            {
                continue;
            }

            std::cout << "[SemMgr] Wall#" << p_candidateWall_in->getId()
                      << " rejected from Room#" << p_room_inout->getId()
                      << ": crosses Room#" << p_otherRoom->getId()
                      << "'s already-admitted Wall#" << p_otherWall->getId()
                      << "." << std::endl;
            return false;
        }
    }

    p_room_inout->setWalls(p_candidateWall_in);
    return true;
}

void SemanticsManager::detectRoom_FreeSpaceCluster(void)
{
    /* Extract latest skeleton cluster */
    const std::vector<std::vector<Eigen::Vector3d>> clusters =
        partitionFreeSpaceAtPassages(getLatestSkeletonCluster());

    /* If cluster is empty then return */
    if (clusters.empty())
    {
        return;
    }

    /* Extract all planes from the map */
    const std::vector<ORB_SLAM3::Plane *> allPlanes = mpAtlas->GetAllPlanes();

    /* Create a list of all the walls there are in the SGraph */
    std::vector<ORB_SLAM3::Plane *> allWalls;
    allWalls.reserve(allPlanes.size());

    /* Ground-aligned axes for evaluateWallAdmissionEvidence's height/width
     * gate (see the comment at its definition for why this must be
     * ground-anchored rather than an arbitrary in-plane axis). */
    Plane          *p_groundPlaneForEvidence = mpAtlas->GetBiggestGroundPlane();
    Eigen::Vector3d groundNormalForEvidence_World = Eigen::Vector3d::Zero();
    if (p_groundPlaneForEvidence != nullptr &&
        !p_groundPlaneForEvidence->isBad())
    {
        const Eigen::Vector4d groundEq =
            p_groundPlaneForEvidence->getGlobalEquation().coeffs();
        const double groundNorm = groundEq.head<3>().norm();
        if (groundEq.allFinite() && groundNorm > 1e-8)
        {
            groundNormalForEvidence_World = groundEq.head<3>() / groundNorm;
        }
    }

    /* For every plane, extract walls */
    for (ORB_SLAM3::Plane *plane : allPlanes)
    {
        /* Skip bad planes */
        if (plane == nullptr || plane->isBad())
        {
            continue;
        }

        /* Append valid wall planes to list */
        if (evaluateWallAdmissionEvidence(plane,
                                          sysParams,
                                          groundNormalForEvidence_World)
                .admissible)
        {
            allWalls.push_back(plane);
        }
    }

    /* If there are no walls, then return */
    if (allWalls.empty())
    {
        return;
    }

    /* Track room IDs matched in this cycle to avoid double-matching */
    std::unordered_set<int> matchedRoomIds;

    /* Iterate through all the clusters */
    for (std::size_t clusterId = 0; clusterId < clusters.size(); clusterId++)
    {
        /* Extract cluster */
        const std::vector<Eigen::Vector3d> &cluster = clusters[clusterId];

        /* If cluster is empty skip */
        if (cluster.empty())
        {
            continue;
        }

        /* Extract the cluster centroid */
        const Eigen::Vector3d clusterCentroid =
            Utils::computeCentroidFromPoints(cluster);

        /* Initialize a list of planes to track the closest walls */
        std::vector<ORB_SLAM3::Plane *> closestWalls;
        closestWalls.reserve(allWalls.size());

        /* For each wall */
        for (ORB_SLAM3::Plane *wall : allWalls)
        {
            /* Skip wall if it is bad */
            if (wall == nullptr || wall->isBad())
            {
                continue;
            }

            /* Extract the point cloud for the wall */
            const Plane::GeometrySnapshot wallGeometry =
                wall->getGeometrySnapshot();
            const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr wallCloud =
                wallGeometry.supportCloud;

            /* Skip wall if the point cloud is invalid */
            if (wallCloud == nullptr || wallCloud->empty())
            {
                continue;
            }

            /* Extract the centroid of the wall */
            const Eigen::Vector3d wallCentroid = wallGeometry.centroid_World_m;

            /* Find the distance from the wall centroid and cluster centroid */
            const double centroidDistance =
                (wallCentroid - clusterCentroid).norm();

            /*!
             * Use centroid distance only as a coarse rejection condition.
             *
             * @note        A long wall may have a centroid far from the room
             *              centre while still forming a valid boundary of the
             *              room.
             */
            const double coarseCentroidDistanceThreshold =
                2.0 * static_cast<double>(
                          sysParams->room_seg
                              .cluster_centroid_wall_centroid_distance_thresh);

            if (centroidDistance >= coarseCentroidDistanceThreshold)
            {
                continue;
            }

            /* Extract plane equation for the wall */
            Eigen::Vector4d wallEquation = wall->getGlobalEquation().coeffs();

            /* Extract the norm of the normal of the wall */
            const double normalMagnitude = wallEquation.head<3>().norm();

            /* Skip wall if norm is invalid */
            if (!std::isfinite(normalMagnitude) || normalMagnitude < 1e-8)
            {
                continue;
            }

            /* Find the normal vector of the wall */
            const Eigen::Vector3d wallNormalVector = wallEquation.head<3>();

            /* Find distance to normal */
            const double wallDistance = wallEquation(3) / normalMagnitude;

            /* Find the normalized vector */
            const Eigen::Vector3d wallNormUnitVector =
                wallNormalVector / normalMagnitude;

            /* Find an axis U which tangental to the wall plane */
            const Eigen::Vector3d axisU =
                wallNormUnitVector.unitOrthogonal().normalized();

            /* Find orthogonal axis to make handed axis with U and normal */
            const Eigen::Vector3d axisV =
                wallNormUnitVector.cross(axisU).normalized();

            /* Measure finite wall bounds in the same basis used below. */
            std::size_t validWallPoints = 0;
            double      minimumWallU_m  = std::numeric_limits<double>::max();
            double      maximumWallU_m  = std::numeric_limits<double>::lowest();
            double      minimumWallV_m  = std::numeric_limits<double>::max();
            double      maximumWallV_m  = std::numeric_limits<double>::lowest();

            /* Iterate through each point in the wall point cloud */
            for (const pcl::PointXYZRGBA &point : wallCloud->points)
            {
                /* If point is invalid, skip */
                if (!pcl::isFinite(point))
                {
                    continue;
                }

                const Eigen::Vector3d wallPoint_World_m(
                    static_cast<double>(point.x),
                    static_cast<double>(point.y),
                    static_cast<double>(point.z));

                const Eigen::Vector3d wallPointRelToCentroid_World_m =
                    wallPoint_World_m - wallCentroid;

                const double wallPointU_m =
                    wallPointRelToCentroid_World_m.dot(axisU);
                const double wallPointV_m =
                    wallPointRelToCentroid_World_m.dot(axisV);

                minimumWallU_m = std::min(minimumWallU_m, wallPointU_m);
                maximumWallU_m = std::max(maximumWallU_m, wallPointU_m);
                minimumWallV_m = std::min(minimumWallV_m, wallPointV_m);
                maximumWallV_m = std::max(maximumWallV_m, wallPointV_m);

                validWallPoints++;
            }

            /* A finite wall cannot be measured without a valid cloud point. */
            if (validWallPoints == 0)
            {
                continue;
            }

            /* Confirm wall bounds are valid, otherwise skip */
            if (!std::isfinite(minimumWallU_m) ||
                !std::isfinite(maximumWallU_m) ||
                !std::isfinite(minimumWallV_m) ||
                !std::isfinite(maximumWallV_m))
            {
                continue;
            }

            const double wallExtentU_m = maximumWallU_m - minimumWallU_m;
            const double wallExtentV_m = maximumWallV_m - minimumWallV_m;

            /* Skip small fragments without erasing their semantic evidence. */
            if (wallExtentU_m < sysParams->room_seg.minimumFiniteWallExtent_m ||
                wallExtentV_m < sysParams->room_seg.minimumFiniteWallExtent_m)
            {
                continue;
            }

            /*!
             * Number of close points whose projections lie inside the finite
             * wall patch.
             */
            std::size_t supportedPointCount = 0;
            std::size_t nearbyPointCount    = 0;

            /* Init variable to track minimum distance from wall and cluster */
            double minimumPlaneDistance = std::numeric_limits<double>::max();

            /* Iterate through each point in cluster to find point in wall */
            for (const Eigen::Vector3d &clusterPoint : cluster)
            {
                /* Find the signed distance along wall normal*/
                const double signedPlaneDistance =
                    wallNormUnitVector.dot(clusterPoint) + wallDistance;

                /* Find the absolute value of the plane distance */
                const double planeDistance = std::abs(signedPlaneDistance);

                /* Update if the disatance is smaller than currently tracked */
                minimumPlaneDistance =
                    std::min(minimumPlaneDistance, planeDistance);

                /* If the plane distance is larger than threshold, skip point */
                if (planeDistance >=
                    sysParams->room_seg.cluster_point_wall_distance_thresh)
                {
                    continue;
                }

                nearbyPointCount++;

                /* Find the projected point on the plane */
                const Eigen::Vector3d projectedPoint =
                    clusterPoint - signedPlaneDistance * wallNormUnitVector;

                /* Find relative distance between point and wall centroid */
                const Eigen::Vector3d projectedRelative =
                    projectedPoint - wallCentroid;

                /* Find distance in U axis on wall */
                const double projectedU = projectedRelative.dot(axisU);

                /* Find distance in V axis on wall */
                const double projectedV = projectedRelative.dot(axisV);

                /* Confirm if the wall encapsulates the projected point */
                const bool insideFiniteWall =
                    projectedU >=
                        minimumWallU_m -
                            sysParams->room_seg.finiteWallBoundsMargin_m &&
                    projectedU <=
                        maximumWallU_m +
                            sysParams->room_seg.finiteWallBoundsMargin_m &&
                    projectedV >=
                        minimumWallV_m -
                            sysParams->room_seg.finiteWallBoundsMargin_m &&
                    projectedV <=
                        maximumWallV_m +
                            sysParams->room_seg.finiteWallBoundsMargin_m;

                /*!
                 * If the point is within the wall plane, incriment counter or
                 * otherwise skip.
                 */
                if (insideFiniteWall)
                {
                    supportedPointCount++;
                }
                else
                {
                    continue;
                }
            }

            /* If the plane distance from cluster is far from threshold, skip */
            if (minimumPlaneDistance >
                sysParams->room_seg.cluster_point_wall_distance_thresh)
            {
                continue;
            }

            /* Calculate the fraction of nearby points inside finite bounds. */
            const double finiteSupportRatio =
                nearbyPointCount > 0
                    ? static_cast<double>(supportedPointCount) /
                          static_cast<double>(nearbyPointCount)
                    : 0.0;

            /*!
             * Accept the wall only when there is sufficient absolute support
             * and the majority of nearby points project inside the finite wall
             * patch.
             */

            if (supportedPointCount >=
                    sysParams->room_seg.minimumWallSupportPointCount &&
                finiteSupportRatio >=
                    sysParams->room_seg.minimumWallSupportRatio)
            {
                closestWalls.push_back(wall);
            }
        }

        /* Organise closest walls in order of ids */
        std::sort(closestWalls.begin(),
                  closestWalls.end(),
                  [](ORB_SLAM3::Plane *first, ORB_SLAM3::Plane *second)
                  { return first->getId() < second->getId(); });

        /* Remove duplicate walls using IDs rather than pointers */
        closestWalls.erase(
            std::unique(closestWalls.begin(),
                        closestWalls.end(),
                        [](ORB_SLAM3::Plane *first, ORB_SLAM3::Plane *second)
                        { return first->getId() == second->getId(); }),
            closestWalls.end());

        /* If there are no closest walls then skip to next cluster */
        if (closestWalls.empty())
        {
            continue;
        }

        /*
         * Match external far-side evidence before admitting any new walls.
         * Taking the wall snapshot here prevents admission by this cluster
         * from manufacturing its own promotion evidence.
         */
        ORB_SLAM3::Room                *p_clusterProspective = nullptr;
        ORB_SLAM3::Passage             *p_clusterPassage     = nullptr;
        std::vector<ORB_SLAM3::Plane *> prospectiveWallsBeforeCluster;
        double                          bestProspectiveDistance_m =
            std::numeric_limits<double>::infinity();

        for (ORB_SLAM3::Passage *p_passage : mpAtlas->GetAllPassages())
        {
            if (p_passage == nullptr)
            {
                continue;
            }

            ORB_SLAM3::Room *p_prospective = p_passage->getProspectiveRoom();

            if (p_prospective == nullptr || p_prospective->isBad() ||
                p_prospective->getRoomVariant() !=
                    ORB_SLAM3::Room::roomVariant::UNDEFINED ||
                matchedRoomIds.count(p_prospective->getId()) > 0U)
            {
                continue;
            }

            Eigen::Vector4d passageEquation_World =
                p_passage->getGlobalEquation().coeffs();
            const double passageNormalNorm =
                passageEquation_World.head<3>().norm();

            if (!passageEquation_World.allFinite() || passageNormalNorm < 1e-8)
            {
                continue;
            }

            passageEquation_World /= passageNormalNorm;

            const Eigen::Vector3d prospectiveCentroid_World_m =
                p_prospective->getCentroid();
            const double prospectiveSide_m =
                passageEquation_World.head<3>().dot(
                    prospectiveCentroid_World_m) +
                passageEquation_World(3);
            const double clusterSide_m =
                passageEquation_World.head<3>().dot(clusterCentroid) +
                passageEquation_World(3);
            const double centroidDistance_m =
                (prospectiveCentroid_World_m - clusterCentroid).norm();

            if (!prospectiveCentroid_World_m.allFinite() ||
                prospectiveSide_m * clusterSide_m <= 0.0 ||
                std::abs(prospectiveSide_m) <= 0.20 ||
                std::abs(clusterSide_m) <= 0.20 ||
                centroidDistance_m > kProspectiveDedupDistance_m)
            {
                continue;
            }

            const std::vector<ORB_SLAM3::Plane *> prospectiveWalls =
                p_prospective->getWalls();
            const bool sharesWallEvidence = std::any_of(
                prospectiveWalls.begin(),
                prospectiveWalls.end(),
                [&closestWalls](ORB_SLAM3::Plane *p_prospectiveWall)
                {
                    return p_prospectiveWall != nullptr &&
                           !p_prospectiveWall->isBad() &&
                           std::find(closestWalls.begin(),
                                     closestWalls.end(),
                                     p_prospectiveWall) != closestWalls.end();
                });

            if (!sharesWallEvidence ||
                centroidDistance_m >= bestProspectiveDistance_m)
            {
                continue;
            }

            p_clusterProspective          = p_prospective;
            p_clusterPassage              = p_passage;
            prospectiveWallsBeforeCluster = prospectiveWalls;
            bestProspectiveDistance_m     = centroidDistance_m;
        }

        /* Prefer the passage's stable handle; otherwise use normal matching. */
        ORB_SLAM3::Room *room = p_clusterProspective != nullptr
                                    ? p_clusterProspective
                                    : associateRooms(clusterCentroid,
                                                     closestWalls,
                                                     cluster,
                                                     matchedRoomIds);

        /* Track matched room to prevent double-matching in this cycle */
        if (room != nullptr)
        {
            matchedRoomIds.insert(room->getId());
        }

        /*!
         * If no existing room describes this free-space cluster.
         * Create one.
         *
         * Do not remove walls that are globally registered. Each mapped wall
         * surface remains owned by one room.
         *
         * Anti-duplicate guard: a wall already claimed by an existing room
         * means this cluster is the same open space (or the boundary-only
         * extension) of that room. Reusing the owner keeps the
         * one-wall-one-room invariant and prevents the classic two-room
         * duplicate derived from two walls of one open space. Only create a
         * fresh candidate when none of the cluster's walls is owned yet.
         */
        if (room == nullptr)
        {
            ORB_SLAM3::Room *p_wallOwnerRoom = nullptr;

            const std::vector<ORB_SLAM3::Room *> existingRooms_World =
                mpAtlas->GetAllRooms();

            for (ORB_SLAM3::Plane *p_candidateWall : closestWalls)
            {
                if (p_candidateWall == nullptr || p_candidateWall->isBad())
                {
                    continue;
                }

                for (ORB_SLAM3::Room *p_existingRoom : existingRooms_World)
                {
                    if (p_existingRoom == nullptr || p_existingRoom->isBad() ||
                        matchedRoomIds.count(p_existingRoom->getId()) > 0)
                    {
                        continue;
                    }

                    const std::vector<ORB_SLAM3::Plane *> roomWallsList =
                        p_existingRoom->getWalls();

                    if (std::find(roomWallsList.begin(),
                                  roomWallsList.end(),
                                  p_candidateWall) != roomWallsList.end())
                    {
                        p_wallOwnerRoom = p_existingRoom;
                        break;
                    }
                }

                if (p_wallOwnerRoom != nullptr)
                {
                    break;
                }
            }

            if (p_wallOwnerRoom != nullptr)
            {
                room = p_wallOwnerRoom;

                std::cout << "[SemMgr] Reusing existing Room#" << room->getId()
                          << " for cluster " << clusterId
                          << " (cluster walls already owned)." << std::endl;
            }
        }

        if (room == nullptr)
        {
            /*! Axiom: every room after the first must be discovered through
             * a passage (a prospective-room handle, checked above via
             * p_clusterProspective/matched wall ownership), not conjured
             * directly from free-space geometry alone -- "if a wall is
             * observed it must be linked to a room; if that room is new, it
             * must be observed through a passage" (user rule). The
             * exception is the first room -- but per-MAP, not per-mission:
             * every tracking-loss reset starts an entirely new Map with no
             * passages yet either, so it needs its own bootstrap room the
             * same way the mission's very first map did. Atlas::GetAllRooms()
             * spans every map (confirmed by reading it), so scoping this to
             * the CURRENT map only is required -- otherwise a confirmed room
             * surviving in an old, now-inactive map would permanently block
             * every future map from ever bootstrapping its own first room. */
            Map *p_currentMapForBootstrapCheck = mpAtlas->GetCurrentMap();
            const std::vector<Room *> currentMapRooms =
                p_currentMapForBootstrapCheck != nullptr
                    ? p_currentMapForBootstrapCheck->GetAllRooms()
                    : std::vector<Room *>();
            const bool anyConfirmedRoomExistsInCurrentMap =
                std::any_of(currentMapRooms.begin(),
                            currentMapRooms.end(),
                            [](ORB_SLAM3::Room *p_existingRoom)
                            {
                                return p_existingRoom != nullptr &&
                                       !p_existingRoom->isBad() &&
                                       p_existingRoom->getRoomVariant() ==
                                           ORB_SLAM3::Room::roomVariant::ROOM;
                            });

            if (anyConfirmedRoomExistsInCurrentMap)
            {
                std::cout << "[SemMgr] Cluster " << clusterId
                          << " matches no existing room, wall owner, or "
                             "passage-linked prospective room; deferring "
                             "(not the first room of this map, so it must be "
                             "discovered through a passage, not created from "
                             "geometry alone)."
                          << std::endl;
                continue;
            }

            /* Reset transient: the active map was just cleared (no live ROOM)
             * but a last-known hierarchy exists. Bootstrap owns first-room
             * creation with the stable ID; free-space must not conjure a
             * fresh SE# from stale cross-map walls/clusters in this cycle. */
            const int pendingRecoveryRoomId =
                mpAtlas != nullptr ? mpAtlas->getCurrentSemanticRoomIdentity()
                                   : -1;
            if (pendingRecoveryRoomId >= 0 &&
                mpAtlas->copyLatestRoomContext(pendingRecoveryRoomId)
                    .has_value())
            {
                std::cout << "[SemMgr] Cluster " << clusterId
                          << " deferred: recovery Room#"
                          << pendingRecoveryRoomId
                          << " owns first-room creation on this map."
                          << std::endl;
                continue;
            }

            /*! First-room creation lives in the bootstrap hierarchy, which
             * runs before free-space detection every cycle and promotes its
             * room immediately: a cluster that matches no room, wall owner,
             * or passage-linked prospective at this point describes no known
             * space, so it is deferred rather than conjured into a duplicate
             * first room. */
            std::cout << "[SemMgr] Cluster " << clusterId
                      << " deferred: first-room creation is owned by "
                         "bootstrap; cluster matches no known space."
                      << std::endl;
            continue;
        }

        /* The room centre is owned by its walls, not by free space: rooms
         * keep their creation centroid until walls arrive, then track the
         * damped wall-centroid mean in the consolidation below. Overwriting
         * from the cluster centroid every cycle drags the centre toward
         * whichever free space was observed last (live-observed: across a
         * passage onto its far side) and couples maintenance to Voxblox
         * liveness. Free-space evidence places a room once, at creation. */

        const std::vector<ORB_SLAM3::Passage *> activePassages =
            mpAtlas->GetAllPassages();
        const bool roomIsPassageBoundProspective =
            std::any_of(activePassages.begin(),
                        activePassages.end(),
                        [room](ORB_SLAM3::Passage *p_passage)
                        {
                            return p_passage != nullptr &&
                                   p_passage->getProspectiveRoom() == room &&
                                   room->getRoomVariant() ==
                                       ORB_SLAM3::Room::roomVariant::UNDEFINED;
                        });

        /* A prospective must first pass the external cluster validation below;
         * generic consolidation must not classify or replace it early. */
        if (!roomIsPassageBoundProspective)
        {
            consolidateRoomsInFreeSpaceCluster(room, cluster, allWalls);
        }

        /* Find the walls of the room */
        std::vector<ORB_SLAM3::Plane *> roomWalls = room->getWalls();

        /*! Perspective guard: a wall observed through an opening is on the far
         * side of the passage's supporting wall and therefore cannot bound the
         * near room. Bind such walls to the passage's prospective room instead,
         * where they can later be matched by independently validated far-side
         * cluster evidence. Keyed by the room centroid so the near side stays
         * stable. When no prospective exists yet, the wall is left off the near
         * room so it can bind onto a prospective created later in the same
         * cycle. The crossing geometry alone drives the far-side decision,
         * independent of the passage's passability state.
         */
        for (ORB_SLAM3::Plane *wall : closestWalls)
        {
            /* Skip invalid walls */
            if (wall == nullptr || wall->isBad())
            {
                continue;
            }

            /*! Reroute far-side walls to the passage's prospective room.
             * The ground normal is needed to project the aperture crossing. */
            bool farSideBound = false;
            {
                Plane *p_groundPlane = mpAtlas->GetBiggestGroundPlane();
                Eigen::Vector3d groundNormal_World = Eigen::Vector3d::Zero();
                if (p_groundPlane != nullptr && !p_groundPlane->isBad())
                {
                    const Eigen::Vector4d groundEq =
                        p_groundPlane->getGlobalEquation().coeffs();
                    const double groundNorm = groundEq.head<3>().norm();
                    if (groundEq.allFinite() && groundNorm > 1e-8)
                    {
                        groundNormal_World = groundEq.head<3>() / groundNorm;
                    }
                }

                for (Passage *p_passage : mpAtlas->GetAllPassages())
                {
                    if (p_passage == nullptr)
                    {
                        continue;
                    }

                    /* The aperture geometry alone decides the far side; the
                     * passage need not yet be passable or have a prospective.
                     */
                    if (!segmentCrossesPassageOpening(
                            room->getCentroid(),
                            wall->getCentroid().cast<double>(),
                            p_passage,
                            groundNormal_World,
                            static_cast<double>(
                                sysParams->room_seg.passagePartition
                                    .openingMargin_m),
                            static_cast<double>(
                                sysParams->room_seg.passagePartition
                                    .minimumSideDistance_m),
                            false))
                    {
                        continue;
                    }

                    /* The wall lies beyond the opening: it belongs to the far
                     * room, which the free-space clustering will create and
                     * claim. The wall is held on the passage's prospective room
                     * so it keeps accumulating walls and is not re-absorbed by
                     * the near room, nor stolen by a confirmed room. Only
                     * divert a wall that is still unowned or owned by the near
                     * room, so a wall already bound to a distinct confirmed
                     * room is never stolen. */
                    bool ownedByConfirmedRoom = false;
                    for (ORB_SLAM3::Room *p_other : mpAtlas->GetAllRooms())
                    {
                        if (p_other == nullptr || p_other->isBad() ||
                            p_other == room ||
                            p_other->getRoomVariant() ==
                                ORB_SLAM3::Room::roomVariant::UNDEFINED)
                        {
                            continue;
                        }
                        const std::vector<Plane *> otherWalls =
                            p_other->getWalls();
                        if (std::find(otherWalls.begin(),
                                      otherWalls.end(),
                                      wall) != otherWalls.end())
                        {
                            ownedByConfirmedRoom = true;
                            break;
                        }
                    }
                    if (ownedByConfirmedRoom)
                    {
                        break;
                    }

                    ORB_SLAM3::Room *pProspective =
                        p_passage->getProspectiveRoom();

                    if (pProspective == nullptr || pProspective->isBad())
                    {
                        /* The far-side room does not exist yet (it may be
                         * created later in this cycle by
                         * associatePassagesToRooms). Keep the wall off the
                         * near room so it can bind onto the prospective. */
                        room->removeWall(wall);
                        if (mpAtlas->GetRoomWallPlaneById(wall->getId()) ==
                            nullptr)
                        {
                            mpAtlas->AddRoomWallPlane(wall);
                        }
                        std::cout
                            << "[SemMgr] Far-side Wall#" << wall->getId()
                            << " at Passage#" << p_passage->getId()
                            << " has no prospective yet; held unbound for the "
                               "far-side room."
                            << std::endl;
                        farSideBound = true;
                        break;
                    }

                    room->removeWall(wall);
                    if (mpAtlas->GetRoomWallPlaneById(wall->getId()) == nullptr)
                    {
                        mpAtlas->AddRoomWallPlane(wall);
                    }
                    admitWallToRoom(pProspective, wall);
                    farSideBound = true;
                    break;
                }

                /* No CONFIRMED passage's aperture caught this wall. That does
                 * not mean no opening exists here -- confirmation requires
                 * several genuinely independent skeleton snapshots
                 * (minimumConfirmationSnapshots) and therefore real elapsed
                 * exploration time, so a real, already skeleton-evidenced
                 * opening can sit here well before it earns a Passage
                 * object. Falling through to ordinary admission for that
                 * entire window is exactly the bug: a wall on the far side
                 * of a genuine (if not yet confirmed) doorway gets bound to
                 * the WRONG (near) room. Re-run the same aperture test
                 * against each pending hypothesis's own evidence (its
                 * supporting wall's plane + accumulated opening size) --
                 * still no prospective room can exist yet (that requires a
                 * confirmed Passage), so the only action available is the
                 * same conservative one already used above for a confirmed
                 * passage with no prospective yet: hold the wall off the
                 * near room rather than admit it anywhere. */
                if (!farSideBound)
                {
                    for (OpenPassageEvidence &evidence : openPassageEvidence_)
                    {
                        if (!segmentCrossesOpenPassageEvidence(
                                room->getCentroid(),
                                wall->getCentroid().cast<double>(),
                                evidence.p_supportingWall,
                                evidence.centroid_World_m,
                                evidence.openingRadius_m,
                                evidence.heightSpan_m,
                                groundNormal_World,
                                static_cast<double>(
                                    sysParams->room_seg.passagePartition
                                        .openingMargin_m),
                                static_cast<double>(
                                    sysParams->room_seg.passagePartition
                                        .minimumSideDistance_m)))
                        {
                            continue;
                        }

                        room->removeWall(wall);
                        if (mpAtlas->GetRoomWallPlaneById(wall->getId()) ==
                            nullptr)
                        {
                            mpAtlas->AddRoomWallPlane(wall);
                        }
                        std::cout << "[SemMgr] Far-side Wall#" << wall->getId()
                                  << " crosses an unconfirmed passage opening "
                                     "(evidence at wall "
                                  << (evidence.p_supportingWall != nullptr
                                          ? evidence.p_supportingWall->getId()
                                          : -1)
                                  << "); held unbound pending confirmation."
                                  << std::endl;
                        farSideBound = true;
                        break;
                    }
                }
            }

            if (farSideBound)
            {
                continue;
            }

            /* Check to see if closest wall is in any of the current rooms */
            const bool alreadyInRoom =
                std::any_of(roomWalls.begin(),
                            roomWalls.end(),
                            [wall](ORB_SLAM3::Plane *existingWall) {
                                return existingWall != nullptr &&
                                       existingWall->getId() == wall->getId();
                            });

            /* If the wall is already in a room, skip to next slosest wall */
            if (alreadyInRoom)
            {
                continue;
            }

            /*
             * A mapped wall surface has one room owner. A room detected on
             * the opposite side must be bounded by its independently observed
             * wall surface rather than sharing this plane object.
             */
            const std::vector<ORB_SLAM3::Room *> mappedRooms =
                mpAtlas->GetAllRooms();

            const auto existingOwnerIterator = std::find_if(
                mappedRooms.begin(),
                mappedRooms.end(),
                [room, wall](ORB_SLAM3::Room *p_otherRoom)
                {
                    if (p_otherRoom == nullptr || p_otherRoom == room ||
                        p_otherRoom->isBad())
                    {
                        return false;
                    }

                    const std::vector<ORB_SLAM3::Plane *> otherRoomWalls =
                        p_otherRoom->getWalls();

                    return std::find(otherRoomWalls.begin(),
                                     otherRoomWalls.end(),
                                     wall) != otherRoomWalls.end();
                });

            Room *p_existingWallOwner =
                existingOwnerIterator != mappedRooms.end()
                    ? *existingOwnerIterator
                    : nullptr;
            bool     existingOwnerIsTransferableProvisional = false;
            Passage *p_transferPassage                      = nullptr;

            if (p_existingWallOwner != nullptr)
            {
                Eigen::Vector4d wallEquation_World =
                    wall->getGlobalEquation().coeffs();
                const double wallNormalNorm =
                    wallEquation_World.head<3>().norm();

                if (wallEquation_World.allFinite() && wallNormalNorm > 1e-8)
                {
                    wallEquation_World /= wallNormalNorm;

                    constexpr double maximumProvisionalPlaneDistance_m = 0.20;
                    const double     ownerPlaneDistance_m =
                        std::abs(wallEquation_World.head<3>().dot(
                                     p_existingWallOwner->getCentroid()) +
                                 wallEquation_World(3));

                    existingOwnerIsTransferableProvisional =
                        p_existingWallOwner->getRoomVariant() ==
                            Room::roomVariant::UNDEFINED &&
                        p_existingWallOwner->getWalls().size() == 1U &&
                        ownerPlaneDistance_m <=
                            maximumProvisionalPlaneDistance_m;
                }

                /*
                 * A wall initially assigned before passage partitioning may
                 * belong to the room on the far side. Transfer it only when
                 * the confirmed opening separates both room centres and the
                 * wall's observing cameras lie on the candidate-room side.
                 */
                Plane *p_groundPlane = mpAtlas->GetBiggestGroundPlane();
                Eigen::Vector3d meanObservationPosition_World_m =
                    Eigen::Vector3d::Zero();
                std::size_t validObservationCount = 0U;

                for (const auto &[p_keyFrame, observation] :
                     wall->getObservations())
                {
                    static_cast<void>(observation);

                    if (p_keyFrame == nullptr || p_keyFrame->isBad())
                    {
                        continue;
                    }

                    const Eigen::Vector3d cameraCentre_World_m =
                        p_keyFrame->GetCameraCenter().cast<double>();

                    if (cameraCentre_World_m.allFinite())
                    {
                        meanObservationPosition_World_m += cameraCentre_World_m;
                        validObservationCount++;
                    }
                }

                if (!existingOwnerIsTransferableProvisional &&
                    validObservationCount > 0U && p_groundPlane != nullptr &&
                    !p_groundPlane->isBad())
                {
                    meanObservationPosition_World_m /=
                        static_cast<double>(validObservationCount);

                    Eigen::Vector4d groundEquation_World =
                        p_groundPlane->getGlobalEquation().coeffs();
                    const double groundNormalNorm =
                        groundEquation_World.head<3>().norm();

                    if (groundEquation_World.allFinite() &&
                        groundNormalNorm > 1e-8)
                    {
                        const Eigen::Vector3d groundNormal_World =
                            groundEquation_World.head<3>() / groundNormalNorm;

                        for (Passage *p_passage : mpAtlas->GetAllPassages())
                        {
                            if (!segmentCrossesPassageOpening(
                                    p_existingWallOwner->getCentroid(),
                                    clusterCentroid,
                                    p_passage,
                                    groundNormal_World,
                                    sysParams->room_seg.passagePartition
                                        .openingMargin_m,
                                    sysParams->room_seg.passagePartition
                                        .minimumSideDistance_m))
                            {
                                continue;
                            }

                            Eigen::Vector4d passageEquation_World =
                                p_passage->getGlobalEquation().coeffs();
                            const double passageNormalNorm =
                                passageEquation_World.head<3>().norm();

                            if (!passageEquation_World.allFinite() ||
                                passageNormalNorm < 1e-8)
                            {
                                continue;
                            }

                            passageEquation_World /= passageNormalNorm;
                            const double candidateSide_m =
                                passageEquation_World.head<3>().dot(
                                    clusterCentroid) +
                                passageEquation_World(3);
                            const double observationSide_m =
                                passageEquation_World.head<3>().dot(
                                    meanObservationPosition_World_m) +
                                passageEquation_World(3);

                            constexpr double minimumEvidenceSideDistance_m =
                                0.10;

                            if (candidateSide_m * observationSide_m > 0.0 &&
                                std::abs(candidateSide_m) >=
                                    minimumEvidenceSideDistance_m &&
                                std::abs(observationSide_m) >=
                                    minimumEvidenceSideDistance_m)
                            {
                                p_transferPassage = p_passage;
                                break;
                            }
                        }
                    }
                }

                if (!existingOwnerIsTransferableProvisional)
                {
                    /* Never transfer a wall already owned by a confirmed room
                     * across a confirmed passage: a wall on the far side of an
                     * opening cannot bound the candidate room. Only a
                     * provisional single-wall structural element may hand its
                     * wall to the room that actually observes it. */
                    continue;
                }
            }

            /* Reject wall hypotheses which would corrupt this boundary. */
            if (!admitWallToRoom(room, wall))
            {
                continue;
            }

            if (p_existingWallOwner != nullptr &&
                p_existingWallOwner->removeWall(wall))
            {
                if (existingOwnerIsTransferableProvisional)
                {
                    Map *p_currentMap = mpAtlas->GetCurrentMap();

                    if (p_currentMap != nullptr)
                    {
                        p_currentMap->EraseDetectedMapRoom(p_existingWallOwner);
                        p_currentMap->EraseMarkerBasedMapRoom(
                            p_existingWallOwner);
                    }

                    p_existingWallOwner->setBad();

                    std::cout << "[SemMgr] Transferred orphan Wall#"
                              << wall->getId() << " from provisional SE#"
                              << p_existingWallOwner->getId() << " to Room#"
                              << room->getId() << "." << std::endl;
                }
                else
                {
                    std::cout
                        << "[SemMgr] Transferred Wall#" << wall->getId()
                        << " from Room#" << p_existingWallOwner->getId()
                        << " to Room#" << room->getId() << " through Passage#"
                        << p_transferPassage->getId()
                        << " using wall-observation evidence." << std::endl;
                }
            }

            roomWalls = room->getWalls();

            /* Register the uniquely owned room-wall surface. */
            if (mpAtlas->GetRoomWallPlaneById(wall->getId()) == nullptr)
            {
                mpAtlas->AddRoomWallPlane(wall);
            }
        }

        /* Find all the walls in a room */
        roomWalls = room->getWalls();

        /*!
         * Consolidate provisional single-wall structural elements whose wall
         * has now been absorbed into the cluster-backed room.
         *
         * @note        This is deliberately more restrictive than the old
         *              centroid-only reAssociateRooms() implementation.
         */
        Utils::consolidateProvisionalRooms(room, mpAtlas);

        /* Remove invalid relationships from the room's persistent graph. */
        room->removeInvalidWalls();
        roomWalls = room->getWalls();

        /*!
         * The semantic room centre is the mean of each wall's centroid
         * nudged INWARD along that wall's own room-facing normal
         * (Room::getWallNormalTowardRoom_World(), oriented against the
         * room's own current centroid before this update replaces it) by
         * a fixed offset, not the raw wall centroids themselves. A plain
         * mean of wall centroids is NOT guaranteed to land inside the room:
         * for a room only partially observed so far (e.g. two adjacent
         * walls, no opposite pair yet), the raw mean sits near the shared
         * corner, which can be right on -- or, depending on geometry,
         * outside -- the room's true interior. Nudging each wall centroid
         * inward before averaging keeps every contributing point already
         * inside the room, so their mean is too. Rooms without walls keep
         * their creation centroid until walls arrive: free-space evidence
         * places a room once, at creation, and never maintains it.
         */
        if (!roomWalls.empty())
        {
            constexpr double centroidInwardOffset_m   = 0.10;
            Eigen::Vector3d  wallMeanCentroid_World_m = Eigen::Vector3d::Zero();
            std::size_t      validWallCount           = 0U;

            for (ORB_SLAM3::Plane *p_roomWall : roomWalls)
            {
                if (p_roomWall == nullptr || p_roomWall->isBad())
                {
                    continue;
                }

                const Eigen::Vector3d wallCentroid_World_m =
                    p_roomWall->getCentroid().cast<double>();

                if (!wallCentroid_World_m.allFinite())
                {
                    continue;
                }

                const std::optional<Eigen::Vector3d> inwardNormal_World =
                    room->getWallNormalTowardRoom_World(p_roomWall);

                const Eigen::Vector3d nudgedCentroid_World_m =
                    inwardNormal_World
                        ? wallCentroid_World_m +
                              centroidInwardOffset_m * (*inwardNormal_World)
                        : wallCentroid_World_m;

                wallMeanCentroid_World_m += nudgedCentroid_World_m;
                validWallCount++;
            }

            if (validWallCount > 0U)
            {
                const Eigen::Vector3d correctedCentroid_World_m =
                    wallMeanCentroid_World_m /
                    static_cast<double>(validWallCount);

                /*!
                 * Damped update, not a snap. This centroid feeds two
                 * decisions that REMOVE walls -- isWallFaceForeignToRoom's
                 * foreign-face check (enforcePassageSideInvariant) and the
                 * passage far-side router -- which change roomWalls, which
                 * changes the raw mean computed above. An undamped snap
                 * closes an unstable feedback loop with no damping: remove
                 * a wall -> centroid shifts -> another wall's side test
                 * flips -> remove that too -> centroid shifts further.
                 * Live-observed 2026-09-04 crossing Office 6's passage: the
                 * same handful of walls near the doorway repeatedly
                 * removed and re-admitted, tens of times in a row, a
                 * previously COMPLETE room's boundary never settling.
                 * Blending in a minority weight of the fresh mean still
                 * lets the centroid track genuinely new evidence over
                 * several cycles, but one cycle's wall churn can no longer
                 * swing it far enough to flip another wall's side test.
                 */
                const Eigen::Vector3d previousCentroid_World_m =
                    room->getCentroid();
                constexpr double      centroidDampingWeight = 0.25;
                const Eigen::Vector3d dampedCentroid_World_m =
                    previousCentroid_World_m.allFinite()
                        ? (centroidDampingWeight * correctedCentroid_World_m +
                           (1.0 - centroidDampingWeight) *
                               previousCentroid_World_m)
                        : correctedCentroid_World_m;

                room->setCentroid(dampedCentroid_World_m);
            }
        }

        /*!
         * Confirm the room from its free-space cluster.
         *
         * @note        A room is defined by connected free space, not by having
         *              a particular arrangement or number of walls. The
         *              associated walls describe the room boundary but do not
         *              define whether the free-space region is a room.
         */
        const bool validFreeSpaceCluster =
            cluster.size() >=
            static_cast<std::size_t>(sysParams->room_seg.min_cluster_vertices);

        /*!
         * Require at least one associated wall before inserting the free-space
         * cluster into the structural hierarchy.
         */
        const bool hasBoundaryEvidence = !roomWalls.empty();

        const bool prospectiveWallEvidenceStillMatches =
            p_clusterProspective == room && p_clusterPassage != nullptr &&
            std::any_of(prospectiveWallsBeforeCluster.begin(),
                        prospectiveWallsBeforeCluster.end(),
                        [&closestWalls, &roomWalls](ORB_SLAM3::Plane *p_wall)
                        {
                            return p_wall != nullptr && !p_wall->isBad() &&
                                   std::find(closestWalls.begin(),
                                             closestWalls.end(),
                                             p_wall) != closestWalls.end() &&
                                   std::find(roomWalls.begin(),
                                             roomWalls.end(),
                                             p_wall) != roomWalls.end();
                        });

        /* Confirm the cluster-backed structural element as a room. */
        if (validFreeSpaceCluster && hasBoundaryEvidence &&
            room->getRoomVariant() == ORB_SLAM3::Room::roomVariant::UNDEFINED)
        {
            if (prospectiveWallEvidenceStillMatches)
            {
                Map *p_roomMap = room->getMap();
                if (p_roomMap != nullptr)
                {
                    p_roomMap->PromoteCandidateMapRoom(room);
                }

                room->setRoomVariant(ORB_SLAM3::Room::roomVariant::ROOM);
                room->setName("Room#" + std::to_string(room->getId()));
                prospectiveRoomCycles_.erase(room->getId());

                room->setDoorways(p_clusterPassage);
                p_clusterPassage->setProspectiveRoom(room);

                std::cout << "[SemMgr] Promoted prospective Room#"
                          << room->getId()
                          << " to ROOM from far-side cluster evidence."
                          << std::endl;
            }
            else if (!roomIsPassageBoundProspective)
            {
                Map *p_roomMap = room->getMap();
                if (p_roomMap != nullptr)
                {
                    p_roomMap->PromoteCandidateMapRoom(room);
                }
                room->setRoomVariant(ORB_SLAM3::Room::roomVariant::ROOM);
                room->setName("Room#" + std::to_string(room->getId()));

                std::cout << "[SemMgr] Structural Element #" << room->getId()
                          << " classified as a Room from free-space cluster "
                          << clusterId << "." << std::endl;
            }
        }
    }
}

void SemanticsManager::getUpdatedFloors(void)
{
    Map *p_currentMap = mpAtlas->GetCurrentMap();
    if (p_currentMap == nullptr)
    {
        return;
    }

    /* The current implementation supports one floor */
    if (p_currentMap->GetAllFloors().empty())
    {
        /* A reset can reach this update before the new map has a usable
         * camera pose. Recover the semantic floor identity from the last
         * current-room hierarchy instead of consuming a new mission identity.
         * The newly allocated Floor object deliberately carries no prior-map
         * geometry; only its semantic ID crosses the reset boundary. */
        std::optional<int> recoveredFloorId;
        const int          currentSemanticRoomId =
            mpAtlas->getCurrentSemanticRoomIdentity();
        if (currentSemanticRoomId >= 0)
        {
            const std::optional<RoomContextSnapshot> recoveryContext =
                mpAtlas->copyLatestRoomContext(currentSemanticRoomId);
            if (recoveryContext.has_value() && recoveryContext->floorId >= 0)
            {
                recoveredFloorId = recoveryContext->floorId;
            }
        }
        GeoSemHelpers::createMapFloor(mpAtlas, recoveredFloorId);
    }

    /* Collapse legacy/merge duplicates before writing any hierarchy edge. */
    std::vector<ORB_SLAM3::Floor *> floors = p_currentMap->GetAllFloors();
    Floor *p_keeperFloor = Floor::selectBestObservedFloor(floors);
    if (p_keeperFloor == nullptr)
    {
        return;
    }
    for (Floor *p_duplicateFloor : floors)
    {
        if (p_duplicateFloor == nullptr || p_duplicateFloor == p_keeperFloor)
        {
            continue;
        }
        p_duplicateFloor->setRooms({});
        p_currentMap->EraseMapFloor(p_duplicateFloor);
    }

    /* Refresh the comparable identity from the strongest observed ground. */
    Plane *p_groundPlane         = p_currentMap->GetBiggestGroundPlane();
    bool   groundIdentityUpdated = false;
    if (p_groundPlane != nullptr)
    {
        const Plane::GeometrySnapshot groundGeometry =
            p_groundPlane->getGeometrySnapshot();
        const double groundNormalNorm =
            groundGeometry.equation_World.head<3>().norm();
        if (groundGeometry.cloudGeneration ==
                groundGeometry.successfulRefitGeneration &&
            groundGeometry.finiteSupportCount > 0U &&
            groundGeometry.equation_World.allFinite() &&
            std::isfinite(groundNormalNorm) &&
            std::abs(groundNormalNorm - 1.0) <= 1e-3)
        {
            groundIdentityUpdated = p_keeperFloor->setPlaneIdentity(
                groundGeometry.equation_World,
                groundGeometry.finiteSupportCount,
                groundGeometry.observationCount);
        }
    }
    if (!groundIdentityUpdated)
    {
        p_keeperFloor->clearPlaneIdentity();
    }

    /* Extract only CONFIRMED rooms (detected map rooms) for floor centroid.
     * Exclude candidate/prospective rooms from mspMarkerBasedRooms. */
    std::vector<ORB_SLAM3::Room *> confirmedRooms =
        mpAtlas->GetAllDetectedMapRooms();

    /* Remove all invalid rooms */
    confirmedRooms.erase(std::remove_if(confirmedRooms.begin(),
                                        confirmedRooms.end(),
                                        [](ORB_SLAM3::Room *room) {
                                            return room == nullptr ||
                                                   room->isBad();
                                        }),
                         confirmedRooms.end());

    /* Keep hierarchy backlinks current even when no valid rooms remain. */
    if (confirmedRooms.empty())
    {
        p_keeperFloor->setRooms({});
        return;
    }

    /* Create list of centroids for each confirmed room */
    std::vector<Eigen::Vector3d> roomCentroids;
    roomCentroids.reserve(confirmedRooms.size());

    /* Extract centroids from each confirmed room */
    for (ORB_SLAM3::Room *room : confirmedRooms)
    {
        roomCentroids.push_back(room->getCentroid());
    }

    /* Find the floor centroid from the confirmed room centroids */
    const Eigen::Vector3d floorCentroid =
        Utils::computeCentroidFromPoints(roomCentroids);

    p_keeperFloor->setRooms(confirmedRooms);
    p_keeperFloor->setCentroid(floorCentroid);
}

void SemanticsManager::reconcileRoomGroundPlanes(void)
{
    Map *p_currentMap = mpAtlas->GetCurrentMap();
    if (p_currentMap == nullptr)
    {
        return;
    }

    std::vector<ORB_SLAM3::Floor *> floors = p_currentMap->GetAllFloors();
    Floor *p_canonicalFloor = Floor::selectBestObservedFloor(floors);
    if (p_canonicalFloor == nullptr || !p_canonicalFloor->hasPlaneIdentity())
    {
        /* No canonical identity to reconcile against yet. */
        return;
    }

    const std::optional<Floor::PlaneIdentity> canonicalIdentity =
        p_canonicalFloor->getPlaneIdentity();
    if (!canonicalIdentity.has_value())
    {
        return;
    }

    Plane *p_canonicalGroundPlane = p_currentMap->GetBiggestGroundPlane();

    for (ORB_SLAM3::Room *p_room : mpAtlas->GetAllDetectedMapRooms())
    {
        if (p_room == nullptr || p_room->isBad())
        {
            continue;
        }

        Plane *p_roomGroundPlane = p_room->getGroundPlane();
        if (p_roomGroundPlane == nullptr || p_roomGroundPlane->isBad() ||
            p_roomGroundPlane == p_canonicalGroundPlane)
        {
            /* Nothing to reconcile: no ground plane yet, or already the
             * canonical one. */
            continue;
        }

        const Plane::GeometrySnapshot roomGroundGeometry =
            p_roomGroundPlane->getGeometrySnapshot();
        const double roomGroundNormalNorm =
            roomGroundGeometry.equation_World.head<3>().norm();
        if (roomGroundGeometry.cloudGeneration !=
                roomGroundGeometry.successfulRefitGeneration ||
            roomGroundGeometry.finiteSupportCount == 0U ||
            !roomGroundGeometry.equation_World.allFinite() ||
            !std::isfinite(roomGroundNormalNorm) ||
            std::abs(roomGroundNormalNorm - 1.0) > 1e-3)
        {
            /* Room's ground plane geometry isn't settled yet -- nothing
             * reliable to compare. */
            continue;
        }

        const Floor::PlaneIdentity roomIdentity{
            roomGroundGeometry.equation_World,
            roomGroundGeometry.finiteSupportCount,
            roomGroundGeometry.observationCount};

        double normalAngle_deg = 0.0;
        double offset_m        = 0.0;
        if (Floor::planeIdentitiesMatch(canonicalIdentity.value(),
                                        roomIdentity,
                                        Floor::kMergeMaxPlaneNormalAngle_deg,
                                        Floor::kMergeMaxPlaneOffset_m,
                                        normalAngle_deg,
                                        offset_m))
        {
            /* Within tolerance -- nothing to reconcile. */
            continue;
        }

        /* A real flatness disagreement. Re-point the less-observed side to
         * the canonical plane -- a pure pointer rewire, never a geometry
         * mutation, so it can never fight a plane's own cloud refit. */
        if (roomIdentity.observationCount <=
                canonicalIdentity->observationCount &&
            p_canonicalGroundPlane != nullptr)
        {
            p_room->setGroundPlane(p_canonicalGroundPlane);
            std::cout << "[SemMgr] Room#" << p_room->getId()
                      << "'s ground plane disagreed with Floor#"
                      << p_canonicalFloor->getId()
                      << "'s canonical level (normal " << normalAngle_deg
                      << " deg, offset " << offset_m
                      << " m) -- re-pointed to the canonical plane."
                      << std::endl;
        }
        else
        {
            std::cout << "[SemMgr] Room#" << p_room->getId()
                      << "'s ground plane is more observed than Floor#"
                      << p_canonicalFloor->getId()
                      << "'s current canonical level (normal "
                      << normalAngle_deg << " deg, offset " << offset_m
                      << " m) -- left as-is; the floor will re-select its "
                         "canonical identity next cycle."
                      << std::endl;
        }
    }
}

ORB_SLAM3::Room *SemanticsManager::associateRooms(
    const Eigen::Vector3d                  clusterCentroid_World_in,
    const std::vector<ORB_SLAM3::Plane *> &wallList_World_in,
    const std::vector<Eigen::Vector3d>    &freeSpaceCluster_World_m_in,
    const std::unordered_set<int>         &excludedRoomIds_in)
{
    /* Extract parameter on centre distance threshold of room */
    const double centerDistanceThreshold =
        static_cast<double>(sysParams->room_seg.center_distance_thresh);

    constexpr double sideEpsilon = 0.20;

    /* Init list variables of nearest room and best shared room */
    ORB_SLAM3::Room *bestSharedRoom = nullptr;
    ORB_SLAM3::Room *nearestRoom    = nullptr;

    /* Init a counter to track the number of best same side matches */
    std::size_t bestSameSideMatches = 0;

    /* Init variables to find the best shared distance and nearest distance */
    double bestSharedDistance = std::numeric_limits<double>::max();
    double nearestDistance    = std::numeric_limits<double>::max();

    /* Get a list of all rooms within map */
    const std::vector<ORB_SLAM3::Room *> allRooms_World =
        mpAtlas->GetAllRooms();

    /* Evaluate every room once against the complete cluster wall set. */
    for (ORB_SLAM3::Room *room_World : allRooms_World)
    {
        /* Skip room if invalid */
        if (room_World == nullptr || room_World->isBad())
        {
            continue;
        }

        /* Skip rooms already matched to another cluster in this cycle */
        if (excludedRoomIds_in.count(room_World->getId()) > 0)
        {
            continue;
        }

        /* Extract room centroid */
        const Eigen::Vector3d roomCenter_World = room_World->getCentroid();

        /* Find the distance from the cluster center to the room center */
        const double roomCenterRelClusterCenterDistance =
            (roomCenter_World - clusterCentroid_World_in).norm();

        const bool roomSeparatedFromCluster = hasSeparatingFiniteWall(
            mpAtlas->GetAllPlanes(),
            roomCenter_World,
            clusterCentroid_World_in,
            static_cast<double>(sysParams->room_seg.finiteWallBoundsMargin_m));

        /* Extract the walls from the room */
        const std::vector<ORB_SLAM3::Plane *> roomWallsList =
            room_World->getWalls();

        /* Init a list to track the room wall ids */
        std::unordered_set<int> roomWallIds;

        /* Reserve space in list to track room ids */
        roomWallIds.reserve(roomWallsList.size());

        /* Iterate through room walls to extract ids of room */
        for (ORB_SLAM3::Plane *roomWall : roomWallsList)
        {
            /* Skip invalid walls */
            if (roomWall != nullptr && !roomWall->isBad())
            {
                roomWallIds.insert(roomWall->getId());
            }
        }

        /* Init counters for walls on same and oposite sides of room */
        std::size_t sameSideMatches     = 0;
        std::size_t oppositeSideMatches = 0;

        /* Init a flag to see if the rooms share any walls */
        bool sharesAnyWall = false;

        /* Iterate through walls in room and see if they share walls */
        for (ORB_SLAM3::Plane *candidateWall : wallList_World_in)
        {
            /* Skip invalid walls */
            if (candidateWall == nullptr || candidateWall->isBad())
            {
                continue;
            }

            /* If wall is not linked to room, skip */
            if (roomWallIds.count(candidateWall->getId()) == 0)
            {
                continue;
            }

            /* If wall is not skipped, then shares a wall */
            sharesAnyWall = true;

            /* Extract plane equation */
            Eigen::Vector4d equation =
                candidateWall->getGlobalEquation().coeffs();

            /* Find plane normal magnitude */
            const double normalNorm = equation.head<3>().norm();

            /* If magnitude is invalud, skip wall */
            if (!std::isfinite(normalNorm) || normalNorm < 1e-8)
            {
                continue;
            }

            /* Find unit vector of plane norm */
            equation /= normalNorm;

            /* Caldaulte the side of the cluster */
            const double clusterSide =
                equation.head<3>().dot(clusterCentroid_World_in) + equation(3);

            /* Caldaulte the side of the room */
            const double roomSide =
                equation.head<3>().dot(roomCenter_World) + equation(3);

            /*!
             * A provisional room may initially have its centroid
             * directly on its first wall. Treat this as compatible.
             */
            const bool centroidNearWall =
                std::abs(clusterSide) <= sideEpsilon ||
                std::abs(roomSide) <= sideEpsilon;

            const bool sameSide = clusterSide * roomSide > 0.0;

            if (centroidNearWall || sameSide)
            {
                sameSideMatches++;
            }
            else
            {
                oppositeSideMatches++;
            }
        }

        /*!
         * Shared-wall matching is preferred, but only when the cluster and
         * room are on the same side.
         */
        if (sameSideMatches > 0 && !roomSeparatedFromCluster)
        {
            if (sameSideMatches > bestSameSideMatches ||
                (sameSideMatches == bestSameSideMatches &&
                 roomCenterRelClusterCenterDistance < bestSharedDistance))
            {
                bestSameSideMatches = sameSideMatches;

                bestSharedDistance = roomCenterRelClusterCenterDistance;

                bestSharedRoom = room_World;
            }
        }

        /*!
         * A shared wall on the opposite side is evidence of an adjacent
         * room. Do not use centroid fallback in that case.
         */
        if (sharesAnyWall && sameSideMatches == 0 && oppositeSideMatches > 0)
        {
            continue;
        }

        double nearestFreeSpacePointDistance_m =
            std::numeric_limits<double>::infinity();

        for (const Eigen::Vector3d &freeSpacePoint_World_m :
             freeSpaceCluster_World_m_in)
        {
            if (freeSpacePoint_World_m.allFinite())
            {
                nearestFreeSpacePointDistance_m = std::min(
                    nearestFreeSpacePointDistance_m,
                    (roomCenter_World - freeSpacePoint_World_m).norm());
            }
        }

        const bool roomSupportedByConnectedFreeSpace =
            nearestFreeSpacePointDistance_m <= centerDistanceThreshold;

        /*!
         * Connected free space may update confirmed rooms even when a
         * changing wall subset provides no exact shared plane. A finite
         * wall between the two centroids always vetoes this fallback.
         */
        if (!sharesAnyWall && roomSupportedByConnectedFreeSpace &&
            !roomSeparatedFromCluster &&
            roomCenterRelClusterCenterDistance < nearestDistance)
        {
            nearestDistance = roomCenterRelClusterCenterDistance;

            nearestRoom = room_World;
        }
    }

    ORB_SLAM3::Room *result =
        bestSharedRoom != nullptr ? bestSharedRoom : nearestRoom;

    return result;
}

void SemanticsManager::consolidateRoomsInFreeSpaceCluster(
    ORB_SLAM3::Room                       *p_retainedRoom_inout,
    const std::vector<Eigen::Vector3d>    &freeSpaceCluster_World_m_in,
    const std::vector<ORB_SLAM3::Plane *> &wallList_World_in)
{
    Map *p_currentMap = mpAtlas->GetCurrentMap();

    if (p_retainedRoom_inout == nullptr || p_retainedRoom_inout->isBad() ||
        p_currentMap == nullptr || freeSpaceCluster_World_m_in.empty())
    {
        return;
    }

    const double maximumClusterSupportDistance_m =
        static_cast<double>(sysParams->room_seg.center_distance_thresh);
    const double finiteWallBoundsMargin_m =
        static_cast<double>(sysParams->room_seg.finiteWallBoundsMargin_m);
    const Eigen::Vector3d retainedCentroid_World_m =
        p_retainedRoom_inout->getCentroid();

    const auto distanceToCluster_m =
        [&freeSpaceCluster_World_m_in](const Eigen::Vector3d &point_World_m_in)
    {
        double minimumDistance_m = std::numeric_limits<double>::infinity();

        for (const Eigen::Vector3d &clusterPoint_World_m :
             freeSpaceCluster_World_m_in)
        {
            if (clusterPoint_World_m.allFinite())
            {
                minimumDistance_m =
                    std::min(minimumDistance_m,
                             (point_World_m_in - clusterPoint_World_m).norm());
            }
        }

        return minimumDistance_m;
    };

    for (ORB_SLAM3::Room *p_duplicateRoom : p_currentMap->GetAllRooms())
    {
        if (p_duplicateRoom == nullptr ||
            p_duplicateRoom == p_retainedRoom_inout || p_duplicateRoom->isBad())
        {
            continue;
        }

        const std::vector<Passage *> activePassages = mpAtlas->GetAllPassages();
        const bool                   duplicateIsLiveProspective = std::any_of(
            activePassages.begin(),
            activePassages.end(),
            [p_duplicateRoom](Passage *p_passage)
            {
                return p_passage != nullptr &&
                       p_passage->getProspectiveRoom() == p_duplicateRoom;
            });

        if (duplicateIsLiveProspective)
        {
            continue;
        }

        const Room::roomVariant retainedType =
            p_retainedRoom_inout->getRoomVariant();
        const Room::roomVariant duplicateType =
            p_duplicateRoom->getRoomVariant();

        if (retainedType != Room::roomVariant::UNDEFINED &&
            duplicateType != Room::roomVariant::UNDEFINED &&
            retainedType != duplicateType)
        {
            continue;
        }

        if (p_retainedRoom_inout->getHasKnownLabel() &&
            p_duplicateRoom->getHasKnownLabel() &&
            p_retainedRoom_inout->getMetaMarkerId() !=
                p_duplicateRoom->getMetaMarkerId())
        {
            continue;
        }

        const Eigen::Vector3d duplicateCentroid_World_m =
            p_duplicateRoom->getCentroid();
        const double centroidDistance_m =
            (duplicateCentroid_World_m - retainedCentroid_World_m).norm();

        /*
         * Do not impose a room-centroid separation limit here. One connected
         * free-space component can legitimately span a large office, and its
         * centroid moves as exploration expands. Membership in that component
         * plus the finite-wall veto is the relevant topological evidence.
         */
        if (!duplicateCentroid_World_m.allFinite() ||
            !std::isfinite(centroidDistance_m) ||
            distanceToCluster_m(duplicateCentroid_World_m) >
                maximumClusterSupportDistance_m ||
            hasSeparatingFiniteWall(wallList_World_in,
                                    retainedCentroid_World_m,
                                    duplicateCentroid_World_m,
                                    finiteWallBoundsMargin_m))
        {
            continue;
        }

        const std::vector<Passage *> retainedPassages =
            p_retainedRoom_inout->getPassages();
        const std::vector<Passage *> duplicatePassages =
            p_duplicateRoom->getPassages();

        bool   roomsSeparatedByConfirmedPassage = false;
        Plane *p_groundPlane = mpAtlas->GetBiggestGroundPlane();

        if (p_groundPlane != nullptr && !p_groundPlane->isBad())
        {
            Eigen::Vector4d groundEquation_World =
                p_groundPlane->getGlobalEquation().coeffs();
            const double groundNormalNorm =
                groundEquation_World.head<3>().norm();

            if (groundEquation_World.allFinite() && groundNormalNorm > 1e-8)
            {
                const Eigen::Vector3d groundNormal_World =
                    groundEquation_World.head<3>() / groundNormalNorm;
                const std::vector<Passage *> confirmedPassages =
                    mpAtlas->GetAllPassages();

                roomsSeparatedByConfirmedPassage =
                    std::any_of(confirmedPassages.begin(),
                                confirmedPassages.end(),
                                [&retainedCentroid_World_m,
                                 &duplicateCentroid_World_m,
                                 &groundNormal_World,
                                 this](Passage *p_passage)
                                {
                                    return segmentCrossesPassageOpening(
                                        retainedCentroid_World_m,
                                        duplicateCentroid_World_m,
                                        p_passage,
                                        groundNormal_World,
                                        sysParams->room_seg.passagePartition
                                            .openingMargin_m,
                                        sysParams->room_seg.passagePartition
                                            .minimumSideDistance_m);
                                });
            }
        }

        if (roomsSeparatedByConfirmedPassage)
        {
            continue;
        }

        /*
         * A shared passage only proves that two hypotheses represent adjacent
         * rooms when their centroids lie on opposite sides of the passage
         * plane. During incremental mapping, the same opening may temporarily
         * be associated with two duplicate hypotheses on the same side. Using
         * passage identity alone would then preserve the duplicate forever.
         */
        constexpr double minimumPassageSideDistance_m = 0.20;

        const bool roomsSeparatedBySharedPassage = std::any_of(
            retainedPassages.begin(),
            retainedPassages.end(),
            [&duplicatePassages,
             &retainedCentroid_World_m,
             &duplicateCentroid_World_m](Passage *p_sharedPassage_in)
            {
                if (p_sharedPassage_in == nullptr ||
                    std::find(duplicatePassages.begin(),
                              duplicatePassages.end(),
                              p_sharedPassage_in) == duplicatePassages.end())
                {
                    return false;
                }

                Eigen::Vector4d passageEquation_World =
                    p_sharedPassage_in->getGlobalEquation().coeffs();

                const double passageNormalNorm =
                    passageEquation_World.head<3>().norm();

                if (!passageEquation_World.allFinite() ||
                    passageNormalNorm < 1e-8)
                {
                    return false;
                }

                passageEquation_World /= passageNormalNorm;

                const double retainedSide_m =
                    passageEquation_World.head<3>().dot(
                        retainedCentroid_World_m) +
                    passageEquation_World(3);

                const double duplicateSide_m =
                    passageEquation_World.head<3>().dot(
                        duplicateCentroid_World_m) +
                    passageEquation_World(3);

                return retainedSide_m * duplicateSide_m < 0.0 &&
                       std::abs(retainedSide_m) >=
                           minimumPassageSideDistance_m &&
                       std::abs(duplicateSide_m) >=
                           minimumPassageSideDistance_m;
            });

        if (roomsSeparatedBySharedPassage)
        {
            continue;
        }

        std::vector<std::pair<Room *, std::vector<Plane *>>> wallSnapshots;
        for (Room *p_snapshotRoom : p_currentMap->GetAllRooms())
        {
            if (p_snapshotRoom != nullptr && !p_snapshotRoom->isBad())
            {
                wallSnapshots.emplace_back(p_snapshotRoom,
                                           p_snapshotRoom->getWalls());
            }
        }

        bool allDuplicateWallsWereAdmitted = true;

        for (Plane *p_wall : p_duplicateRoom->getWalls())
        {
            if (!admitWallToRoom(p_retainedRoom_inout, p_wall))
            {
                allDuplicateWallsWereAdmitted = false;
                break;
            }
        }

        /*
         * A topology-rejected provisional wall is not a duplicate room. Keep
         * that structural element alive so another free-space component can
         * claim it instead of repeatedly deleting and recreating it.
         */
        if (!allDuplicateWallsWereAdmitted)
        {
            for (auto &[p_snapshotRoom, snapshotWalls] : wallSnapshots)
            {
                p_snapshotRoom->clearWalls();
                for (Plane *p_snapshotWall : snapshotWalls)
                {
                    p_snapshotRoom->setWalls(p_snapshotWall);
                }
            }
            continue;
        }

        for (Passage *p_passage : duplicatePassages)
        {
            p_retainedRoom_inout->setDoorways(p_passage);
        }

        if (p_retainedRoom_inout->getGroundPlane() == nullptr)
        {
            p_retainedRoom_inout->setGroundPlane(
                p_duplicateRoom->getGroundPlane());
        }

        if (!p_retainedRoom_inout->getHasKnownLabel() &&
            p_duplicateRoom->getHasKnownLabel())
        {
            p_retainedRoom_inout->setHasKnownLabel(true);
            p_retainedRoom_inout->setMetaMarker(
                p_duplicateRoom->getMetaMarker());
            p_retainedRoom_inout->setMetaMarkerId(
                p_duplicateRoom->getMetaMarkerId());
            p_retainedRoom_inout->setName(p_duplicateRoom->getName());
        }

        if (retainedType == Room::roomVariant::UNDEFINED &&
            duplicateType != Room::roomVariant::UNDEFINED)
        {
            p_retainedRoom_inout->setRoomVariant(duplicateType);
        }

        for (Floor *p_floor : p_currentMap->GetAllFloors())
        {
            if (p_floor != nullptr)
            {
                p_floor->replaceRoom(p_duplicateRoom, p_retainedRoom_inout);
            }
        }

        p_currentMap->EraseDetectedMapRoom(p_duplicateRoom);
        p_currentMap->EraseMarkerBasedMapRoom(p_duplicateRoom);
        p_duplicateRoom->clearWalls();
        p_duplicateRoom->clearPassages();
        p_duplicateRoom->setBad();

        std::cout << "[SemMgr] Fused Room#" << p_duplicateRoom->getId()
                  << " into Room#" << p_retainedRoom_inout->getId()
                  << " using connected free-space evidence (centroid distance "
                  << centroidDistance_m << " m)." << std::endl;
    }
}

void SemanticsManager::associateAllWallsToRooms(void)
{
    /*!
     * A wall must be observed from several keyframes before it is inserted into
     * the higher-level semantic hierarchy.
     */
    /* Extract all planes from the current map */
    const std::vector<ORB_SLAM3::Plane *> allPlanes = mpAtlas->GetAllPlanes();

    /* Extract all current rooms and provisional structural elements */
    std::vector<ORB_SLAM3::Room *> allRooms      = mpAtlas->GetAllRooms();
    Map                           *p_activeMap   = mpAtlas->GetCurrentMap();
    Room                          *p_currentRoom = nullptr;
    const int                      currentRoomId = getCurrentRoomId();
    const auto planeClassName = [](const Plane::planeVariant variant_in)
    {
        switch (variant_in)
        {
        case Plane::planeVariant::WALL:
            return "WALL";
        case Plane::planeVariant::GROUND:
            return "GROUND";
        case Plane::planeVariant::DOOR:
            return "DOOR";
        case Plane::planeVariant::WINDOW:
            return "WINDOW";
        case Plane::planeVariant::UNDEFINED:
        default:
            return "UNDEFINED";
        }
    };
    for (Room *p_room : allRooms)
    {
        if (p_room != nullptr && !p_room->isBad() &&
            p_room->getMap() == p_activeMap &&
            p_room->getId() == currentRoomId &&
            p_room->getRoomVariant() == Room::roomVariant::ROOM)
        {
            p_currentRoom = p_room;
            break;
        }
    }

    /* Ground-aligned axes for evaluateWallAdmissionEvidence's height/width
     * gate (see the comment at its definition). */
    Plane          *p_groundPlaneForEvidence = mpAtlas->GetBiggestGroundPlane();
    Eigen::Vector3d groundNormalForEvidence_World = Eigen::Vector3d::Zero();
    if (p_groundPlaneForEvidence != nullptr &&
        !p_groundPlaneForEvidence->isBad())
    {
        const Eigen::Vector4d groundEq =
            p_groundPlaneForEvidence->getGlobalEquation().coeffs();
        const double groundNorm = groundEq.head<3>().norm();
        if (groundEq.allFinite() && groundNorm > 1e-8)
        {
            groundNormalForEvidence_World = groundEq.head<3>() / groundNorm;
        }
    }

    /* Helper which confirms that a room already contains a wall */
    const auto roomContainsWall = [](ORB_SLAM3::Room  *room,
                                     ORB_SLAM3::Plane *wall) -> bool
    {
        /* Return false if either object is invalid */
        if (room == nullptr || wall == nullptr)
        {
            return false;
        }

        /* Extract the walls currently assigned to the room */
        const std::vector<ORB_SLAM3::Plane *> roomWalls = room->getWalls();

        /* Check whether the requested wall is already present */
        return std::any_of(roomWalls.begin(),
                           roomWalls.end(),
                           [wall](ORB_SLAM3::Plane *existingWall) {
                               return existingWall != nullptr &&
                                      existingWall->getId() == wall->getId();
                           });
    };

    /* Iterate through every mapped plane */
    for (ORB_SLAM3::Plane *wall : allPlanes)
    {
        /* Skip invalid planes */
        if (wall == nullptr || wall->isBad())
        {
            continue;
        }

        /* Only fitted walls with semantic and observation evidence enter. */
        const WallAdmissionEvidence admissionEvidence =
            evaluateWallAdmissionEvidence(wall,
                                          sysParams,
                                          groundNormalForEvidence_World);
        if (!admissionEvidence.admissible)
        {
            const std::string reason =
                wall->getPlaneType() != Plane::planeVariant::WALL ||
                        wall->getExpectedPlaneType() !=
                            Plane::planeVariant::WALL
                    ? "CLASS_NOT_WALL"
                : !admissionEvidence.adequateFiniteFit
                    ? "INADEQUATE_FINITE_FIT"
                    : "INSUFFICIENT_OBSERVATIONS";
            if (loggedWallRejectionReasons_[wall->getId()] != reason)
            {
                loggedWallRejectionReasons_[wall->getId()] = reason;
                std::cout << "SG_PIPELINE {\"event\":\"wall_rejection\","
                             "\"map_id\":"
                          << (p_activeMap != nullptr
                                  ? static_cast<long long>(p_activeMap->GetId())
                                  : -1)
                          << ",\"semantic_cycle\":" << pipelineSemanticCycle_
                          << ",\"wall_id\":" << wall->getId() << ",\"class\":\""
                          << planeClassName(wall->getPlaneType())
                          << "\",\"lifecycle\":\"REJECTED\","
                             "\"owner\":\"NONE\",\"reason\":\""
                          << reason << "\",\"support\":"
                          << admissionEvidence.fittedPointCount
                          << ",\"observations\":"
                          << admissionEvidence.observationCount << "}"
                          << std::endl;
            }
            continue;
        }
        loggedWallRejectionReasons_.erase(wall->getId());

        /* Init flag which confirms whether the wall already has a parent */
        bool wallHasRoom = false;

        /* Search every valid room for the wall */
        for (ORB_SLAM3::Room *room : allRooms)
        {
            /* Skip invalid rooms */
            if (room == nullptr || room->isBad())
            {
                continue;
            }

            /* If the room contains the wall, the hierarchy is complete */
            if (roomContainsWall(room, wall))
            {
                wallHasRoom = true;
                break;
            }
        }

        /* Skip walls which already belong to a room */
        if (wallHasRoom)
        {
            continue;
        }

        const bool admitted =
            p_currentRoom != nullptr && admitWallToRoom(p_currentRoom, wall);
        Room *p_selectedOwner = nullptr;
        for (Room *p_room : mpAtlas->GetAllRooms())
        {
            if (p_room != nullptr && !p_room->isBad() &&
                roomContainsWall(p_room, wall))
            {
                p_selectedOwner = p_room;
                break;
            }
        }

        if (admitted && p_selectedOwner != nullptr)
        {
            if (mpAtlas->GetRoomWallPlaneById(wall->getId()) == nullptr)
            {
                mpAtlas->AddRoomWallPlane(wall);
            }
            undefendedWalls_.erase(wall->getId());
            loggedOrphanWallIds_.erase(wall->getId());
            std::cout << "SG_PIPELINE {\"event\":\"wall_admission\","
                         "\"map_id\":"
                      << p_activeMap->GetId()
                      << ",\"semantic_cycle\":" << pipelineSemanticCycle_
                      << ",\"wall_id\":" << wall->getId()
                      << ",\"class\":\"WALL\","
                         "\"lifecycle\":\"COMMITTED\",\"owner_room_id\":"
                      << p_selectedOwner->getId() << ",\"reason\":\""
                      << (p_selectedOwner == p_currentRoom
                              ? "CURRENT_ROOM_OBSERVATION"
                              : "PASSAGE_FAR_SIDE_PRECEDENCE")
                      << "\",\"support\":" << admissionEvidence.fittedPointCount
                      << ",\"observations\":"
                      << admissionEvidence.observationCount << "}" << std::endl;
            continue;
        }

        /*!
         * The free-space detector did not assign this wall to a room.
         * Register the wall for future association via passages or room
         * detection. Do NOT create a provisional room for orphan walls - only
         * passages create prospective rooms.
         */

        /* Register the uniquely owned wall in the room-wall collection. */
        if (mpAtlas->GetRoomWallPlaneById(wall->getId()) == nullptr)
        {
            mpAtlas->AddRoomWallPlane(wall);
        }

        if (loggedOrphanWallIds_.insert(wall->getId()).second)
        {
            std::cout << "SG_PIPELINE {\"event\":\"wall_pending\","
                         "\"map_id\":"
                      << (p_activeMap != nullptr
                              ? static_cast<long long>(p_activeMap->GetId())
                              : -1)
                      << ",\"semantic_cycle\":" << pipelineSemanticCycle_
                      << ",\"wall_id\":" << wall->getId()
                      << ",\"class\":\"WALL\","
                         "\"lifecycle\":\"PENDING\",\"owner\":\"PENDING\","
                         "\"reason\":\""
                      << (p_currentRoom == nullptr ? "NO_CURRENT_ROOM"
                                                   : "SAFE_ADMISSION_REJECTED")
                      << "\",\"support\":" << admissionEvidence.fittedPointCount
                      << ",\"observations\":"
                      << admissionEvidence.observationCount
                      << ",\"pending_age\":0}" << std::endl;
        }
    }
}

void SemanticsManager::suppressUndefendedWalls(void)
{
    Map *p_currentMap = mpAtlas->GetCurrentMap();

    if (p_currentMap == nullptr)
    {
        undefendedWalls_.clear();
        return;
    }

    const std::vector<Room *>    allRooms    = p_currentMap->GetAllRooms();
    const std::vector<Passage *> allPassages = p_currentMap->GetAllPassages();
    const std::vector<Plane *>   allPlanes   = p_currentMap->GetAllPlanes();
    const std::vector<std::vector<Eigen::Vector3d>> skeletonClusters =
        p_currentMap->GetSkeletonClusterPoints();
    std::unordered_set<int> mappedWallIds;

    /* Ground-aligned axes for evaluateWallAdmissionEvidence's height/width
     * gate (see the comment at its definition). */
    Plane *p_groundPlaneForEvidence = p_currentMap->GetBiggestGroundPlane();
    Eigen::Vector3d groundNormalForEvidence_World = Eigen::Vector3d::Zero();
    if (p_groundPlaneForEvidence != nullptr &&
        !p_groundPlaneForEvidence->isBad())
    {
        const Eigen::Vector4d groundEq =
            p_groundPlaneForEvidence->getGlobalEquation().coeffs();
        const double groundNorm = groundEq.head<3>().norm();
        if (groundEq.allFinite() && groundNorm > 1e-8)
        {
            groundNormalForEvidence_World = groundEq.head<3>() / groundNorm;
        }
    }

    for (Plane *p_wall : allPlanes)
    {
        if (p_wall == nullptr)
        {
            continue;
        }

        const int wallId = p_wall->getId();
        mappedWallIds.insert(wallId);

        if (p_wall->isBad() ||
            p_wall->getPlaneType() != Plane::planeVariant::WALL)
        {
            undefendedWalls_.erase(wallId);
            continue;
        }

        const bool ownedByLiveRoom = std::any_of(
            allRooms.begin(),
            allRooms.end(),
            [p_wall](Room *p_room)
            {
                if (p_room == nullptr || p_room->isBad())
                {
                    return false;
                }

                const std::vector<Plane *> roomWalls = p_room->getWalls();
                return std::find(roomWalls.begin(), roomWalls.end(), p_wall) !=
                       roomWalls.end();
            });
        const bool associatedWithPassage =
            std::any_of(allPassages.begin(),
                        allPassages.end(),
                        [p_wall](Passage *p_passage)
                        {
                            if (p_passage == nullptr || p_passage->isBad())
                            {
                                return false;
                            }

                            const std::vector<Plane *> passageWalls =
                                p_passage->getAssociateWalls();
                            return p_passage->getAssociateDoor() == p_wall ||
                                   std::find(passageWalls.begin(),
                                             passageWalls.end(),
                                             p_wall) != passageWalls.end();
                        });
        const WallAdmissionEvidence evidence =
            evaluateWallAdmissionEvidence(p_wall,
                                          sysParams,
                                          groundNormalForEvidence_World);

        const Plane::GeometrySnapshot wallGeometry =
            p_wall->getGeometrySnapshot();
        const Eigen::Vector4d wallEquation_World = wallGeometry.equation_World;
        const double wallNormalNorm = wallEquation_World.head<3>().norm();
        bool         hasCompatibleLiveCluster = false;
        if (wallGeometry.centroid_World_m.allFinite() &&
            wallEquation_World.allFinite() && wallNormalNorm > 1e-8)
        {
            const Eigen::Vector3d wallNormal_World =
                wallEquation_World.head<3>() / wallNormalNorm;
            const double wallOffset_m = wallEquation_World(3) / wallNormalNorm;
            const double maximumCentroidDistance_m =
                2.0 * static_cast<double>(
                          sysParams->room_seg
                              .cluster_centroid_wall_centroid_distance_thresh);
            const double maximumPointDistance_m = static_cast<double>(
                sysParams->room_seg.cluster_point_wall_distance_thresh);

            for (const std::vector<Eigen::Vector3d> &cluster : skeletonClusters)
            {
                if (cluster.empty())
                {
                    continue;
                }
                const Eigen::Vector3d clusterCentroid_World_m =
                    Utils::computeCentroidFromPoints(cluster);
                if (!clusterCentroid_World_m.allFinite() ||
                    (clusterCentroid_World_m - wallGeometry.centroid_World_m)
                            .norm() > maximumCentroidDistance_m)
                {
                    continue;
                }
                hasCompatibleLiveCluster = std::any_of(
                    cluster.begin(),
                    cluster.end(),
                    [&wallNormal_World, wallOffset_m, maximumPointDistance_m](
                        const Eigen::Vector3d &point)
                    {
                        return point.allFinite() &&
                               std::abs(wallNormal_World.dot(point) +
                                        wallOffset_m) <= maximumPointDistance_m;
                    });
                if (hasCompatibleLiveCluster)
                {
                    break;
                }
            }
        }

        /*!
         * Good geometry alone does NOT defend a wall (user rule: a wall
         * with no room does not get to persist as a wall plane, regardless
         * of how clean its evidence looks). Only an actual owning room or
         * a real passage association counts. A wall genuinely mid-way
         * through bootstrapping a brand-new room is still protected below
         * by the stagnant-cycle counter resetting on cloudGrew/
         * observationGrew -- it keeps gaining points/observations every
         * cycle the UAV still looks at it, so it never goes stagnant long
         * enough to retire while real room-formation is in progress.
         */
        if (ownedByLiveRoom || associatedWithPassage ||
            hasCompatibleLiveCluster)
        {
            undefendedWalls_.erase(wallId);
            if (hasCompatibleLiveCluster && !ownedByLiveRoom &&
                !associatedWithPassage)
            {
                std::cout << "SG_PIPELINE {\"event\":\"wall_pending\","
                             "\"map_id\":"
                          << p_currentMap->GetId()
                          << ",\"semantic_cycle\":" << pipelineSemanticCycle_
                          << ",\"wall_id\":" << wallId
                          << ",\"class\":\"WALL\","
                             "\"lifecycle\":\"PENDING\",\"owner\":\"PENDING\","
                             "\"reason\":\"LIVE_CLUSTER_SUPPORT\",\"support\":"
                          << evidence.fittedPointCount
                          << ",\"observations\":" << evidence.observationCount
                          << ",\"pending_age\":0}" << std::endl;
            }
            continue;
        }

        const auto        p_cloud = p_wall->getGeometrySnapshot().supportCloud;
        const std::size_t cloudPointCount =
            p_cloud != nullptr ? p_cloud->size() : 0U;
        const std::size_t observationCount = p_wall->getObservationCount();
        auto [stateIterator, inserted]     = undefendedWalls_.try_emplace(
            wallId,
            UndefendedWallState{p_wall, 0U, cloudPointCount, observationCount});

        if (!inserted && stateIterator->second.p_wall != p_wall)
        {
            stateIterator->second = UndefendedWallState{p_wall,
                                                        0U,
                                                        cloudPointCount,
                                                        observationCount};
            inserted              = true;
        }

        UndefendedWallState &state = stateIterator->second;

        if (inserted)
        {
            /* The first sighting is recent evidence; age starts next cycle. */
            continue;
        }

        const bool cloudGrew       = cloudPointCount > state.cloudPointCount;
        const bool observationGrew = observationCount > state.observationCount;
        state.cloudPointCount      = cloudPointCount;
        state.observationCount     = observationCount;

        if ((evidence.adequateFiniteFit && cloudGrew) || observationGrew)
        {
            state.unresolvedCycles = 0U;
            continue;
        }

        state.unresolvedCycles++;

        if (state.unresolvedCycles <
            sysParams->room_seg.minimumUndefendedWallHoldCycles)
        {
            std::cout << "SG_PIPELINE {\"event\":\"wall_pending\","
                         "\"map_id\":"
                      << p_currentMap->GetId()
                      << ",\"semantic_cycle\":" << pipelineSemanticCycle_
                      << ",\"wall_id\":" << wallId
                      << ",\"class\":\"WALL\","
                         "\"lifecycle\":\"PENDING\",\"owner\":\"PENDING\","
                         "\"reason\":\"GRACE_ACTIVE_NO_GROWTH\","
                         "\"support\":"
                      << evidence.fittedPointCount
                      << ",\"observations\":" << evidence.observationCount
                      << ",\"pending_age\":" << state.unresolvedCycles << "}"
                      << std::endl;
            continue;
        }

        const unsigned int retiredAfterCycles = state.unresolvedCycles;

        /* Relationships were checked above under the semantic transaction. */
        const std::map<KeyFrame *, Plane::Observation> wallObservations =
            p_wall->getObservations();

        /* Sweep every keyframe that references this plane, including
         * those that hold it in mvpMapPlanes without an Observation
         * entry, so no stale pointer survives retirement. */
        const std::vector<KeyFrame *> allKeyFrames =
            p_currentMap->GetAllKeyFrames();
        for (KeyFrame *p_keyFrame : allKeyFrames)
        {
            if (p_keyFrame != nullptr)
            {
                p_keyFrame->RemoveMapPlane(p_wall);
            }
        }

        for (const auto &[p_keyFrame, observation] : wallObservations)
        {
            static_cast<void>(observation);
            if (p_keyFrame != nullptr)
            {
                p_wall->eraseObservation(p_keyFrame);
            }
        }

        p_wall->setBad();
        p_currentMap->EraseRoomWallPlane(p_wall);
        p_currentMap->EraseMapPlane(p_wall);
        p_wall->refKeyFrame = nullptr;
        p_wall->SetMap(nullptr);
        undefendedWalls_.erase(wallId);

        if (loggedRetiredWallIds_.insert(wallId).second)
        {
            std::cout << "SG_PIPELINE {\"event\":\"wall_retirement\","
                         "\"map_id\":"
                      << p_currentMap->GetId()
                      << ",\"semantic_cycle\":" << pipelineSemanticCycle_
                      << ",\"wall_id\":" << wallId
                      << ",\"class\":\"WALL\",\"lifecycle\":\"RETIRED\","
                         "\"reason\":\"GRACE_EXPIRED_NO_GROWTH_NO_OWNER_NO_"
                         "PASSAGE_NO_CLUSTER\","
                         "\"support\":"
                      << evidence.fittedPointCount
                      << ",\"observations\":" << evidence.observationCount
                      << ",\"pending_age\":" << retiredAfterCycles << "}"
                      << std::endl;
        }
    }

    for (auto stateIterator = undefendedWalls_.begin();
         stateIterator != undefendedWalls_.end();)
    {
        stateIterator = mappedWallIds.count(stateIterator->first) == 0U
                            ? undefendedWalls_.erase(stateIterator)
                            : std::next(stateIterator);
    }
}

std::vector<semantic::OpenPassageHypothesisRecord>
    SemanticsManager::captureOpenPassageHypotheses(void) const
{
    std::vector<semantic::OpenPassageHypothesisRecord> records;
    records.reserve(openPassageEvidence_.size());
    for (const OpenPassageEvidence &evidence : openPassageEvidence_)
    {
        semantic::OpenPassageHypothesisRecord record;
        record.supportingWallRef =
            semantic::rawPlaneRef(evidence.p_supportingWall);
        record.centroid_World_m  = evidence.centroid_World_m;
        record.confirmationCount = evidence.confirmationCount;
        record.missedUpdateCount = evidence.missedUpdateCount;
        record.lastConfirmedSkeletonFingerprint =
            evidence.lastConfirmedSkeletonFingerprint;
        record.openingRadius_m = evidence.openingRadius_m;
        record.heightSpan_m    = evidence.heightSpan_m;
        records.push_back(record);
    }
    std::sort(records.begin(),
              records.end(),
              [](const semantic::OpenPassageHypothesisRecord &lhs_in,
                 const semantic::OpenPassageHypothesisRecord &rhs_in)
              {
                  if (semantic::isRawPlaneRefLess(lhs_in.supportingWallRef,
                                                  rhs_in.supportingWallRef))
                      return true;
                  if (semantic::isRawPlaneRefLess(rhs_in.supportingWallRef,
                                                  lhs_in.supportingWallRef))
                      return false;
                  return semantic::isVector3dLess(lhs_in.centroid_World_m,
                                                  rhs_in.centroid_World_m);
              });
    return records;
}

std::vector<semantic::UnresolvedWallHypothesisRecord>
    SemanticsManager::captureUnresolvedWallHypotheses(void) const
{
    std::vector<semantic::UnresolvedWallHypothesisRecord> records;
    records.reserve(undefendedWalls_.size());
    for (const auto &[wallId, state] : undefendedWalls_)
    {
        semantic::UnresolvedWallHypothesisRecord record;
        record.wallRef          = semantic::rawPlaneRef(state.p_wall);
        record.unresolvedCycles = state.unresolvedCycles;
        record.cloudPointCount  = state.cloudPointCount;
        record.observationCount = state.observationCount;
        records.push_back(record);
    }
    std::sort(
        records.begin(),
        records.end(),
        [](const semantic::UnresolvedWallHypothesisRecord &lhs_in,
           const semantic::UnresolvedWallHypothesisRecord &rhs_in)
        {
            if (semantic::isRawPlaneRefLess(lhs_in.wallRef, rhs_in.wallRef))
                return true;
            if (semantic::isRawPlaneRefLess(rhs_in.wallRef, lhs_in.wallRef))
                return false;
            return lhs_in.unresolvedCycles < rhs_in.unresolvedCycles;
        });
    return records;
}

void SemanticsManager::logSemanticDiagnostics(
    const semantic::SemanticReportCacheEntry &entry_in)
{
    /* Pure diff/JSON construction lives in SemanticDiagnostics (P1.7,
     * semantic-axiom-reliability-plan.md); this method's only job is
     * deciding whether/what to print. mSemanticDiagnosticState_ is the
     * only mutable state carried across calls. */
    const semantic::SemanticDiagnosticUpdate update =
        semantic::buildSemanticDiagnosticUpdate(entry_in,
                                                mSemanticDiagnosticState_);
    if (!update.emit)
    {
        return;
    }

    std::cout << "SG_AXIOM " << update.summary.dump() << std::endl;
    for (const nlohmann::json &detail : update.violationDetails)
    {
        std::cout << "SG_VIOLATION " << detail.dump() << std::endl;
    }
}

void SemanticsManager::enforceUniqueWallOwnership(void)
{
    std::vector<ORB_SLAM3::Room *> allRooms = mpAtlas->GetAllRooms();
    std::sort(allRooms.begin(),
              allRooms.end(),
              [](const Room *p_firstRoom, const Room *p_secondRoom)
              {
                  if (p_firstRoom == nullptr)
                  {
                      return false;
                  }

                  if (p_secondRoom == nullptr)
                  {
                      return true;
                  }

                  return p_firstRoom->getId() < p_secondRoom->getId();
              });

    std::vector<Passage *> allPassages = mpAtlas->GetAllPassages();
    std::sort(allPassages.begin(),
              allPassages.end(),
              [](const Passage *p_first, const Passage *p_second)
              {
                  if (p_first == nullptr)
                  {
                      return false;
                  }
                  if (p_second == nullptr)
                  {
                      return true;
                  }
                  return p_first->getId() < p_second->getId();
              });

    Eigen::Vector3d groundNormal_World = Eigen::Vector3d::Zero();
    Plane          *p_groundPlane      = mpAtlas->GetBiggestGroundPlane();
    if (p_groundPlane != nullptr && !p_groundPlane->isBad())
    {
        const Eigen::Vector4d groundEquation =
            p_groundPlane->getGlobalEquation().coeffs();
        const double groundNormalNorm = groundEquation.head<3>().norm();
        if (groundEquation.allFinite() && groundNormalNorm > 1e-8)
        {
            groundNormal_World = groundEquation.head<3>() / groundNormalNorm;
        }
    }

    std::unordered_map<Plane *, std::vector<Room *>> wallOwners;

    for (Room *p_room : allRooms)
    {
        if (p_room == nullptr || p_room->isBad())
        {
            continue;
        }

        for (Plane *p_wall : p_room->getWalls())
        {
            if (p_wall == nullptr || p_wall->isBad())
            {
                continue;
            }

            std::vector<Room *> &owners = wallOwners[p_wall];
            if (std::find(owners.begin(), owners.end(), p_room) == owners.end())
            {
                owners.push_back(p_room);
            }
        }
    }

    for (auto &[p_wall, owners] : wallOwners)
    {
        if (owners.size() < 2U)
        {
            continue;
        }

        Room                      *p_retainedOwner = nullptr;
        std::unordered_set<Room *> passageRejectedOwners;

        /* Passage-side routing is authoritative. A near-side owner whose
         * centroid-to-wall segment crosses an opening is not eligible; a live
         * stable far-side handle is preferred unless that would steal from a
         * different confirmed owner.
         *
         * Eligibility here is deliberately geometric only (isPassable(), the
         * passage's own detected-opening evidence) -- traversal evidence
         * (the camera/UAV having flown through this spot) proves only that
         * a room change happened there, not this passage's own aperture
         * geometry. Substituting it in as an OR-alternative would let a
         * geometrically-unconfirmed "passage" arbitrate which confirmed
         * room owns a contested wall, conflating motion evidence with wall
         * identity. */
        for (Room *p_nearOwner : owners)
        {
            for (Passage *p_passage : allPassages)
            {
                if (p_passage == nullptr || !p_passage->isPassable() ||
                    !segmentCrossesPassageOpening(
                        p_nearOwner->getCentroid(),
                        p_wall->getCentroid().cast<double>(),
                        p_passage,
                        groundNormal_World,
                        sysParams->room_seg.passagePartition.openingMargin_m,
                        sysParams->room_seg.passagePartition
                            .minimumSideDistance_m,
                        false))
                {
                    continue;
                }

                Room *p_farSideOwner = p_passage->getProspectiveRoom();
                if (p_farSideOwner == nullptr || p_farSideOwner->isBad() ||
                    p_farSideOwner == p_nearOwner ||
                    p_farSideOwner->getMap() != mpAtlas->GetCurrentMap())
                {
                    passageRejectedOwners.insert(p_nearOwner);
                    continue;
                }

                const bool wouldStealDistinctConfirmedOwner =
                    std::any_of(owners.begin(),
                                owners.end(),
                                [p_nearOwner, p_farSideOwner](Room *p_owner)
                                {
                                    return p_owner != p_nearOwner &&
                                           p_owner != p_farSideOwner &&
                                           p_owner->getRoomVariant() ==
                                               Room::roomVariant::ROOM;
                                });
                if (wouldStealDistinctConfirmedOwner)
                {
                    continue;
                }

                if (p_retainedOwner == nullptr ||
                    (p_farSideOwner->getRoomVariant() ==
                         Room::roomVariant::ROOM &&
                     p_retainedOwner->getRoomVariant() !=
                         Room::roomVariant::ROOM) ||
                    (p_farSideOwner->getRoomVariant() ==
                         p_retainedOwner->getRoomVariant() &&
                     p_farSideOwner->getId() < p_retainedOwner->getId()))
                {
                    p_retainedOwner = p_farSideOwner;
                }
            }
        }

        /* Existing confirmed ownership outranks camera proximity. */
        if (p_retainedOwner == nullptr)
        {
            for (Room *p_owner : owners)
            {
                if (passageRejectedOwners.count(p_owner) == 0U &&
                    p_owner->getRoomVariant() == Room::roomVariant::ROOM)
                {
                    p_retainedOwner = p_owner;
                    break;
                }
            }
        }

        /* Camera proximity is the final fallback among equivalent/provisional
         * owners only. */
        if (p_retainedOwner == nullptr)
        {
            Eigen::Vector3d meanObservationPosition_World_m =
                Eigen::Vector3d::Zero();
            std::size_t validObservationCount = 0U;

            for (const auto &[p_keyFrame, observation] :
                 p_wall->getObservations())
            {
                static_cast<void>(observation);

                if (p_keyFrame != nullptr && !p_keyFrame->isBad())
                {
                    const Eigen::Vector3d cameraCenter_World_m =
                        p_keyFrame->GetCameraCenter().cast<double>();

                    if (cameraCenter_World_m.allFinite())
                    {
                        meanObservationPosition_World_m += cameraCenter_World_m;
                        validObservationCount++;
                    }
                }
            }

            if (validObservationCount > 0U)
            {
                meanObservationPosition_World_m /=
                    static_cast<double>(validObservationCount);
                double bestDistance_m = std::numeric_limits<double>::infinity();

                for (Room *p_owner : owners)
                {
                    if (passageRejectedOwners.count(p_owner) > 0U)
                    {
                        continue;
                    }

                    const double distance_m = (p_owner->getCentroid() -
                                               meanObservationPosition_World_m)
                                                  .norm();
                    if (distance_m < bestDistance_m)
                    {
                        bestDistance_m  = distance_m;
                        p_retainedOwner = p_owner;
                    }
                }
            }

            if (p_retainedOwner == nullptr)
            {
                for (Room *p_owner : owners)
                {
                    if (passageRejectedOwners.count(p_owner) == 0U)
                    {
                        p_retainedOwner = p_owner;
                        break;
                    }
                }
            }
        }

        for (Room *p_owner : owners)
        {
            if (p_owner != p_retainedOwner && p_owner->removeWall(p_wall))
            {
                std::cerr << "[SemMgr] Corrected duplicate ownership of Wall#"
                          << p_wall->getId() << ": "
                          << (p_retainedOwner != nullptr
                                  ? "retained Room#" +
                                        std::to_string(p_retainedOwner->getId())
                                  : "left orphaned")
                          << ", detached Room#" << p_owner->getId() << "."
                          << std::endl;
            }
        }

        if (p_retainedOwner != nullptr &&
            std::find(owners.begin(), owners.end(), p_retainedOwner) ==
                owners.end())
        {
            p_retainedOwner->setWalls(p_wall);
        }
    }
}

void SemanticsManager::reconcileWallFacePairs(void)
{
    Plane          *p_groundPlane      = mpAtlas->GetBiggestGroundPlane();
    Eigen::Vector3d groundNormal_World = Eigen::Vector3d::Zero();
    if (p_groundPlane != nullptr && !p_groundPlane->isBad())
    {
        const Eigen::Vector4d groundEq =
            p_groundPlane->getGlobalEquation().coeffs();
        const double groundNorm = groundEq.head<3>().norm();
        if (groundEq.allFinite() && groundNorm > 1e-8)
        {
            groundNormal_World = groundEq.head<3>() / groundNorm;
        }
    }

    const double minimumThickness_m =
        static_cast<double>(sysParams->sem_seg.wallPairing.minimumThickness_m);
    const double maximumThickness_m =
        static_cast<double>(sysParams->sem_seg.wallPairing.maximumThickness_m);
    const double minimumOverlapRatio =
        static_cast<double>(sysParams->sem_seg.wallPairing.minimumOverlapRatio);

    std::vector<Plane *> wallPlanes;
    for (Plane *p_plane : mpAtlas->GetAllPlanes())
    {
        if (p_plane != nullptr && !p_plane->isBad() &&
            p_plane->getPlaneType() == Plane::planeVariant::WALL)
        {
            wallPlanes.push_back(p_plane);
        }
    }
    /* Lock two Planes in ascending id order to avoid a lock-order hazard,
     * matching the convention already used for passages in
     * admitWallToRoom(). */
    std::sort(wallPlanes.begin(),
              wallPlanes.end(),
              [](const Plane *p_first, const Plane *p_second)
              {
                  if (p_first == nullptr)
                  {
                      return false;
                  }
                  if (p_second == nullptr)
                  {
                      return true;
                  }
                  return p_first->getId() < p_second->getId();
              });

    for (Plane *p_wall : wallPlanes)
    {
        Plane *p_existingTwin = p_wall->getTwinFace();
        if (p_existingTwin != nullptr)
        {
            /* Cheap common case: re-validate rather than search again. A
             * lower-id plane already validated (and, if still plausible,
             * re-linked) this pair when it was itself visited. */
            if (p_wall->getId() < p_existingTwin->getId())
            {
                continue;
            }

            if (arePlausibleTwinWallFaces(p_wall,
                                          p_existingTwin,
                                          minimumThickness_m,
                                          maximumThickness_m,
                                          minimumOverlapRatio,
                                          groundNormal_World))
            {
                continue;
            }

            /* Pairing is no longer plausible (e.g. one side drifted after a
             * refit) -- unlink both sides rather than leave a stale
             * one-directional pointer. */
            p_existingTwin->clearTwinFace();
            p_wall->clearTwinFace();
        }
    }

    for (std::size_t firstIndex = 0; firstIndex < wallPlanes.size();
         ++firstIndex)
    {
        Plane *p_first = wallPlanes[firstIndex];
        if (p_first->getTwinFace() != nullptr)
        {
            continue;
        }

        Plane *p_bestMatch         = nullptr;
        double bestOverlapRatio_m2 = -1.0;

        for (std::size_t secondIndex = firstIndex + 1;
             secondIndex < wallPlanes.size();
             ++secondIndex)
        {
            Plane *p_second = wallPlanes[secondIndex];
            if (p_second->getTwinFace() != nullptr)
            {
                continue;
            }

            if (!arePlausibleTwinWallFaces(p_first,
                                           p_second,
                                           minimumThickness_m,
                                           maximumThickness_m,
                                           minimumOverlapRatio,
                                           groundNormal_World))
            {
                continue;
            }

            /* Prefer the most-overlapping plausible candidate when more
             * than one exists, using observation count as a simple,
             * deterministic tiebreaker proxy for "most overlap". */
            const double candidateScore =
                static_cast<double>(p_second->getObservationCount());
            if (p_bestMatch == nullptr || candidateScore > bestOverlapRatio_m2)
            {
                p_bestMatch         = p_second;
                bestOverlapRatio_m2 = candidateScore;
            }
        }

        if (p_bestMatch != nullptr)
        {
            p_first->setTwinFace(p_bestMatch);
            p_bestMatch->setTwinFace(p_first);
            std::cout << "[SemMgr] Linked Wall#" << p_first->getId()
                      << " and Wall#" << p_bestMatch->getId()
                      << " as opposite faces of one physical wall."
                      << std::endl;
        }
    }
}

void SemanticsManager::validateRoomBoundaries(void)
{
    std::cout << "[SemMgr] validateRoomBoundaries() called" << std::endl;

    const SystemParams::room_seg::BoundaryTopology &topologyParameters =
        sysParams->room_seg.boundaryTopology;

    if (!topologyParameters.enabled)
    {
        std::cout << "[SemMgr] boundary topology disabled" << std::endl;
        return;
    }

    Plane *p_groundPlane = mpAtlas->GetBiggestGroundPlane();

    if (p_groundPlane == nullptr || p_groundPlane->isBad())
    {
        return;
    }

    Eigen::Vector4d groundEquation_World =
        p_groundPlane->getGlobalEquation().coeffs();
    const double groundNormalNorm = groundEquation_World.head<3>().norm();

    if (!groundEquation_World.allFinite() || groundNormalNorm < 1e-8)
    {
        return;
    }

    const Eigen::Vector3d groundNormal_World =
        groundEquation_World.head<3>() / groundNormalNorm;
    const Eigen::Vector3d groundAxisU_World =
        groundNormal_World.unitOrthogonal().normalized();
    const Eigen::Vector3d groundAxisV_World =
        groundNormal_World.cross(groundAxisU_World).normalized();

    const auto updateBoundaryStatus =
        [](Room                               *p_room_in,
           const Room::BoundaryStatus          boundaryStatus_in,
           const std::vector<Eigen::Vector3d> &corners_World_m_in = {})
    {
        /* Refresh stored corners every cycle the loop is COMPLETE (even when
         * the status itself didn't change -- wall positions can still
         * drift), and clear them the moment it stops being COMPLETE. */
        p_room_in->setBoundaryCorners_World_m(
            boundaryStatus_in == Room::BoundaryStatus::COMPLETE
                ? corners_World_m_in
                : std::vector<Eigen::Vector3d>{});

        const Room::BoundaryStatus previousBoundaryStatus =
            p_room_in->getBoundaryStatus();

        if (previousBoundaryStatus == boundaryStatus_in)
        {
            return;
        }

        p_room_in->setBoundaryStatus(boundaryStatus_in);

        const auto boundaryStatusName = [](const Room::BoundaryStatus status)
        {
            switch (status)
            {
            case Room::BoundaryStatus::UNOBSERVED:
                return "UNOBSERVED";
            case Room::BoundaryStatus::INCOMPLETE:
                return "INCOMPLETE";
            case Room::BoundaryStatus::COMPLETE:
                return "COMPLETE";
            case Room::BoundaryStatus::CONFLICTING:
                return "CONFLICTING";
            }

            return "unknown";
        };

        std::cout << "[SemMgr] Room#" << p_room_in->getId()
                  << " boundary=" << boundaryStatusName(boundaryStatus_in)
                  << " (" << p_room_in->getWalls().size() << " walls)"
                  << std::endl;
    };

    for (Room *p_room : mpAtlas->GetAllRooms())
    {
        if (p_room == nullptr || p_room->isBad())
        {
            continue;
        }

        bool                             boundaryWasRepaired  = true;
        bool                             hasAmbiguousConflict = false;
        std::vector<FiniteWallSegment2d> wallSegments;

        while (boundaryWasRepaired)
        {
            boundaryWasRepaired  = false;
            hasAmbiguousConflict = false;
            wallSegments.clear();

            for (Plane *p_wall : p_room->getWalls())
            {
                FiniteWallSegment2d wallSegment;

                if (buildFiniteWallSegment2d(
                        p_wall,
                        groundNormal_World,
                        groundAxisU_World,
                        groundAxisV_World,
                        topologyParameters.endpointTrimRatio,
                        topologyParameters.minimumWallLength_m,
                        wallSegment))
                {
                    wallSegments.push_back(std::move(wallSegment));
                }
            }

            std::cout << "[SemMgr] Room#" << p_room->getId()
                      << " boundary check: roomWalls="
                      << p_room->getWalls().size()
                      << ", wallSegments=" << wallSegments.size()
                      << ", minWallCount="
                      << topologyParameters.minimumWallCount << std::endl;

            if (wallSegments.size() < topologyParameters.minimumWallCount)
            {
                updateBoundaryStatus(p_room, Room::BoundaryStatus::INCOMPLETE);
                continue;
            }

            for (std::size_t firstWallIndex = 0U;
                 firstWallIndex < wallSegments.size() && !boundaryWasRepaired &&
                 !hasAmbiguousConflict;
                 ++firstWallIndex)
            {
                for (std::size_t secondWallIndex = firstWallIndex + 1U;
                     secondWallIndex < wallSegments.size();
                     ++secondWallIndex)
                {
                    Eigen::Vector2d intersection_World_m;
                    double          firstParameter  = 0.0;
                    double          secondParameter = 0.0;

                    if (!intersectSupportingLines(wallSegments[firstWallIndex],
                                                  wallSegments[secondWallIndex],
                                                  intersection_World_m,
                                                  firstParameter,
                                                  secondParameter) ||
                        firstParameter < 0.0 || firstParameter > 1.0 ||
                        secondParameter < 0.0 || secondParameter > 1.0)
                    {
                        continue;
                    }

                    const double firstInteriorDistance_m =
                        std::min(firstParameter, 1.0 - firstParameter) *
                        wallSegments[firstWallIndex].length_m;
                    const double secondInteriorDistance_m =
                        std::min(secondParameter, 1.0 - secondParameter) *
                        wallSegments[secondWallIndex].length_m;

                    if (std::max(firstInteriorDistance_m,
                                 secondInteriorDistance_m) <=
                        topologyParameters.maximumInteriorIntersection_m)
                    {
                        continue;
                    }

                    const double firstSupport =
                        wallSegments[firstWallIndex].supportScore;
                    const double secondSupport =
                        wallSegments[secondWallIndex].supportScore;
                    const double weakerSupport =
                        std::max(std::min(firstSupport, secondSupport), 1e-8);
                    const double supportRatio =
                        std::max(firstSupport, secondSupport) / weakerSupport;

                    if (supportRatio <
                        topologyParameters.decisiveConflictSupportRatio)
                    {
                        hasAmbiguousConflict = true;
                        break;
                    }

                    Plane *p_rejectedWall =
                        firstSupport < secondSupport
                            ? wallSegments[firstWallIndex].p_wall
                            : wallSegments[secondWallIndex].p_wall;
                    Plane *p_retainedWall =
                        firstSupport < secondSupport
                            ? wallSegments[secondWallIndex].p_wall
                            : wallSegments[firstWallIndex].p_wall;

                    if (p_room->removeWall(p_rejectedWall))
                    {
                        boundaryWasRepaired = true;

                        std::cout << "[SemMgr] Detached clashing Wall#"
                                  << p_rejectedWall->getId() << " from Room#"
                                  << p_room->getId() << "; Wall#"
                                  << p_retainedWall->getId()
                                  << " has decisively stronger finite support."
                                  << std::endl;
                    }

                    break;
                }
            }
        }

        /* Situational awareness for incomplete rooms (user rule): compute
         * this room's unobserved angular sectors from whatever wall
         * evidence currently exists, regardless of the boundary-status
         * outcome below -- this is precisely the "what's still missing"
         * signal a genuinely COMPLETE room no longer needs. Runs ahead of
         * the CONFLICTING/INCOMPLETE/UNOBSERVED branches below so it isn't
         * skipped by any of their early `continue`s. */
        {
            const Eigen::Vector3d gapCentroid_World_m = p_room->getCentroid();
            if (gapCentroid_World_m.allFinite())
            {
                const Eigen::Vector2d gapCentroid_Ground_m(
                    gapCentroid_World_m.dot(groundAxisU_World),
                    gapCentroid_World_m.dot(groundAxisV_World));
                p_room->setObservationGaps(
                    computeRoomObservationGaps(wallSegments,
                                               gapCentroid_Ground_m));
            }
            else
            {
                p_room->setObservationGaps({});
            }
        }

        if (hasAmbiguousConflict)
        {
            updateBoundaryStatus(p_room, Room::BoundaryStatus::CONFLICTING);
            continue;
        }

        if (wallSegments.size() < topologyParameters.minimumWallCount)
        {
            updateBoundaryStatus(p_room, Room::BoundaryStatus::INCOMPLETE);
            continue;
        }

        const Eigen::Vector3d roomCentroid_World_m = p_room->getCentroid();

        if (!roomCentroid_World_m.allFinite())
        {
            updateBoundaryStatus(p_room, Room::BoundaryStatus::UNOBSERVED);
            continue;
        }

        const Eigen::Vector2d roomCentroid_Ground_m(
            roomCentroid_World_m.dot(groundAxisU_World),
            roomCentroid_World_m.dot(groundAxisV_World));

        WallLoopClosure closure = tryCloseWallLoop(wallSegments,
                                                   roomCentroid_Ground_m,
                                                   topologyParameters);

        /* User rule: a wall the room owns but which does not belong to the
         * room's true closed boundary is invalid and must be pruned, not
         * treated as an unrelated reason the whole loop fails to close.
         * The full wallSegments set may include exactly one such outlier
         * (e.g. a wall genuinely belonging to a neighbouring, unlinked
         * room, or a stale duplicate) -- if closing the full set fails, and
         * excluding exactly one wall lets the remainder close cleanly, that
         * excluded wall is the outlier: reassign wallSegments to the
         * closure-achieving subset so every downstream step (self-
         * intersection, area, corner heights, and the loop-membership
         * pruning check below) is consistent, and the excluded wall is
         * naturally caught as "not part of the loop" and detached there
         * (never removed here directly -- that keeps this decision subject
         * to the same passage-explained-ness check as any other off-loop
         * wall). Only a single outlier is handled: searching every subset
         * of exclusions is combinatorial and unnecessary for the case this
         * rule targets. */
        if (closure.hasOpenBoundary &&
            wallSegments.size() > topologyParameters.minimumWallCount)
        {
            std::vector<std::size_t> indicesBySupportAscending(
                wallSegments.size());
            std::iota(indicesBySupportAscending.begin(),
                      indicesBySupportAscending.end(),
                      0U);
            std::sort(
                indicesBySupportAscending.begin(),
                indicesBySupportAscending.end(),
                [&wallSegments](std::size_t firstIndex, std::size_t secondIndex)
                {
                    return wallSegments[firstIndex].supportScore <
                           wallSegments[secondIndex].supportScore;
                });

            for (std::size_t excludeIndex : indicesBySupportAscending)
            {
                std::vector<FiniteWallSegment2d> reducedWallSegments;
                reducedWallSegments.reserve(wallSegments.size() - 1U);
                for (std::size_t segmentIndex = 0U;
                     segmentIndex < wallSegments.size();
                     ++segmentIndex)
                {
                    if (segmentIndex != excludeIndex)
                    {
                        reducedWallSegments.push_back(
                            wallSegments[segmentIndex]);
                    }
                }

                WallLoopClosure reducedClosure =
                    tryCloseWallLoop(reducedWallSegments,
                                     roomCentroid_Ground_m,
                                     topologyParameters);

                if (!reducedClosure.hasOpenBoundary)
                {
                    std::cout
                        << "[SemMgr] Room#" << p_room->getId()
                        << ": excluding Wall#"
                        << wallSegments[excludeIndex].p_wall->getId()
                        << " lets the remaining " << reducedWallSegments.size()
                        << " wall(s) close a valid loop; treating it as an "
                           "off-loop outlier."
                        << std::endl;
                    closure      = reducedClosure;
                    wallSegments = reducedWallSegments;
                    break;
                }
            }
        }

        if (closure.hasOpenBoundary)
        {
            updateBoundaryStatus(p_room, Room::BoundaryStatus::INCOMPLETE);
            continue;
        }

        std::vector<Eigen::Vector2d> boundaryCorners_World_m =
            closure.corners_World_m;

        bool polygonSelfIntersects = false;

        for (std::size_t firstEdgeIndex = 0U;
             firstEdgeIndex < boundaryCorners_World_m.size() &&
             !polygonSelfIntersects;
             ++firstEdgeIndex)
        {
            FiniteWallSegment2d firstBoundaryEdge;
            firstBoundaryEdge.start_World_m =
                boundaryCorners_World_m[firstEdgeIndex];
            firstBoundaryEdge.end_World_m =
                boundaryCorners_World_m[(firstEdgeIndex + 1U) %
                                        boundaryCorners_World_m.size()];

            for (std::size_t secondEdgeIndex = firstEdgeIndex + 1U;
                 secondEdgeIndex < boundaryCorners_World_m.size();
                 ++secondEdgeIndex)
            {
                const bool edgesAreAdjacent =
                    secondEdgeIndex == firstEdgeIndex + 1U ||
                    (firstEdgeIndex == 0U &&
                     secondEdgeIndex + 1U == boundaryCorners_World_m.size());

                if (edgesAreAdjacent)
                {
                    continue;
                }

                FiniteWallSegment2d secondBoundaryEdge;
                secondBoundaryEdge.start_World_m =
                    boundaryCorners_World_m[secondEdgeIndex];
                secondBoundaryEdge.end_World_m =
                    boundaryCorners_World_m[(secondEdgeIndex + 1U) %
                                            boundaryCorners_World_m.size()];

                Eigen::Vector2d intersection_World_m;
                double          firstParameter  = 0.0;
                double          secondParameter = 0.0;

                if (intersectSupportingLines(firstBoundaryEdge,
                                             secondBoundaryEdge,
                                             intersection_World_m,
                                             firstParameter,
                                             secondParameter) &&
                    firstParameter > 1e-6 && firstParameter < 1.0 - 1e-6 &&
                    secondParameter > 1e-6 && secondParameter < 1.0 - 1e-6)
                {
                    polygonSelfIntersects = true;
                    break;
                }
            }
        }

        const double enclosedArea_m2 =
            computePolygonArea_m2(boundaryCorners_World_m);

        std::cout << "[SemMgr] Room#" << p_room->getId()
                  << " boundary validation: walls=" << wallSegments.size()
                  << ", corners=" << boundaryCorners_World_m.size()
                  << ", selfIntersects="
                  << (polygonSelfIntersects ? "true" : "false")
                  << ", area=" << enclosedArea_m2 << " m2"
                  << ", minArea=" << topologyParameters.minimumEnclosedArea_m2
                  << " m2" << std::endl;

        if (polygonSelfIntersects)
        {
            updateBoundaryStatus(p_room, Room::BoundaryStatus::CONFLICTING);
        }
        else if (!std::isfinite(enclosedArea_m2) ||
                 enclosedArea_m2 < topologyParameters.minimumEnclosedArea_m2)
        {
            updateBoundaryStatus(p_room, Room::BoundaryStatus::INCOMPLETE);
        }
        else
        {
            /* Lift the validated 2D ground-tangent corners back into world
             * coordinates: U*axisU + V*axisV recovers the horizontal
             * position exactly (the orthonormal decomposition this loop's
             * own 2D coordinates were built from), and each corner's height
             * is the mean of its two meeting walls' own position along the
             * ground normal. */
            std::vector<Eigen::Vector3d> boundaryCorners3D_World_m;
            boundaryCorners3D_World_m.reserve(boundaryCorners_World_m.size());
            for (std::size_t cornerIndex = 0U;
                 cornerIndex < boundaryCorners_World_m.size();
                 ++cornerIndex)
            {
                const Plane *p_currentCornerWall =
                    wallSegments[cornerIndex].p_wall;
                const Plane *p_nextCornerWall =
                    wallSegments[(cornerIndex + 1U) % wallSegments.size()]
                        .p_wall;
                double height_m = 0.0;
                if (p_currentCornerWall != nullptr &&
                    p_nextCornerWall != nullptr)
                {
                    height_m =
                        0.5 *
                        (p_currentCornerWall->getCentroid().cast<double>().dot(
                             groundNormal_World) +
                         p_nextCornerWall->getCentroid().cast<double>().dot(
                             groundNormal_World));
                }
                boundaryCorners3D_World_m.push_back(
                    boundaryCorners_World_m[cornerIndex].x() *
                        groundAxisU_World +
                    boundaryCorners_World_m[cornerIndex].y() *
                        groundAxisV_World +
                    height_m * groundNormal_World);
            }
            updateBoundaryStatus(p_room,
                                 Room::BoundaryStatus::COMPLETE,
                                 boundaryCorners3D_World_m);

            /* User rule: once a room's boundary is a genuine closed loop,
             * any wall it still owns that is NOT one of that loop's own
             * walls, and that no passage tied to this room explains (i.e.
             * not one of the wall faces framing a doorway out of this
             * room), could never actually have been observed as this
             * room's own boundary -- detach it. This is deliberately
             * intra-room only (mirrors the existing intra-room clash
             * repair in admitWallToRoom()): it only prunes walls the loop
             * computation above already excluded, never a wall that
             * belongs to a different room. */
            std::vector<Plane *> loopWalls;
            loopWalls.reserve(wallSegments.size());
            for (const FiniteWallSegment2d &segment : wallSegments)
            {
                if (segment.p_wall != nullptr)
                {
                    loopWalls.push_back(segment.p_wall);
                }
            }

            const std::vector<Passage *> roomPassages = p_room->getPassages();

            for (Plane *p_ownedWall : p_room->getWalls())
            {
                if (p_ownedWall == nullptr)
                {
                    continue;
                }

                if (std::find(loopWalls.begin(),
                              loopWalls.end(),
                              p_ownedWall) != loopWalls.end())
                {
                    continue;
                }

                const bool explainedByPassage = std::any_of(
                    roomPassages.begin(),
                    roomPassages.end(),
                    [p_ownedWall](Passage *p_passage)
                    {
                        if (p_passage == nullptr)
                        {
                            return false;
                        }
                        const std::vector<Plane *> supportingWalls =
                            p_passage->getAssociateWalls();
                        return std::find(supportingWalls.begin(),
                                         supportingWalls.end(),
                                         p_ownedWall) != supportingWalls.end();
                    });

                if (explainedByPassage)
                {
                    continue;
                }

                if (p_room->removeWall(p_ownedWall))
                {
                    std::cout << "[SemMgr] Room#" << p_room->getId()
                              << "'s boundary is COMPLETE; detached Wall#"
                              << p_ownedWall->getId()
                              << ", which is neither part of the closed wall "
                                 "loop nor explained by any of this room's "
                                 "passages."
                              << std::endl;
                }
            }
        }
    }
}

void SemanticsManager::recomputeRoomCentroidsFromWalls(void)
{
    /* Same inward-nudge-then-average construction as
     * detectRoom_FreeSpaceCluster()'s wall-centroid correction -- see that
     * site's comment for why a plain mean of wall centroids is not
     * guaranteed to land inside the room. Kept in sync with it rather than
     * shared, since this function only runs for non-FREE_SPACE
     * room_seg.method configurations (the FREE_SPACE path, this project's
     * configured default, uses the other site directly). */
    constexpr double centroidInwardOffset_m = 0.10;

    for (ORB_SLAM3::Room *p_room : mpAtlas->GetAllRooms())
    {
        if (p_room == nullptr || p_room->isBad())
        {
            continue;
        }

        const std::vector<ORB_SLAM3::Plane *> roomWalls = p_room->getWalls();
        if (roomWalls.empty())
        {
            continue;
        }

        Eigen::Vector3d wallCentroidSum = Eigen::Vector3d::Zero();
        int             wallCount       = 0;

        for (ORB_SLAM3::Plane *p_wall : roomWalls)
        {
            if (p_wall == nullptr || p_wall->isBad())
            {
                continue;
            }

            const Eigen::Vector3d wallCentroid_World_m =
                p_wall->getCentroid().cast<double>();
            const std::optional<Eigen::Vector3d> inwardNormal_World =
                p_room->getWallNormalTowardRoom_World(p_wall);

            wallCentroidSum +=
                inwardNormal_World
                    ? wallCentroid_World_m +
                          centroidInwardOffset_m * (*inwardNormal_World)
                    : wallCentroid_World_m;
            wallCount++;
        }

        if (wallCount > 0)
        {
            p_room->setCentroid(wallCentroidSum / wallCount);
        }
    }
}

void SemanticsManager::associatePassagesToRooms(void)
{
    /* Extract all rooms from the current map */
    std::vector<ORB_SLAM3::Room *> allRooms = mpAtlas->GetAllRooms();

    /* Extract all passages from the current map */
    const std::vector<ORB_SLAM3::Passage *> allPassages =
        mpAtlas->GetAllPassages();

    constexpr double sideEpsilon_m = 0.20;

    constexpr double maximumSupportingPlaneDistance_m = 1.00;
    constexpr double maximumOpeningEdgeDistance_m     = 3.00;
    constexpr double minimumNormalAlignment           = 0.80;

    /* Stable room ordering makes equal-distance passage associations
     * repeatable. */
    std::sort(allRooms.begin(),
              allRooms.end(),
              [](const Room *p_firstRoom, const Room *p_secondRoom)
              {
                  if (p_firstRoom == nullptr)
                  {
                      return false;
                  }

                  if (p_secondRoom == nullptr)
                  {
                      return true;
                  }

                  return p_firstRoom->getId() < p_secondRoom->getId();
              });

    /*!
     * Snapshot the previous topology before rebuilding it. The semantic pass
     * runs periodically, so logging every unchanged edge as newly associated
     * would obscure genuine topology changes.
     */
    std::unordered_map<ORB_SLAM3::Room *, std::unordered_set<int>>
        previousPassageIdsByRoom;

    /* Rebuild the topology so stale associations cannot survive a remerge. */
    for (ORB_SLAM3::Room *p_room : allRooms)
    {
        if (p_room != nullptr && !p_room->isBad())
        {
            std::unordered_set<int> &previousPassageIds =
                previousPassageIdsByRoom[p_room];

            for (ORB_SLAM3::Passage *p_previousPassage : p_room->getPassages())
            {
                if (p_previousPassage != nullptr)
                {
                    previousPassageIds.insert(p_previousPassage->getId());
                }
            }

            p_room->clearPassages();
        }
    }

    /* Iterate through every passage */
    for (ORB_SLAM3::Passage *p_passage : allPassages)
    {
        /* Skip invalid passages */
        if (p_passage == nullptr || p_passage->isBad())
        {
            continue;
        }

        /* A recovery proxy carries only stable identity and topology. Its
         * historical coordinates deliberately are not copied into the new
         * map frame. Preserve its reciprocal room edge until map alignment
         * can reconcile it with newly observed passage geometry. */
        if (p_passage->isRecoveryProxy())
        {
            const Passage::KnownSideProvenance knownSide =
                p_passage->getKnownSideProvenance();
            Room *p_farSideRoom = p_passage->getProspectiveRoom();
            if (knownSide.pRoom != nullptr && !knownSide.pRoom->isBad())
            {
                knownSide.pRoom->setDoorways(p_passage);
            }
            if (p_farSideRoom != nullptr && !p_farSideRoom->isBad())
            {
                p_farSideRoom->setDoorways(p_passage);
            }
            passageZeroRoomCycles_.erase(p_passage->getId());
            continue;
        }

        /* Extract the wall or walls supporting the passage */
        const std::vector<ORB_SLAM3::Plane *> supportingWalls =
            p_passage->getAssociateWalls();

        /* A passage without a supporting wall cannot connect rooms */
        if (supportingWalls.empty())
        {
            continue;
        }

        /* Extract and normalize the passage plane equation */
        Eigen::Vector4d passageEquation_World =
            p_passage->getGlobalEquation().coeffs();

        const double passageNormalNorm = passageEquation_World.head<3>().norm();

        if (!std::isfinite(passageNormalNorm) || passageNormalNorm < 1e-8)
        {
            continue;
        }

        passageEquation_World /= passageNormalNorm;

        /* Extract the passage centroid in double precision */
        const Eigen::Vector3d passageCentroid_World_m =
            p_passage->getCentroid().cast<double>();

        Passage::KnownSideProvenance knownSide =
            p_passage->getKnownSideProvenance();
        if (!knownSide.hasDirection())
        {
            /* Which side the passage was seen from is a property of the
             * observation that produced its supporting wall face, so take it
             * from that face's stamped observation origin
             * (Plane::getObservationOrigin_World()). Deriving it instead from
             * a median over the wall's whole observation history would
             * migrate to the far side once the UAV flew through this very
             * passage -- inverting the passage's own notion of which side it
             * was discovered from. The median remains only as a fallback for
             * faces created before the stamp existed. */
            for (Plane *p_supportingWall : supportingWalls)
            {
                if (p_supportingWall == nullptr || p_supportingWall->isBad())
                {
                    continue;
                }

                const std::optional<Eigen::Vector3d> observationOrigin_World_m =
                    p_supportingWall->getObservationOrigin_World();

                std::optional<double> observedSide_m;

                if (observationOrigin_World_m.has_value() &&
                    observationOrigin_World_m->allFinite())
                {
                    observedSide_m = passageEquation_World.head<3>().dot(
                                         observationOrigin_World_m.value()) +
                                     passageEquation_World(3);
                }
                else
                {
                    const Plane::ObservationSideSnapshot sideSnapshot =
                        p_supportingWall->getObservationSideSnapshot(
                            passageEquation_World);
                    observedSide_m = sideSnapshot.medianSignedDistance_m;
                }

                if (!observedSide_m.has_value() ||
                    !std::isfinite(observedSide_m.value()))
                {
                    continue;
                }

                p_passage->setKnownSideDirection(
                    observedSide_m.value() > 0.0
                        ? Eigen::Vector3d(passageEquation_World.head<3>())
                        : Eigen::Vector3d(-passageEquation_World.head<3>()));
                knownSide = p_passage->getKnownSideProvenance();
                break;
            }
        }

        /* Track the closest room found on each side of the passage */
        ORB_SLAM3::Room *p_negativeSideRoom             = nullptr;
        ORB_SLAM3::Room *p_positiveSideRoom             = nullptr;
        ORB_SLAM3::Room *p_negativeExactSupportingOwner = nullptr;
        ORB_SLAM3::Room *p_positiveExactSupportingOwner = nullptr;

        double negativeRoomDistance_m = std::numeric_limits<double>::max();
        double positiveRoomDistance_m = std::numeric_limits<double>::max();

        /* Iterate through all valid rooms */
        for (ORB_SLAM3::Room *p_room : allRooms)
        {
            /* Skip invalid rooms */
            if (p_room == nullptr || p_room->isBad())
            {
                continue;
            }

            /* Extract the walls assigned to the room */
            const std::vector<ORB_SLAM3::Plane *> roomWalls =
                p_room->getWalls();

            /*
             * Match either the passage's source wall or a separately observed
             * wall surface at the same physical opening. Adjacent rooms must
             * not share one Plane pointer merely to obtain a graph edge.
             */
            const bool hasSupportingWallGeometry = std::any_of(
                roomWalls.begin(),
                roomWalls.end(),
                [&](ORB_SLAM3::Plane *p_roomWall)
                {
                    if (p_roomWall == nullptr || p_roomWall->isBad())
                    {
                        return false;
                    }

                    if (std::find(supportingWalls.begin(),
                                  supportingWalls.end(),
                                  p_roomWall) != supportingWalls.end())
                    {
                        return true;
                    }

                    const Plane::GeometrySnapshot roomWallGeometry =
                        p_roomWall->getGeometrySnapshot();
                    Eigen::Vector4d roomWallEquation =
                        roomWallGeometry.equation_World;

                    const double roomWallNormalNorm =
                        roomWallEquation.head<3>().norm();

                    if (!roomWallEquation.allFinite() ||
                        roomWallNormalNorm < 1e-8)
                    {
                        return false;
                    }

                    roomWallEquation /= roomWallNormalNorm;

                    const double normalAlignment =
                        std::abs(roomWallEquation.head<3>().dot(
                            passageEquation_World.head<3>()));

                    const double passagePlaneDistance_m =
                        std::abs(roomWallEquation.head<3>().dot(
                                     passageCentroid_World_m) +
                                 roomWallEquation(3));

                    if (normalAlignment < minimumNormalAlignment ||
                        passagePlaneDistance_m >
                            maximumSupportingPlaneDistance_m)
                    {
                        return false;
                    }

                    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr
                        p_roomWallCloud = roomWallGeometry.supportCloud;

                    if (p_roomWallCloud == nullptr || p_roomWallCloud->empty())
                    {
                        return false;
                    }

                    double nearestOpeningEdgeDistance_m =
                        std::numeric_limits<double>::infinity();

                    for (const pcl::PointXYZRGBA &wallPoint :
                         p_roomWallCloud->points)
                    {
                        if (!pcl::isFinite(wallPoint))
                        {
                            continue;
                        }

                        const Eigen::Vector3d wallPoint_World_m(
                            static_cast<double>(wallPoint.x),
                            static_cast<double>(wallPoint.y),
                            static_cast<double>(wallPoint.z));

                        Eigen::Vector3d openingOffset_World_m =
                            wallPoint_World_m - passageCentroid_World_m;

                        openingOffset_World_m -=
                            openingOffset_World_m.dot(
                                roomWallEquation.head<3>()) *
                            roomWallEquation.head<3>();

                        nearestOpeningEdgeDistance_m =
                            std::min(nearestOpeningEdgeDistance_m,
                                     openingOffset_World_m.norm());
                    }

                    return nearestOpeningEdgeDistance_m <=
                           maximumOpeningEdgeDistance_m;
                });

            /* A passage can only belong to a room when the wall it is linked
             * to (its supporting wall) belongs to that room. A room that owns
             * none of the passage's supporting wall surfaces - even one whose
             * free-space skeleton happens to cross the opening - must not be
             * connected to this passage. The consistency knot of the semantic
             * graph is the wall itself: the passage anchors to walls, and
             * walls anchor to exactly one room. */
            if (!hasSupportingWallGeometry)
            {
                continue;
            }

            const bool ownsExactSupportingWall = std::any_of(
                roomWalls.begin(),
                roomWalls.end(),
                [&supportingWalls](Plane *p_roomWall)
                {
                    return std::find(supportingWalls.begin(),
                                     supportingWalls.end(),
                                     p_roomWall) != supportingWalls.end();
                });

            /* Association guard: exact ownership of the passage's supporting
             * wall is definitive adjacency evidence (handled above and
             * below). Anything weaker -- geometric proximity of some other
             * wall plus centroid distance -- may only compete for a side
             * when the room has enough boundary substance to make its
             * centroid meaningful. A single-wall room has no 2D extent; its
             * centroid sits on that one wall and wins whatever passage
             * happens to be nearest (typically right after a map reset,
             * latching the fresh room onto the wrong passage and locking it
             * in via wall ownership). Defer the edge until a second wall
             * arrives rather than invent topology. */
            if (!ownsExactSupportingWall)
            {
                constexpr std::size_t minimumWallsForProximityAssociation = 2U;
                std::size_t           validWallCount                      = 0U;
                for (Plane *p_roomWall : roomWalls)
                {
                    if (p_roomWall != nullptr && !p_roomWall->isBad())
                    {
                        ++validWallCount;
                    }
                }
                if (validWallCount < minimumWallsForProximityAssociation)
                {
                    static std::set<std::pair<int, int>> reportedSparseSkips;
                    if (reportedSparseSkips
                            .emplace(p_passage->getId(), p_room->getId())
                            .second)
                    {
                        std::cout
                            << "[SemMgr] Passage#" << p_passage->getId()
                            << " skipping Room#" << p_room->getId() << " (only "
                            << validWallCount << " valid wall(s); needs "
                            << minimumWallsForProximityAssociation
                            << " without exact supporting-wall ownership)."
                            << std::endl;
                    }
                    continue;
                }
            }

            /* Extract the room centroid */
            const Eigen::Vector3d roomCentroid_World_m = p_room->getCentroid();

            /* Determine which side of the passage plane contains the room */
            const double roomSide_m =
                passageEquation_World.head<3>().dot(roomCentroid_World_m) +
                passageEquation_World(3);

            /*!
             * A wall-centred provisional SE does not yet provide enough
             * evidence to form a room-to-passage connection -- UNLESS the
             * room owns the passage's own exact supporting wall. Owning
             * that wall is definitive, purely semantic (plane-equation)
             * evidence that the room borders this passage; which side of
             * the (nearly coincident, since the wall IS the passage's own
             * plane) passage plane the room's overall centroid happens to
             * land on is not meaningful evidence and must never veto it.
             * A sparsely-observed room (e.g. one confirmed wall so far)
             * can have its centroid sit within sideEpsilon_m purely
             * because that one known wall is this passage's supporting
             * wall -- silently dropping the room-to-passage edge every
             * cycle even though ownsExactSupportingWall already proves
             * the association.
             */
            if (!ownsExactSupportingWall &&
                std::abs(roomSide_m) <= sideEpsilon_m)
            {
                continue;
            }

            /* Find the distance from the room to the passage */
            const double roomDistance_m =
                (roomCentroid_World_m - passageCentroid_World_m).norm();

            /* The centroid of a sparse room can lie on its only known wall.
             * Exact ownership of the passage's supporting wall is still
             * definitive adjacency evidence; use the stamped observation-side
             * provenance to break the otherwise-zero side test. */
            if (ownsExactSupportingWall &&
                std::abs(roomSide_m) <= sideEpsilon_m)
            {
                const double knownSideSign =
                    knownSide.hasDirection()
                        ? knownSide.direction_World.dot(
                              passageEquation_World.head<3>())
                        : 0.0;
                if (knownSideSign >= 0.0 &&
                    roomDistance_m < positiveRoomDistance_m)
                {
                    positiveRoomDistance_m         = roomDistance_m;
                    p_positiveSideRoom             = p_room;
                    p_positiveExactSupportingOwner = p_room;
                }
                else if (knownSideSign < 0.0 &&
                         roomDistance_m < negativeRoomDistance_m)
                {
                    negativeRoomDistance_m         = roomDistance_m;
                    p_negativeSideRoom             = p_room;
                    p_negativeExactSupportingOwner = p_room;
                }
                continue;
            }

            /* Keep the closest room on the negative side */
            if (roomSide_m < 0.0 && roomDistance_m < negativeRoomDistance_m)
            {
                negativeRoomDistance_m = roomDistance_m;
                p_negativeSideRoom     = p_room;
                p_negativeExactSupportingOwner =
                    ownsExactSupportingWall ? p_room : nullptr;
            }

            /* Keep the closest room on the positive side */
            if (roomSide_m > 0.0 && roomDistance_m < positiveRoomDistance_m)
            {
                positiveRoomDistance_m = roomDistance_m;
                p_positiveSideRoom     = p_room;
                p_positiveExactSupportingOwner =
                    ownsExactSupportingWall ? p_room : nullptr;
            }
        }

        /*! Axiom: both rooms a passage links must be on the same floor,
         * except through a vertical passage / staircase (not implemented
         * yet -- see the user's own carve-out). A same-passage,
         * different-floor match is therefore not new information, it is a
         * matching error: since no vertical-passage mechanism exists to
         * produce a genuine one, one of the two sides must be wrong. Keep
         * whichever side is closer to the passage (the stronger match) and
         * drop the farther one back to unresolved for this cycle -- it can
         * still recover in a later cycle, e.g. once its own floor identity
         * is corrected, or a different room wins that side instead. */
        if (p_negativeSideRoom != nullptr && p_positiveSideRoom != nullptr)
        {
            ORB_SLAM3::Floor *p_negativeFloor = p_negativeSideRoom->getFloor();
            ORB_SLAM3::Floor *p_positiveFloor = p_positiveSideRoom->getFloor();

            if (p_negativeFloor != nullptr && p_positiveFloor != nullptr &&
                p_negativeFloor->hasPlaneIdentity() &&
                p_positiveFloor->hasPlaneIdentity() &&
                p_negativeFloor->getId() != p_positiveFloor->getId())
            {
                const bool negativeIsFarther =
                    negativeRoomDistance_m >= positiveRoomDistance_m;
                ORB_SLAM3::Room *p_droppedRoom =
                    negativeIsFarther ? p_negativeSideRoom : p_positiveSideRoom;

                std::cout << "[SemMgr] Passage#" << p_passage->getId()
                          << " matched Room#" << p_negativeSideRoom->getId()
                          << " (Floor#" << p_negativeFloor->getId()
                          << ") and Room#" << p_positiveSideRoom->getId()
                          << " (Floor#" << p_positiveFloor->getId()
                          << ") on different floors -- no vertical passage "
                             "mechanism exists, so this is a matching "
                             "error, not a real staircase; dropping the "
                             "farther match Room#"
                          << p_droppedRoom->getId() << " for this cycle."
                          << std::endl;

                if (negativeIsFarther)
                {
                    p_negativeSideRoom             = nullptr;
                    p_negativeExactSupportingOwner = nullptr;
                }
                else
                {
                    p_positiveSideRoom             = nullptr;
                    p_positiveExactSupportingOwner = nullptr;
                }
            }
        }

        /* Helper which adds a passage to a room without duplicates */
        const auto addPassageToRoom = [p_passage, &previousPassageIdsByRoom](
                                          ORB_SLAM3::Room *p_room_inout)
        {
            /* Skip invalid rooms */
            if (p_room_inout == nullptr)
            {
                return;
            }

            /* Extract the passages already assigned to the room */
            const std::vector<ORB_SLAM3::Passage *> roomPassages =
                p_room_inout->getPassages();

            /* Check whether the relationship already exists */
            const bool alreadyAssociated = std::any_of(
                roomPassages.begin(),
                roomPassages.end(),
                [p_passage](ORB_SLAM3::Passage *p_existingPassage)
                {
                    return p_existingPassage != nullptr &&
                           p_existingPassage->getId() == p_passage->getId();
                });

            /* Add the relationship if required */
            if (!alreadyAssociated)
            {
                p_room_inout->setDoorways(p_passage);

                const auto previousPassagesIterator =
                    previousPassageIdsByRoom.find(p_room_inout);

                const bool relationshipAlreadyExisted =
                    previousPassagesIterator !=
                        previousPassageIdsByRoom.end() &&
                    previousPassagesIterator->second.count(p_passage->getId()) >
                        0U;

                if (!relationshipAlreadyExisted)
                {
                    std::cout << "[SemMgr] Associated Passage#"
                              << p_passage->getId() << " with Room#"
                              << p_room_inout->getId() << "." << std::endl;
                }
            }
        };

        /*!
         * Preserve every geometrically supported room-to-passage edge.
         *
         * A free-space ray may confirm an opening before the room on the far
         * side has enough boundary evidence to exist in the semantic graph.
         * Retaining the known-side edge represents that passage as a frontier
         * without inventing a second room. Once distinct rooms are observed on
         * both sides, the same two edges form the complete room-to-room route.
         */
        addPassageToRoom(p_negativeSideRoom);

        if (p_positiveSideRoom != p_negativeSideRoom)
        {
            addPassageToRoom(p_positiveSideRoom);
        }

        /* Enforce the invariant that a passage is linked to AT MOST TWO rooms
         * (the nearest room on each side of its supporting wall). Rooms that
         * no longer win their side - or that are duplicate hypotheses of the
         * winning side - must have this passage association revoked so the
         * semantic graph never shows a passage with three rooms. */
        for (ORB_SLAM3::Room *p_candidateRoom : allRooms)
        {
            if (p_candidateRoom == nullptr || p_candidateRoom->isBad() ||
                p_candidateRoom == p_negativeSideRoom ||
                p_candidateRoom == p_positiveSideRoom)
            {
                continue;
            }

            if (p_candidateRoom->removePassageAssociation(p_passage))
            {
                std::cout << "[SemMgr] Revoked Passage#" << p_passage->getId()
                          << " from Room#" << p_candidateRoom->getId()
                          << " (enforcing max-2-rooms-per-passage)."
                          << std::endl;
            }
        }

        /* Enforce 1-2 room invariant for this passage */
        std::size_t      associatedRoomCount          = 0;
        std::size_t      confirmedAssociatedRoomCount = 0;
        ORB_SLAM3::Room *p_confirmedAssociatedRoom    = nullptr;
        ORB_SLAM3::Room *p_undefinedAssociatedRoom    = nullptr;

        const auto classifyAssociatedRoom =
            [&confirmedAssociatedRoomCount,
             &p_confirmedAssociatedRoom,
             &p_undefinedAssociatedRoom](ORB_SLAM3::Room *p_room)
        {
            if (p_room->getRoomVariant() ==
                ORB_SLAM3::Room::roomVariant::UNDEFINED)
            {
                p_undefinedAssociatedRoom = p_room;
                return;
            }

            confirmedAssociatedRoomCount++;
            p_confirmedAssociatedRoom = p_room;
        };

        if (p_negativeSideRoom != nullptr)
        {
            associatedRoomCount++;
            classifyAssociatedRoom(p_negativeSideRoom);
        }
        if (p_positiveSideRoom != nullptr &&
            p_positiveSideRoom != p_negativeSideRoom)
        {
            associatedRoomCount++;
            classifyAssociatedRoom(p_positiveSideRoom);
        }

        const double knownSideSign =
            knownSide.hasDirection()
                ? knownSide.direction_World.dot(passageEquation_World.head<3>())
                : 0.0;
        const auto roomIsOnKnownSide =
            [&passageEquation_World, &knownSide, knownSideSign](Room *p_room)
        {
            if (p_room == nullptr || !knownSide.hasDirection() ||
                std::abs(knownSideSign) < 1e-8)
            {
                return false;
            }
            const double roomSide_m =
                passageEquation_World.head<3>().dot(p_room->getCentroid()) +
                passageEquation_World(3);
            return roomSide_m * knownSideSign > 0.0;
        };

        if (knownSide.pRoom == nullptr)
        {
            Room *p_knownSideRoom = roomIsOnKnownSide(p_negativeSideRoom)
                                        ? p_negativeSideRoom
                                        : (roomIsOnKnownSide(p_positiveSideRoom)
                                               ? p_positiveSideRoom
                                               : nullptr);
            if (p_knownSideRoom != nullptr)
            {
                p_passage->setKnownSideRoom(p_knownSideRoom);
                knownSide = p_passage->getKnownSideProvenance();
            }
        }

        if (associatedRoomCount == 0)
        {
            /* A passage linked to no room at all -- real or prospective --
             * is not a valid passage. Give it a short grace period (fresh
             * passages start at 0 rooms for a cycle or two before nearby
             * wall/room evidence catches up) before invalidating it, rather
             * than deleting on the very first zero-room cycle. Passage has
             * no removal from the Atlas, only Plane/Room's isBad()
             * convention (see Passage::setBad()'s own comment). */
            constexpr std::size_t maximumZeroRoomCycles = 5U;
            const std::size_t     zeroRoomCycles =
                ++passageZeroRoomCycles_[p_passage->getId()];

            if (zeroRoomCycles > maximumZeroRoomCycles)
            {
                p_passage->setBad();
                passageZeroRoomCycles_.erase(p_passage->getId());
                std::cout << "[SemMgr] Passage#" << p_passage->getId()
                          << " invalidated: 0 associated rooms for "
                          << zeroRoomCycles << " consecutive cycles."
                          << std::endl;
            }
            else
            {
                std::cout << "[SemMgr] Passage#" << p_passage->getId()
                          << " has 0 associated rooms (" << zeroRoomCycles
                          << "/" << maximumZeroRoomCycles
                          << " grace cycles); camera-side provenance="
                          << (knownSide.hasDirection() ? "known" : "missing")
                          << "." << std::endl;
            }
        }
        else
        {
            passageZeroRoomCycles_.erase(p_passage->getId());
        }

        if (associatedRoomCount > 2)
        {
            std::cout << "[SemMgr] WARNING: Passage#" << p_passage->getId()
                      << " has " << associatedRoomCount
                      << " associated rooms; expected max 2." << std::endl;
        }

        /* ----------------------------------------------------------------------
         * * PROSPECTIVE ROOM CREATION
         * ----------------------------------------------------------------------
         * * When a passage has exactly 1 associated room, create a PROVISIONAL
         * (UNDEFINED variant) room on the far side. This represents the spatial
         * hypothesis that traversable space continues beyond the opening.
         *
         * The prospective room centroid is estimated as:
         *   passage_centroid + passage_normal * estimated_room_depth
         *
         * where passage_normal points from the known room toward the far side.
         * The depth heuristic (0.15 m) deliberately stays close to the
         * passage rather than guessing a typical room depth -- it is a
         * placeholder handle, not a position estimate, and gets corrected
         * the moment real far-side evidence (a wall, a cluster) arrives.
         *
         * Constraints:
         *   - Spatial deduplication: reuse existing prospective room
         * within 2.0m
         *   - Max 12 prospective rooms total (matches office_clean's 12 rooms)
         *   - Max 2 rooms per passage (near + far side)
         *   - Passage pointer is the persistent primary handle
         *   - Wall ownership alone never promotes the prospective
         *   - Validated far-side cluster evidence may promote it in place
         */
        ORB_SLAM3::Room *p_existingProspective =
            p_passage->getProspectiveRoom();
        const bool hadProspectiveHandle = p_existingProspective != nullptr;

        if (p_existingProspective != nullptr && p_existingProspective->isBad())
        {
            p_passage->setProspectiveRoom(nullptr);
        }
        else if (p_existingProspective != nullptr &&
                 p_existingProspective->getRoomVariant() !=
                     ORB_SLAM3::Room::roomVariant::UNDEFINED)
        {
            /* Promotion/replacement keeps the same far-side resolution. */
            p_existingProspective->setDoorways(p_passage);
            prospectiveRoomCycles_.erase(p_existingProspective->getId());
        }

        Room *p_currentFarSideHandle = p_passage->getProspectiveRoom();
        if ((confirmedAssociatedRoomCount == 1U &&
             p_currentFarSideHandle == p_confirmedAssociatedRoom) ||
            (confirmedAssociatedRoomCount == 2U &&
             p_currentFarSideHandle != nullptr &&
             p_currentFarSideHandle != p_negativeSideRoom &&
             p_currentFarSideHandle != p_positiveSideRoom))
        {
            p_passage->setProspectiveRoom(nullptr);
        }

        if (confirmedAssociatedRoomCount == 2U)
        {
            Room *p_farSideConfirmedRoom = nullptr;
            if (knownSide.pRoom == p_negativeSideRoom)
            {
                p_farSideConfirmedRoom = p_positiveSideRoom;
            }
            else if (knownSide.pRoom == p_positiveSideRoom)
            {
                p_farSideConfirmedRoom = p_negativeSideRoom;
            }
            else if (knownSide.hasDirection())
            {
                p_farSideConfirmedRoom = roomIsOnKnownSide(p_negativeSideRoom)
                                             ? p_positiveSideRoom
                                             : p_negativeSideRoom;
            }
            else if (p_negativeExactSupportingOwner != nullptr)
            {
                p_passage->setKnownSideRoom(p_negativeExactSupportingOwner);
                p_passage->setKnownSideDirection(
                    -passageEquation_World.head<3>());
                p_farSideConfirmedRoom = p_positiveSideRoom;
            }
            else if (p_positiveExactSupportingOwner != nullptr)
            {
                p_passage->setKnownSideRoom(p_positiveExactSupportingOwner);
                p_passage->setKnownSideDirection(
                    passageEquation_World.head<3>());
                p_farSideConfirmedRoom = p_negativeSideRoom;
            }
            if (p_farSideConfirmedRoom != nullptr)
            {
                Room *p_previousFarSideHandle = p_passage->getProspectiveRoom();
                p_passage->setProspectiveRoom(p_farSideConfirmedRoom);
                p_farSideConfirmedRoom->setDoorways(p_passage);
                if (p_previousFarSideHandle != p_farSideConfirmedRoom)
                {
                    std::cout << "[SemMgr] Passage#" << p_passage->getId()
                              << " resolved to opposite confirmed Room#"
                              << p_farSideConfirmedRoom->getId()
                              << " with both sides observed." << std::endl;
                }
            }
        }

        if (!p_passage->hasProspectiveRoom() &&
            confirmedAssociatedRoomCount == 1 &&
            p_undefinedAssociatedRoom != nullptr)
        {
            p_passage->setProspectiveRoom(p_undefinedAssociatedRoom);
        }

        if ((confirmedAssociatedRoomCount == 1 ||
             (confirmedAssociatedRoomCount == 0 && knownSide.hasDirection())) &&
            !p_passage->hasProspectiveRoom())
        {
            /* Passage limit: don't create prospective if passage already has 2
             * rooms */
            if (associatedRoomCount >= 2)
            {
                std::cout << "[SemMgr] Passage#" << p_passage->getId()
                          << " already has 2 associated rooms; skipping "
                             "prospective creation."
                          << std::endl;
            }
            else
            {
                Eigen::Vector4d passageEq =
                    p_passage->getGlobalEquation().coeffs();
                const double normalNorm = passageEq.head<3>().norm();

                if (std::isfinite(normalNorm) && normalNorm > 1e-8)
                {
                    passageEq /= normalNorm;
                    const Eigen::Vector3d passageNormal = passageEq.head<3>();
                    const Eigen::Vector3d passageCentroid =
                        p_passage->getCentroid().cast<double>();

                    /* Persisted provenance, not the current camera pose,
                     * defines the side opposite which the stable handle is
                     * created. */
                    ORB_SLAM3::Room *p_knownRoom = p_confirmedAssociatedRoom;
                    Eigen::Vector3d  knownSideDirection =
                        knownSide.hasDirection() ? knownSide.direction_World
                                                  : Eigen::Vector3d::Zero();
                    if (!knownSide.hasDirection() && p_knownRoom != nullptr)
                    {
                        const double knownRoomSide =
                            passageNormal.dot(p_knownRoom->getCentroid()) +
                            passageEq(3);
                        knownSideDirection = knownRoomSide < 0.0
                                                 ? -passageNormal
                                                 : passageNormal;
                        p_passage->setKnownSideDirection(knownSideDirection);
                        p_passage->setKnownSideRoom(p_knownRoom);
                    }
                    const Eigen::Vector3d farSideNormal = -knownSideDirection;

                    /*!
                     * Anti-churn gate: a passage whose far side already holds a
                     * confirmed room must not create (and then immediately
                     * delete) a prospective placeholder for that same space.
                     * Matches the promotion search exactly, so nothing flips
                     * between created-this-cycle and promoted-next-cycle.
                     */
                    bool             farSideConfirmedRoomExists = false;
                    ORB_SLAM3::Room *p_existingFarSideRoom      = nullptr;

                    /*!
                     * Same class of flaw as the promotion search further
                     * below (and the same fix): a bare same-side-of-the-
                     * infinite-passage-plane sign test, with only a 0.05m
                     * epsilon, is satisfied by any room past this passage
                     * on the far side -- including a room several doors
                     * down the same corridor that is nowhere near this
                     * specific opening. This gate runs BEFORE any
                     * prospective placeholder exists, so it must carry the
                     * same rigor itself rather than relying on the
                     * promotion search to catch it later: the bounded
                     * aperture test (segmentCrossesPassageOpening) plus the
                     * intervening-wall test (segmentCrossesForeignWall).
                     */
                    Plane *p_anteChurnGroundPlane =
                        mpAtlas->GetBiggestGroundPlane();

                    if (p_knownRoom != nullptr &&
                        p_anteChurnGroundPlane != nullptr &&
                        !p_anteChurnGroundPlane->isBad())
                    {
                        const Eigen::Vector4d anteChurnGroundEq =
                            p_anteChurnGroundPlane->getGlobalEquation()
                                .coeffs();
                        const double anteChurnGroundNorm =
                            anteChurnGroundEq.head<3>().norm();

                        if (anteChurnGroundEq.allFinite() &&
                            anteChurnGroundNorm > 1e-8)
                        {
                            const Eigen::Vector3d anteChurnGroundNormal_World =
                                anteChurnGroundEq.head<3>() /
                                anteChurnGroundNorm;
                            const Eigen::Vector3d anteChurnGroundAxisU_World =
                                anteChurnGroundNormal_World.unitOrthogonal()
                                    .normalized();
                            const Eigen::Vector3d anteChurnGroundAxisV_World =
                                anteChurnGroundNormal_World
                                    .cross(anteChurnGroundAxisU_World)
                                    .normalized();
                            const SystemParams::room_seg::PassagePartition
                                &anteChurnPartitionParameters =
                                    sysParams->room_seg.passagePartition;
                            const double anteChurnOpeningMargin_m =
                                static_cast<double>(anteChurnPartitionParameters
                                                        .openingMargin_m);
                            const double anteChurnMinimumSideDistance_m =
                                static_cast<double>(anteChurnPartitionParameters
                                                        .minimumSideDistance_m);
                            const SystemParams::room_seg::BoundaryTopology
                                &anteChurnTopologyParameters =
                                    sysParams->room_seg.boundaryTopology;
                            const Eigen::Vector3d knownRoomCentroid =
                                p_knownRoom->getCentroid();
                            const std::vector<ORB_SLAM3::Room *>
                                anteChurnExcludedRooms = {p_knownRoom};

                            for (ORB_SLAM3::Room *p_otherRoom : allRooms)
                            {
                                if (p_otherRoom == nullptr ||
                                    p_otherRoom->isBad() ||
                                    p_otherRoom == p_knownRoom ||
                                    p_otherRoom->getRoomVariant() ==
                                        ORB_SLAM3::Room::roomVariant::UNDEFINED)
                                {
                                    continue;
                                }

                                if (!segmentCrossesPassageOpening(
                                        knownRoomCentroid,
                                        p_otherRoom->getCentroid(),
                                        p_passage,
                                        anteChurnGroundNormal_World,
                                        anteChurnOpeningMargin_m,
                                        anteChurnMinimumSideDistance_m))
                                {
                                    continue;
                                }

                                if (segmentCrossesForeignWall(
                                        knownRoomCentroid,
                                        p_otherRoom->getCentroid(),
                                        anteChurnExcludedRooms,
                                        allRooms,
                                        anteChurnGroundAxisU_World,
                                        anteChurnGroundAxisV_World,
                                        anteChurnGroundNormal_World,
                                        anteChurnTopologyParameters
                                            .endpointTrimRatio,
                                        anteChurnTopologyParameters
                                            .minimumWallLength_m))
                                {
                                    continue;
                                }

                                farSideConfirmedRoomExists = true;
                                p_existingFarSideRoom      = p_otherRoom;
                                break;
                            }
                        }
                    }

                    if (farSideConfirmedRoomExists)
                    {
                        if (p_existingFarSideRoom != nullptr)
                        {
                            p_existingFarSideRoom->setDoorways(p_passage);
                            p_passage->setProspectiveRoom(
                                p_existingFarSideRoom);

                            std::cout << "[SemMgr] Passage#"
                                      << p_passage->getId()
                                      << " resolved directly to confirmed Room#"
                                      << p_existingFarSideRoom->getId()
                                      << " on the far side." << std::endl;
                        }
                    }
                    else
                    {
                        /* Placeholder handle for "some room exists on the
                         * far side of this doorway," not a real position
                         * estimate -- it gets corrected the moment any real
                         * far-side evidence (a wall, a cluster) arrives. Kept
                         * close to the passage rather than out at a typical
                         * room's centre depth so it doesn't visually or
                         * spatially masquerade as a real room position in
                         * the meantime. */
                        constexpr double      estimatedRoomDepth_m = 0.15;
                        const Eigen::Vector3d prospectiveCentroid =
                            passageCentroid +
                            farSideNormal * estimatedRoomDepth_m;

                        /* SPATIAL DEDUPLICATION: Check if a
                         * candidate/prospective room already exists near this
                         * location (within 2.0m) across ALL passages. */
                        bool prospectiveExists = false;
                        const std::vector<ORB_SLAM3::Room *> candidateRooms =
                            mpAtlas->GetAllCandidateMapRooms();

                        /* Count current prospective rooms (UNDEFINED variant
                         * candidates) */
                        int prospectiveRoomCount = 0;
                        for (ORB_SLAM3::Room *p_candidate : candidateRooms)
                        {
                            if (p_candidate != nullptr &&
                                !p_candidate->isBad() &&
                                p_candidate->getRoomVariant() ==
                                    ORB_SLAM3::Room::roomVariant::UNDEFINED)
                            {
                                prospectiveRoomCount++;
                            }
                        }

                        /* Enforce max prospective rooms cap */
                        if (prospectiveRoomCount >= kMaxProspectiveRooms &&
                            !p_passage->isPassable() &&
                            !p_passage->getTraversalEvidence() &&
                            !hadProspectiveHandle)
                        {
                            std::cout
                                << "[SemMgr] Max prospective rooms ("
                                << kMaxProspectiveRooms
                                << ") reached; skipping creation for Passage#"
                                << p_passage->getId() << std::endl;
                        }
                        else
                        {
                            for (ORB_SLAM3::Room *p_candidate : candidateRooms)
                            {
                                if (p_candidate == nullptr ||
                                    p_candidate->isBad() ||
                                    p_candidate->getRoomVariant() !=
                                        ORB_SLAM3::Room::roomVariant::UNDEFINED)
                                {
                                    continue;
                                }
                                const Eigen::Vector3d candidateCentroid =
                                    p_candidate->getCentroid();
                                const double dist =
                                    (candidateCentroid - prospectiveCentroid)
                                        .norm();

                                bool passageIdentityMatches = false;
                                if (dist <= kProspectiveDedupDistance_m)
                                {
                                    for (Passage *p_candidatePassage :
                                         allPassages)
                                    {
                                        if (p_candidatePassage == nullptr ||
                                            p_candidatePassage == p_passage ||
                                            p_candidatePassage
                                                    ->getProspectiveRoom() !=
                                                p_candidate)
                                        {
                                            continue;
                                        }

                                        Eigen::Vector4d candidatePassageEq =
                                            p_candidatePassage
                                                ->getGlobalEquation()
                                                .coeffs();
                                        const double candidatePassageNorm =
                                            candidatePassageEq.head<3>().norm();
                                        if (!candidatePassageEq.allFinite() ||
                                            candidatePassageNorm < 1e-8)
                                        {
                                            continue;
                                        }
                                        candidatePassageEq /=
                                            candidatePassageNorm;

                                        const double openingDistance_m =
                                            (p_candidatePassage->getCentroid() -
                                             passageCentroid)
                                                .norm();
                                        const double normalAlignment = std::abs(
                                            candidatePassageEq.head<3>().dot(
                                                passageEq.head<3>()));
                                        const double planeResidual_m =
                                            std::abs(passageEq.head<3>().dot(
                                                         p_candidatePassage
                                                             ->getCentroid()) +
                                                     passageEq(3));

                                        if (openingDistance_m <=
                                                sysParams->sem_seg
                                                    .passageDetection
                                                    .duplicatePassageDistance_m &&
                                            normalAlignment >=
                                                sysParams->sem_seg
                                                    .passageDetection
                                                    .duplicateNormalAlignment &&
                                            planeResidual_m <= 0.30)
                                        {
                                            passageIdentityMatches = true;
                                            break;
                                        }
                                    }
                                }

                                if (passageIdentityMatches)
                                {
                                    /* Cross-passage reuse requires equivalent
                                     * supporting-plane/opening geometry. */
                                    p_passage->setProspectiveRoom(p_candidate);
                                    p_candidate->setDoorways(p_passage);
                                    prospectiveExists = true;
                                    std::cout << "[SemMgr] Reusing existing "
                                                 "prospective Room#"
                                              << p_candidate->getId()
                                              << " (dist=" << dist
                                              << "m) for Passage#"
                                              << p_passage->getId()
                                              << std::endl;
                                    break;
                                }
                            }

                            if (!prospectiveExists)
                            {
                                /* Create the prospective room */
                                ORB_SLAM3::Room *p_prospectiveRoom =
                                    GeoSemHelpers::createBlankRoomCandidate(
                                        mpAtlas,
                                        prospectiveCentroid);

                                if (p_prospectiveRoom != nullptr)
                                {
                                    /* Mark as provisional - will be promoted
                                     * when walls are observed */
                                    p_prospectiveRoom->setRoomVariant(
                                        ORB_SLAM3::Room::roomVariant::
                                            UNDEFINED);
                                    p_prospectiveRoom->setName(
                                        "Prospective#" +
                                        std::to_string(
                                            p_prospectiveRoom->getId()));

                                    /* Add to atlas as a candidate (not yet a
                                     * confirmed room) */
                                    mpAtlas->AddCandidateMapRoom(
                                        p_prospectiveRoom);

                                    /* Link passage <-> prospective room */
                                    p_passage->setProspectiveRoom(
                                        p_prospectiveRoom);
                                    p_prospectiveRoom->setDoorways(p_passage);

                                    /* Register the live passage-created handle.
                                     */
                                    prospectiveRoomCycles_[p_prospectiveRoom
                                                               ->getId()] = 0;

                                    std::cout
                                        << "[SemMgr] Created prospective Room#"
                                        << p_prospectiveRoom->getId() << " at "
                                        << prospectiveCentroid.transpose()
                                        << " for Passage#" << p_passage->getId()
                                        << " (total prospective: "
                                        << prospectiveRoomCount + 1 << ")"
                                        << std::endl;
                                }
                            }
                        }
                    }
                }
            }
        }

        if ((p_passage->isPassable() || p_passage->getTraversalEvidence()) &&
            (confirmedAssociatedRoomCount > 0U || knownSide.hasDirection()) &&
            !p_passage->hasProspectiveRoom())
        {
            std::cerr << "[SemMgr] WARNING: Passage#" << p_passage->getId()
                      << " has a confirmed side but no stable far-side handle; "
                         "it is not routable this cycle."
                      << std::endl;
        }

        /* ----------------------------------------------------------------------
         * * PROSPECTIVE ROOM PROMOTION
         * ----------------------------------------------------------------------
         * * Wall ownership alone cannot promote a prospective. Promotion is
         * performed only by detectRoom_FreeSpaceCluster() after an independent
         * far-side cluster matches the prospective's existing walls. If a
         * distinct confirmed room already resolves the far side, retire the
         * placeholder and preserve that room as the passage's stable handle.
         */
        if (p_passage->hasProspectiveRoom())
        {
            ORB_SLAM3::Room *p_prospectiveRoom =
                p_passage->getProspectiveRoom();

            if (p_prospectiveRoom != nullptr && !p_prospectiveRoom->isBad() &&
                p_prospectiveRoom->getRoomVariant() ==
                    ORB_SLAM3::Room::roomVariant::UNDEFINED &&
                !p_prospectiveRoom->getWalls().empty())
            {
                /* Find a confirmed (non-prospective) room on the FAR side of
                 * the passage - i.e. on the same side as the prospective room.
                 * Use ANY such room, not just the strictly associated side
                 * rooms, so that a passage opening onto a corridor or another
                 * already-mapped room stops being prospective as soon as that
                 * room exists. When one is found, the prospective placeholder
                 * is deleted - it must not linger as a loose provisional room.
                 */
                ORB_SLAM3::Room *p_farSideConfirmedRoom = nullptr;

                /*!
                 * Per WP8-B's own driving principle: "a wall observed on the
                 * far side of ANY confirmed passage aperture belongs to that
                 * passage's prospective room... decide which side of every
                 * passage it lies on" -- using segmentCrossesPassageOpening,
                 * the same purely semantic (passage width/height aperture,
                 * not voxblox cluster geometry) test already used for wall
                 * admission (e.g. line ~3759). A same-side-of-the-infinite-
                 * plane sign test is NOT proof of adjacency: a room several
                 * metres past this passage (only reachable through an
                 * intervening, not-yet-confirmed room) satisfies "same
                 * side" just as well as a genuinely bordering room does.
                 * Observed directly: Passage#1 sitting between Room#4 and a
                 * distant Room#5 kept resolving straight to Room#5,
                 * destroying the middle prospective room meant to sit
                 * between them, every cycle. Testing whether the segment
                 * between the two room centroids actually threads through
                 * THIS passage's own bounded opening (not just crosses its
                 * infinite plane somewhere) rejects that distant match
                 * without any distance threshold borrowed from an unrelated
                 * (voxblox free-space cluster) subsystem.
                 *
                 * This still isn't sufficient on its own when the
                 * prospective room is nothing but its creation-time
                 * heuristic position (passage_centroid + normal * an
                 * assumed depth, before any real wall has been admitted to
                 * it): in a straight corridor with several doors in a row,
                 * that guessed point and a genuinely distant, unrelated
                 * room can both sit close enough to the corridor centreline
                 * for the straight segment between them to thread THIS
                 * passage's aperture too, even though a different room and
                 * passage lie directly between them. Observed directly:
                 * a freshly created prospective room, still with zero
                 * walls, resolved straight to a confirmed room three doors
                 * down the same corridor on the very cycle it was created.
                 * Requiring at least one real, admitted wall first (the
                 * same evidence bar promotion already applies below: "wall
                 * ownership alone cannot promote... validated far-side
                 * cluster evidence may promote") anchors the near endpoint
                 * of the crossing test to an actually observed position
                 * instead of an unvalidated depth guess.
                 *
                 * Also excludes the passage's own near-side/known room from
                 * candidacy, mirroring the creation-time anti-churn guard
                 * above (§7342-7374) -- a room that owns the passage's own
                 * near side must never be matched as its far side.
                 */
                ORB_SLAM3::Room *p_knownSideRoom =
                    p_passage->getKnownSideProvenance().pRoom;

                Plane *p_groundPlane = mpAtlas->GetBiggestGroundPlane();

                if (p_groundPlane != nullptr && !p_groundPlane->isBad())
                {
                    const Eigen::Vector4d groundEquation_World =
                        p_groundPlane->getGlobalEquation().coeffs();
                    const double groundNormalNorm =
                        groundEquation_World.head<3>().norm();

                    if (groundEquation_World.allFinite() &&
                        groundNormalNorm > 1e-8)
                    {
                        const Eigen::Vector3d groundNormal_World =
                            groundEquation_World.head<3>() / groundNormalNorm;
                        const Eigen::Vector3d groundAxisU_World =
                            groundNormal_World.unitOrthogonal().normalized();
                        const Eigen::Vector3d groundAxisV_World =
                            groundNormal_World.cross(groundAxisU_World)
                                .normalized();
                        const SystemParams::room_seg::PassagePartition
                            &partitionParameters =
                                sysParams->room_seg.passagePartition;
                        const double openingMargin_m = static_cast<double>(
                            partitionParameters.openingMargin_m);
                        const double minimumSideDistance_m =
                            static_cast<double>(
                                partitionParameters.minimumSideDistance_m);
                        const SystemParams::room_seg::BoundaryTopology
                            &topologyParameters =
                                sysParams->room_seg.boundaryTopology;
                        const Eigen::Vector3d prospectiveCentroid =
                            p_prospectiveRoom->getCentroid();
                        const std::vector<ORB_SLAM3::Room *> excludedRooms = {
                            p_prospectiveRoom,
                            p_knownSideRoom};

                        for (ORB_SLAM3::Room *p_otherRoom : allRooms)
                        {
                            if (p_otherRoom == nullptr ||
                                p_otherRoom->isBad() ||
                                p_otherRoom == p_prospectiveRoom ||
                                p_otherRoom == p_knownSideRoom ||
                                p_otherRoom->getRoomVariant() ==
                                    ORB_SLAM3::Room::roomVariant::UNDEFINED)
                            {
                                continue;
                            }

                            if (!segmentCrossesPassageOpening(
                                    prospectiveCentroid,
                                    p_otherRoom->getCentroid(),
                                    p_passage,
                                    groundNormal_World,
                                    openingMargin_m,
                                    minimumSideDistance_m))
                            {
                                continue;
                            }

                            /*!
                             * A candidate that threads this passage's own
                             * aperture is still not a legitimate match when
                             * the straight line to it is blocked by another
                             * room's own wall -- see segmentCrossesForeignWall
                             * above. Observed directly: a prospective
                             * placeholder's heuristic position resolved
                             * straight to a confirmed room three doors down
                             * the same corridor, with the true intervening
                             * room's own wall sitting directly on that line.
                             */
                            if (segmentCrossesForeignWall(
                                    prospectiveCentroid,
                                    p_otherRoom->getCentroid(),
                                    excludedRooms,
                                    allRooms,
                                    groundAxisU_World,
                                    groundAxisV_World,
                                    groundNormal_World,
                                    topologyParameters.endpointTrimRatio,
                                    topologyParameters.minimumWallLength_m))
                            {
                                continue;
                            }

                            p_farSideConfirmedRoom = p_otherRoom;
                            break;
                        }
                    }
                }

                if (p_farSideConfirmedRoom != nullptr)
                {
                    /* Transfer uniquely held evidence before retiring the
                     * distinct placeholder. Failed admissions remain unowned
                     * for the normal wall-association pass below. */
                    p_passage->setProspectiveRoom(p_farSideConfirmedRoom);
                    for (ORB_SLAM3::Plane *p_wall :
                         p_prospectiveRoom->getWalls())
                    {
                        admitWallToRoom(p_farSideConfirmedRoom, p_wall);
                        p_prospectiveRoom->removeWall(p_wall);
                    }

                    prospectiveRoomCycles_.erase(p_prospectiveRoom->getId());

                    Map *p_roomMap = p_prospectiveRoom->getMap();
                    if (p_roomMap != nullptr)
                    {
                        p_roomMap->EraseMarkerBasedMapRoom(p_prospectiveRoom);
                    }

                    p_prospectiveRoom->clearPassages();
                    p_prospectiveRoom->setBad();

                    p_farSideConfirmedRoom->setDoorways(p_passage);

                    std::cout
                        << "[SemMgr] Resolved prospective Room#"
                        << p_prospectiveRoom->getId() << " with confirmed Room#"
                        << p_farSideConfirmedRoom->getId() << " for Passage#"
                        << p_passage->getId() << "." << std::endl;
                }
            }
        }
    }

    /* Report, but never fabricate, missing passage connectivity. */
    std::unordered_set<int> disconnectedRoomIds;

    for (ORB_SLAM3::Room *p_room : allRooms)
    {
        if (p_room == nullptr || p_room->isBad() ||
            p_room->getRoomVariant() ==
                ORB_SLAM3::Room::roomVariant::UNDEFINED ||
            !p_room->getPassages().empty())
        {
            continue;
        }

        disconnectedRoomIds.insert(p_room->getId());

        if (disconnectedRoomIds_.count(p_room->getId()) == 0U)
        {
            std::cout << "[SemMgr] Room#" << p_room->getId()
                      << " is not yet connected by a confirmed passage; "
                         "semantic routing will treat it as disconnected."
                      << std::endl;
        }
    }

    disconnectedRoomIds_ = std::move(disconnectedRoomIds);
}

void SemanticsManager::RequestFinish(void)
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool SemanticsManager::CheckFinish(void)
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void SemanticsManager::SetFinish(void)
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool SemanticsManager::isFinished(void)
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinished;
}

} // namespace ORB_SLAM3

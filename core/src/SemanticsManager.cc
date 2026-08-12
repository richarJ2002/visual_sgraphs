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
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
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
    Plane          *p_wall        = nullptr;
    Eigen::Vector2d start_World_m = Eigen::Vector2d::Zero();
    Eigen::Vector2d end_World_m   = Eigen::Vector2d::Zero();
    double          length_m      = 0.0;
    double          supportScore  = 0.0;
};

struct WallAdmissionEvidence
{
    bool        admissible        = false;
    bool        adequateFiniteFit = false;
    std::size_t finitePointCount  = 0U;
    std::size_t fittedPointCount  = 0U;
    std::size_t observationCount  = 0U;
};

WallAdmissionEvidence
    evaluateWallAdmissionEvidence(Plane              *p_wall_in,
                                  const SystemParams *p_systemParams_in)
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

    const Eigen::Vector3d axisU_World =
        normal_World.unitOrthogonal().normalized();
    const Eigen::Vector3d axisV_World =
        normal_World.cross(axisU_World).normalized();
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
 * @brief Builds a robust finite wall segment on the horizontal ground plane.
 *
 * @param[in] p_wall_in Wall whose observed cloud defines the finite extent.
 * @param[in] groundNormal_World_in Unit ground normal in the world frame.
 * @param[in] groundAxisU_World_in First horizontal ground axis.
 * @param[in] groundAxisV_World_in Second horizontal ground axis.
 * @param[in] endpointTrimRatio_in Fraction trimmed from both extent tails.
 * @param[in] minimumWallLength_m_in Minimum accepted horizontal length.
 * @param[out] segment_out Resulting finite horizontal segment.
 * @return True when the wall provides a valid finite segment.
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
 * @brief Intersects the infinite lines supporting two finite wall segments.
 *
 * @param[in] firstSegment_in First wall segment.
 * @param[in] secondSegment_in Second wall segment.
 * @param[out] intersection_World_m_out Intersection in horizontal world axes.
 * @param[out] firstParameter_out Parametric coordinate on the first segment.
 * @param[out] secondParameter_out Parametric coordinate on the second segment.
 * @return False when the supporting lines are parallel.
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

/*!
 * @brief Computes the unsigned area of an ordered horizontal polygon.
 */
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
        (requirePassable_in && !p_passage_in->isPassable()) ||
        !segmentStart_World_m_in.allFinite() ||
        !segmentEnd_World_m_in.allFinite())
    {
        return false;
    }

    Eigen::Vector4d passageEquation_World =
        p_passage_in->getGlobalEquation().coeffs();
    const double passageNormalNorm = passageEquation_World.head<3>().norm();

    if (!passageEquation_World.allFinite() || passageNormalNorm < 1e-8)
    {
        return false;
    }

    passageEquation_World /= passageNormalNorm;
    const Eigen::Vector3d passageNormal_World = passageEquation_World.head<3>();
    const double          startSide_m =
        passageNormal_World.dot(segmentStart_World_m_in) +
        passageEquation_World(3);
    const double endSide_m = passageNormal_World.dot(segmentEnd_World_m_in) +
                             passageEquation_World(3);

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
    const Eigen::Vector3d passageCentroid_World_m = p_passage_in->getCentroid();

    if (!passageCentroid_World_m.allFinite())
    {
        return false;
    }

    Eigen::Vector3d apertureOffset_World_m =
        intersection_World_m - passageCentroid_World_m;
    apertureOffset_World_m -=
        apertureOffset_World_m.dot(passageNormal_World) * passageNormal_World;

    const double verticalOffset_m =
        std::abs(apertureOffset_World_m.dot(groundNormal_World_in));
    const Eigen::Vector3d horizontalOffset_World_m =
        apertureOffset_World_m -
        apertureOffset_World_m.dot(groundNormal_World_in) *
            groundNormal_World_in;
    const double horizontalOffset_m = horizontalOffset_World_m.norm();

    return horizontalOffset_m <=
               0.5 * p_passage_in->getWidth() + openingMargin_m_in &&
           verticalOffset_m <=
               0.5 * p_passage_in->getHeight() + openingMargin_m_in;
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
    trackerConfig.reacquire_retry_interval_s =
        static_cast<double>(sysParams->room_tracking.reacquire_retry_interval_s);
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

void SemanticsManager::Run(void)
{
    std::size_t summaryCycle = 0U;

    while (true)
    {
        /* Find the current time of the loop */
        const std::chrono::_V2::system_clock::time_point start =
            std::chrono::high_resolution_clock::now();

        /*!
         * Treat one hierarchy update as an atomic semantic transaction. Map
         * merging takes the same atlas-owned lock before changing frames or
         * ownership, so this cycle can never traverse a half-merged graph.
         */
        std::unique_lock<std::mutex> semanticUpdateLock =
            mpAtlas->acquireSemanticUpdateLock();

        resetTemporalStateForMap(mpAtlas->GetCurrentMap());

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

        /*  Detect passages after room-wall membership is current */
        if (sysParams->sem_seg.enable_passage_detection)
        {
            detectDoorsAndDoorways(mpAtlas);
            updatePassages(mpAtlas);
            updateTraversalEvidence(mpAtlas);
            Utils::reAssociatePassages(mpAtlas);
            associatePassagesToRooms();
            detachWallsBeyondConfirmedPassages();

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
                    if (p_passage != nullptr &&
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

        /* Propagate room identity from prior-map context snapshots to untagged
         * rooms in the current map. Triggered after room detection so rooms
         * exist to be matched. */
        mpAtlas->matchRoomsToContext(mpAtlas->GetCurrentMap());

        /* Advance the room-state machine (WP13 Section 18.2). It consumes the
         * traversal crossings recorded above and reports accepted/rejected
         * transitions. It is read-only with respect to the room id members. */
        updateRoomTrackerState();

        /*!
         * Deterministically merge one map pair per cycle.
         *
         * A new map becomes a merge candidate once both maps agree on a
         * tagged room name. Atlas::MergeMapPair() transforms the other map's
         * semantic graph into the current map's frame, fuses duplicate rooms,
         * re-associates passages, and retires the other map. The semantic
         * lock acquired above is held for the whole merge, and only one pair
         * is merged per cycle because the active map list changes mid-merge.
         */
        Map *pCurrentMap = mpAtlas->GetCurrentMap();

        if (pCurrentMap != nullptr && !pCurrentMap->IsBad())
        {
            const vector<Map *> allMaps = mpAtlas->GetAllMaps();
            std::cout << "[SemMgr] Merge check: current map "
                      << pCurrentMap->GetId()
                      << ", total maps: " << allMaps.size() << std::endl;
            for (Map *p_otherMap : allMaps)
            {
                /* Skip the survivor, retired maps, and inactive maps. */
                if (p_otherMap == nullptr || p_otherMap == pCurrentMap ||
                    p_otherMap->IsBad() || !mpAtlas->isActiveMap(p_otherMap))
                {
                    continue;
                }

                std::cout << "[SemMgr] Checking merge with map "
                          << p_otherMap->GetId() << std::endl;

                /* Merge only when both maps share a tagged room name. */
                if (sharesRoomNameTag(pCurrentMap, p_otherMap))
                {
                    std::cout << "[SemanticsManager] Merging map "
                              << p_otherMap->GetId() << " into current map "
                              << pCurrentMap->GetId() << std::endl;

                    mpAtlas->MergeMapPair(pCurrentMap, p_otherMap);
                    break;
                }
            }
        }

        /* ------------------------------------------------------------------ *
         * SEMANTIC STATE SUMMARY (validation aid)
         * Emit a compact, machine-grepable snapshot of the semantic topology
         * once per cycle so run-time results can be validated from text.
         * ------------------------------------------------------------------ */
        const std::size_t snapshotCycle = ++summaryCycle;
        const auto        getMapId      = [](ORB_SLAM3::Map *p_map) {
            return p_map != nullptr ? static_cast<long long>(p_map->GetId())
                                                : -1LL;
        };
        const auto formatSortedIds = [](std::vector<int> ids)
        {
            std::sort(ids.begin(), ids.end());
            std::ostringstream stream;
            stream << '[';
            for (std::size_t index = 0U; index < ids.size(); ++index)
            {
                if (index > 0U)
                {
                    stream << ',';
                }
                stream << ids[index];
            }
            stream << ']';
            return stream.str();
        };

        std::vector<ORB_SLAM3::Room *> summaryRooms = mpAtlas->GetAllRooms();
        std::sort(
            summaryRooms.begin(),
            summaryRooms.end(),
            [&getMapId](ORB_SLAM3::Room *p_first, ORB_SLAM3::Room *p_second)
            {
                if (p_first == nullptr)
                {
                    return false;
                }
                if (p_second == nullptr)
                {
                    return true;
                }
                return std::make_pair(getMapId(p_first->getMap()),
                                      p_first->getId()) <
                       std::make_pair(getMapId(p_second->getMap()),
                                      p_second->getId());
            });

        std::cout << "[SemMgrSummary] SNAPSHOT_BEGIN cycle=" << snapshotCycle
                  << std::endl;
        std::cout << "[SemMgrSummary] === semantic state ===" << std::endl;
        std::cout << "[SemMgrSummary] --- planes ---" << std::endl;
        std::vector<ORB_SLAM3::Plane *> summaryPlanes = mpAtlas->GetAllPlanes();
        std::sort(
            summaryPlanes.begin(),
            summaryPlanes.end(),
            [&getMapId](ORB_SLAM3::Plane *p_first, ORB_SLAM3::Plane *p_second)
            {
                if (p_first == nullptr)
                {
                    return false;
                }
                if (p_second == nullptr)
                {
                    return true;
                }
                return std::make_pair(getMapId(p_first->GetMap()),
                                      p_first->getId()) <
                       std::make_pair(getMapId(p_second->GetMap()),
                                      p_second->getId());
            });
        for (ORB_SLAM3::Plane *p_summaryPlane : summaryPlanes)
        {
            if (p_summaryPlane == nullptr || p_summaryPlane->isBad())
            {
                continue;
            }

            const Plane::GeometrySnapshot geometry =
                p_summaryPlane->getGeometrySnapshot();
            Eigen::Vector4d normalizedEquation = geometry.equation_World;
            const double    normalNorm = normalizedEquation.head<3>().norm();
            if (normalizedEquation.allFinite() && std::isfinite(normalNorm) &&
                normalNorm >= 1e-8)
            {
                normalizedEquation /= normalNorm;
                for (Eigen::Index component = 0;
                     component < normalizedEquation.head<3>().size();
                     ++component)
                {
                    if (std::abs(normalizedEquation(component)) <= 1e-12)
                    {
                        continue;
                    }
                    if (normalizedEquation(component) < 0.0)
                    {
                        normalizedEquation = -normalizedEquation;
                    }
                    break;
                }
            }

            const Plane::ObservationSideSnapshot faceSnapshot =
                p_summaryPlane->getObservationSideSnapshot(normalizedEquation);
            const char *face = "UNKNOWN";
            switch (faceSnapshot.face)
            {
            case Plane::ObservationSideSnapshot::Face::POSITIVE:
                face = "POSITIVE";
                break;
            case Plane::ObservationSideSnapshot::Face::NEGATIVE:
                face = "NEGATIVE";
                break;
            case Plane::ObservationSideSnapshot::Face::AMBIGUOUS:
                face = "AMBIGUOUS";
                break;
            default:
                break;
            }

            const char *planeType = "UNDEFINED";
            switch (p_summaryPlane->getPlaneType())
            {
            case Plane::planeVariant::WALL:
                planeType = "WALL";
                break;
            case Plane::planeVariant::GROUND:
                planeType = "GROUND";
                break;
            case Plane::planeVariant::DOOR:
                planeType = "DOOR";
                break;
            case Plane::planeVariant::WINDOW:
                planeType = "WINDOW";
                break;
            default:
                break;
            }

            std::cout << "[SemMgrSummary] Plane#" << p_summaryPlane->getId()
                      << " mapId=" << getMapId(p_summaryPlane->GetMap())
                      << " type=" << planeType << " equation=("
                      << normalizedEquation(0) << ',' << normalizedEquation(1)
                      << ',' << normalizedEquation(2) << ','
                      << normalizedEquation(3)
                      << ") finiteSupport=" << geometry.finiteSupportCount
                      << " centroid=(" << geometry.centroid_World_m.x() << ','
                      << geometry.centroid_World_m.y() << ','
                      << geometry.centroid_World_m.z() << ") bounds=("
                      << geometry.minPlaneU_m << ',' << geometry.maxPlaneU_m
                      << ',' << geometry.minPlaneV_m << ','
                      << geometry.maxPlaneV_m << ") dimensions=("
                      << geometry.maxPlaneU_m - geometry.minPlaneU_m << ','
                      << geometry.maxPlaneV_m - geometry.minPlaneV_m
                      << ") cloudGeneration=" << geometry.cloudGeneration
                      << " successfulRefitGeneration="
                      << geometry.successfulRefitGeneration << " face=" << face
                      << " faceConsensus=" << faceSnapshot.consensusRatio
                      << " faceEvidence=" << faceSnapshot.evidenceCount
                      << std::endl;
        }

        std::cout << "[SemMgrSummary] --- rooms ---" << std::endl;
        for (ORB_SLAM3::Room *p_summaryRoom : summaryRooms)
        {
            if (p_summaryRoom == nullptr || p_summaryRoom->isBad())
            {
                continue;
            }

            const std::vector<ORB_SLAM3::Plane *> summaryWalls =
                p_summaryRoom->getWalls();
            std::vector<int> summaryWallIds;
            for (ORB_SLAM3::Plane *p_wall : summaryWalls)
            {
                if (p_wall != nullptr && !p_wall->isBad())
                {
                    summaryWallIds.push_back(p_wall->getId());
                }
            }
            const std::vector<ORB_SLAM3::Passage *> summaryPassages =
                p_summaryRoom->getPassages();
            std::vector<int> summaryPassageIds;
            for (ORB_SLAM3::Passage *p_passage : summaryPassages)
            {
                if (p_passage != nullptr)
                {
                    summaryPassageIds.push_back(p_passage->getId());
                }
            }
            ORB_SLAM3::Floor     *p_summaryFloor = p_summaryRoom->getFloor();
            const Eigen::Vector3d summaryCentroid =
                p_summaryRoom->getCentroid().cast<double>();
            const std::string summaryVariant =
                (p_summaryRoom->getRoomVariant() ==
                         ORB_SLAM3::Room::roomVariant::ROOM
                     ? "ROOM"
                     : "UNDEFINED");
            const std::string summaryBoundary =
                (p_summaryRoom->getBoundaryStatus() ==
                         ORB_SLAM3::Room::BoundaryStatus::COMPLETE
                     ? "COMPLETE"
                     : (p_summaryRoom->getBoundaryStatus() ==
                                ORB_SLAM3::Room::BoundaryStatus::INCOMPLETE
                            ? "INCOMPLETE"
                            : "UNOBSERVED"));

            std::cout << "[SemMgrSummary] Room#" << p_summaryRoom->getId()
                      << " variant=" << summaryVariant
                      << " mapId=" << getMapId(p_summaryRoom->getMap())
                      << " floorId="
                      << (p_summaryFloor != nullptr ? p_summaryFloor->getId()
                                                    : -1)
                      << " boundary=" << summaryBoundary
                      << " walls=" << summaryWalls.size()
                      << " wallIds=" << formatSortedIds(summaryWallIds)
                      << " centroid=(" << summaryCentroid.x() << ", "
                      << summaryCentroid.y() << ", " << summaryCentroid.z()
                      << ")" << " passages=" << summaryPassages.size()
                      << " passageIds=" << formatSortedIds(summaryPassageIds)
                      << std::endl;
        }

        std::cout << "[SemMgrSummary] --- candidate/prospective rooms ---"
                  << std::endl;
        std::vector<ORB_SLAM3::Room *> summaryCandidates =
            mpAtlas->GetAllCandidateMapRooms();
        std::sort(
            summaryCandidates.begin(),
            summaryCandidates.end(),
            [&getMapId](ORB_SLAM3::Room *p_first, ORB_SLAM3::Room *p_second)
            {
                if (p_first == nullptr)
                {
                    return false;
                }
                if (p_second == nullptr)
                {
                    return true;
                }
                return std::make_pair(getMapId(p_first->getMap()),
                                      p_first->getId()) <
                       std::make_pair(getMapId(p_second->getMap()),
                                      p_second->getId());
            });
        for (ORB_SLAM3::Room *p_summaryCandidate : summaryCandidates)
        {
            if (p_summaryCandidate == nullptr || p_summaryCandidate->isBad())
            {
                continue;
            }

            const std::vector<ORB_SLAM3::Plane *> candidateWalls =
                p_summaryCandidate->getWalls();
            const Eigen::Vector3d candidateCentroid =
                p_summaryCandidate->getCentroid().cast<double>();

            std::cout << "[SemMgrSummary] CandidateRoom#"
                      << p_summaryCandidate->getId()
                      << " mapId=" << getMapId(p_summaryCandidate->getMap())
                      << " walls=" << candidateWalls.size() << " centroid=("
                      << candidateCentroid.x() << ", " << candidateCentroid.y()
                      << ", " << candidateCentroid.z() << ")" << std::endl;
        }

        std::cout << "[SemMgrSummary] --- floors ---" << std::endl;
        std::vector<ORB_SLAM3::Floor *> summaryFloors = mpAtlas->GetAllFloors();
        std::sort(
            summaryFloors.begin(),
            summaryFloors.end(),
            [&getMapId](ORB_SLAM3::Floor *p_first, ORB_SLAM3::Floor *p_second)
            {
                if (p_first == nullptr)
                {
                    return false;
                }
                if (p_second == nullptr)
                {
                    return true;
                }
                return std::make_pair(getMapId(p_first->getMap()),
                                      p_first->getId()) <
                       std::make_pair(getMapId(p_second->getMap()),
                                      p_second->getId());
            });
        for (ORB_SLAM3::Floor *p_summaryFloor : summaryFloors)
        {
            if (p_summaryFloor == nullptr)
            {
                continue;
            }

            std::vector<int> summaryFloorRoomIds;
            for (ORB_SLAM3::Room *p_room : p_summaryFloor->getRooms())
            {
                if (p_room != nullptr && !p_room->isBad())
                {
                    summaryFloorRoomIds.push_back(p_room->getId());
                }
            }

            std::cout << "[SemMgrSummary] Floor#" << p_summaryFloor->getId()
                      << " mapId=" << getMapId(p_summaryFloor->getMap())
                      << " planeValid="
                      << (p_summaryFloor->hasPlaneIdentity() ? 1 : 0)
                      << " roomIds=" << formatSortedIds(summaryFloorRoomIds)
                      << std::endl;
        }

        std::cout << "[SemMgrSummary] --- passages ---" << std::endl;
        std::vector<ORB_SLAM3::Passage *> summaryPassages =
            mpAtlas->GetAllPassages();
        std::sort(summaryPassages.begin(),
                  summaryPassages.end(),
                  [&getMapId](ORB_SLAM3::Passage *p_first,
                              ORB_SLAM3::Passage *p_second)
                  {
                      if (p_first == nullptr)
                      {
                          return false;
                      }
                      if (p_second == nullptr)
                      {
                          return true;
                      }
                      return std::make_pair(getMapId(p_first->getMap()),
                                            p_first->getId()) <
                             std::make_pair(getMapId(p_second->getMap()),
                                            p_second->getId());
                  });
        for (ORB_SLAM3::Passage *p_summaryPassage : summaryPassages)
        {
            if (p_summaryPassage == nullptr)
            {
                continue;
            }

            const Eigen::Vector3d passageCentroid =
                p_summaryPassage->getCentroid().cast<double>();
            const int prospectiveRoomId =
                (p_summaryPassage->hasProspectiveRoom() &&
                         p_summaryPassage->getProspectiveRoom() != nullptr
                     ? p_summaryPassage->getProspectiveRoom()->getId()
                     : -1);
            const Passage::KnownSideProvenance knownSide =
                p_summaryPassage->getKnownSideProvenance();
            const int knownSideRoomId =
                knownSide.pRoom != nullptr ? knownSide.pRoom->getId() : -1;

            std::cout << "[SemMgrSummary] Passage#" << p_summaryPassage->getId()
                      << " mapId=" << getMapId(p_summaryPassage->getMap())
                      << " supportWalls="
                      << p_summaryPassage->getAssociateWalls().size()
                      << " centroid=(" << passageCentroid.x() << ", "
                      << passageCentroid.y() << ", " << passageCentroid.z()
                      << ")" << " prospectiveRoom=" << prospectiveRoomId
                      << " knownSideRoom=" << knownSideRoomId
                      << " knownSideDirection=("
                      << knownSide.direction_World.x() << ','
                      << knownSide.direction_World.y() << ','
                      << knownSide.direction_World.z() << ')' << " passable="
                      << (p_summaryPassage->isPassable() ? 1 : 0)
                      << " traversed="
                      << (p_summaryPassage->getTraversalEvidence() ? 1 : 0)
                      << " traversalKnownToFar="
                      << p_summaryPassage->getTraversalKnownToFarCount()
                      << " traversalFarToKnown="
                      << p_summaryPassage->getTraversalFarToKnownCount()
                      << " traversalUnknown="
                      << p_summaryPassage->getTraversalUnknownCount()
                      << " bidirectional="
                      << (p_summaryPassage->hasBidirectionalTraversalEvidence()
                              ? 1
                              : 0)
                      << std::endl;
        }
        std::cout << "[SemMgrSummary] === end semantic state ===" << std::endl;
        std::cout << "[SemMgrSummary] SNAPSHOT_END cycle=" << snapshotCycle
                  << std::endl;

        /* Find the time after it took to run the loop */
        const auto end = std::chrono::high_resolution_clock::now();

        /* Calculate the elapsed time */
        const std::chrono::duration<double> elapsed = end - start;

        /* Allow loop closing and segmentation to update the semantic graph. */
        semanticUpdateLock.unlock();

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
}

std::vector<std::vector<Eigen::Vector3d>>
    SemanticsManager::getLatestSkeletonCluster(void)
{
    /* Lock the skeleton cluster */
    unique_lock<std::mutex> lock(mMutexNewRooms);

    /* Get the latest skeleton cluster from Atlas */
    return mpAtlas->GetSkeletoClusterPoints();
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
    float threshY = computeGroundPlaneHeight(groundPlane) -
                    sysParams->sem_seg.max_step_elevation;

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

        /* If planes above inverted y threshold, then reset plane semantics */
        if (computeGroundPlaneHeight(plane) < threshY)
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
    const std::size_t minimumConfirmationCount =
        passageParameters.minimumConfirmationSnapshots;
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

        double      openingRadius     = 0.0;
        std::size_t confirmationCount = 0;
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

                const double measuredHeightSpan_m =
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
                                            skeletonFingerprint});
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

        if (p_matchingEvidence->lastConfirmedSkeletonFingerprint !=
            skeletonFingerprint)
        {
            p_matchingEvidence->confirmationCount++;
            p_matchingEvidence->lastConfirmedSkeletonFingerprint =
                skeletonFingerprint;
        }

        candidate.crossingPoint     = p_matchingEvidence->centroid_World_m;
        candidate.confirmationCount = p_matchingEvidence->confirmationCount;

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

            continue;
        }

        if (hasAmbiguousNearbyPassage)
        {
            continue;
        }

        if (candidate.confirmationCount < minimumConfirmationCount)
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
                /* Report the geometrically verified crossing to the room-state
                 * machine (Phase 1 of WP13). */
                crossingEventPending_ = true;

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
                if (p_reachedRoom != nullptr)
                {
                    std::lock_guard<std::mutex> currentRoomLock(
                        mMutexCurrentRoom);
                    currentRoomId_ = p_reachedRoom->getId();
                }

                const bool addedTraversal =
                    p_passage->addTraversalObservation(traversalDirection,
                                                       p_keyFrame->mnFrameId,
                                                       p_keyFrame->mnId);

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
    if (currentRoomId_ != lastKnownRoomId_)
    {
        lastKnownRoomId_ = currentRoomId_;
        trackingLostPending_ = true;
    }
}

void SemanticsManager::updateRoomTrackerState(void)
{
    /* Consume the per-cycle signals. trackingLostPending_ is set on another
     * thread (System::GetMissionHealthSnapshot -> onTrackingLost), so it is
     * read and cleared under mMutexCurrentRoom. */
    bool crossingPending  = false;
    bool trackingLostPending = false;
    {
        std::lock_guard<std::mutex> currentRoomLock(mMutexCurrentRoom);
        crossingPending     = crossingEventPending_;
        trackingLostPending = trackingLostPending_;
        crossingEventPending_     = false;
        trackingLostPending_      = false;
    }

    const std::chrono::duration<double> elapsed =
        std::chrono::steady_clock::now().time_since_epoch();
    const double now_s = elapsed.count();

    /* Passage crossing evidence. segmentCrossesPassageOpening() already
     * required a passable passage; a detected crossing is therefore direct
     * geometric evidence and carries full traversal confidence until the
     * Phase 4 verifier supplies a calibrated value. */
    TraversalGuardValues crossing;
    crossing.passageDetected = crossingPending;
    crossing.passable        = crossingPending;
    crossing.confidence      = crossingPending ? 1.0 : 0.0;

    /* The verification verdict is an abstract, stubbed result until Phase 4
     * (Section 19.2). Only the very first room confirmation uses a
     * repository-available proxy: a confirmed ROOM in the active map. */
    VerificationVerdict verdict;
    Map *p_activeMap = mpAtlas->GetCurrentMap();
    if (p_activeMap != nullptr &&
        roomTracker_.getState() == RoomTrackingState::UNKNOWN)
    {
        const std::vector<Room *> rooms = p_activeMap->GetAllRooms();
        // TODO(WP13 Phase 4): replace the room-presence proxy below with the
        // plane-gated geometric verification verdict.
        for (Room *p_room : rooms)
        {
            if (p_room != nullptr && !p_room->isBad() &&
                p_room->getRoomVariant() == Room::roomVariant::ROOM)
            {
                verdict.pass     = true;
                verdict.inlierCount = 1U;
                break;
            }
        }
    }

    TrackingStatusInput tracking;
    tracking.lost = trackingLostPending;
    /* TODO(WP13 Phase 2): a new-map-created signal from CreateMapInAtlas is
     * not visible to SemanticsManager; wire it through System once Phase 2
     * persistence lands. newMapCreated stays false here. */

    roomTracker_.step(now_s, crossing, verdict, tracking);
}

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

float SemanticsManager::computeGroundPlaneHeight(Plane *groundPlane)
{
    /* Transform the planeCloud according to the planePose */
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr planeCloud =
        groundPlane->getGeometrySnapshot().supportCloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedCloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud(*planeCloud, *transformedCloud, mPlanePoseMat);

    /* get the median height of the plane */
    std::vector<float> yVals;
    for (const auto &point : transformedCloud->points)
    {
        yVals.push_back(point.y);
    }

    size_t numPoint = yVals.size() / 2;

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
        if (p_passage != nullptr && p_passage->isPassable())
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

    /*! Far-side passage backstop: a wall whose centroid lies beyond ANY
     * passable passage aperture cannot bound the near room, even when that
     * passage is not associated with the near room. Reroute it to the
     * passage's prospective room so the far room's wall evidence accumulates
     * there instead of corrupting the near boundary. This chokepoint covers
     * every admission path, including the second cluster loop and the
     * duplicate merge. When no prospective room exists yet, the wall is left
     * unbound so the orphan pass can claim it for the far room. */
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

    for (Passage *p_passage : allPassages)
    {
        if (p_passage == nullptr)
        {
            continue;
        }

        if (!segmentCrossesPassageOpening(
                p_room_inout->getCentroid(),
                p_candidateWall_in->getCentroid().cast<double>(),
                p_passage,
                farSideGroundNormal_World,
                static_cast<double>(
                    sysParams->room_seg.passagePartition.openingMargin_m),
                static_cast<double>(sysParams->room_seg.passagePartition
                                        .minimumSideDistance_m)))
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
            if (std::find(otherWalls.begin(),
                          otherWalls.end(),
                          p_candidateWall_in) != otherWalls.end())
            {
                ownedByConfirmedRoom = true;
                break;
            }
        }
        if (ownedByConfirmedRoom)
        {
            /* A distinct confirmed room already owns this wall. Leave it on
             * that owner rather than re-binding it to the near room. */
            p_room_inout->removeWall(p_candidateWall_in);
            return false;
        }

        ORB_SLAM3::Room *p_prospective = p_passage->getProspectiveRoom();
        if (p_prospective == nullptr || p_prospective->isBad() ||
            p_prospective == p_room_inout)
        {
            p_room_inout->removeWall(p_candidateWall_in);
            std::cout << "[SemMgr] Far-side Wall#"
                      << p_candidateWall_in->getId() << " at Passage#"
                      << p_passage->getId()
                      << " has no opposite stable room; left unbound."
                      << std::endl;
            return false;
        }

        p_room_inout->removeWall(p_candidateWall_in);
        if (mpAtlas->GetRoomWallPlaneById(p_candidateWall_in->getId()) ==
            nullptr)
        {
            mpAtlas->AddRoomWallPlane(p_candidateWall_in);
        }
        p_prospective->setWalls(p_candidateWall_in);
        std::cout << "[SemMgr] Redirected far-side Wall#"
                  << p_candidateWall_in->getId() << " to prospective Room#"
                  << p_prospective->getId() << "." << std::endl;
        return true;
    }

    if (alreadyPresent)
    {
        return true;
    }

    if (!evaluateWallAdmissionEvidence(p_candidateWall_in, sysParams)
             .admissible)
    {
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

    /* For every plane, extract walls */
    for (ORB_SLAM3::Plane *plane : allPlanes)
    {
        /* Skip bad planes */
        if (plane == nullptr || plane->isBad())
        {
            continue;
        }

        /* Append valid wall planes to list */
        if (evaluateWallAdmissionEvidence(plane, sysParams).admissible)
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

        bool createdRoomCandidate = false;

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
            room = GeoSemHelpers::createBlankRoomCandidate(mpAtlas,
                                                           clusterCentroid);

            if (room == nullptr)
            {
                std::cerr << "[SemMgr] Failed to create room "
                             "candidate for cluster "
                          << clusterId << "." << std::endl;

                continue;
            }

            mpAtlas->AddCandidateMapRoom(room);
            createdRoomCandidate = true;

            std::cout << "[SemMgr] Created room candidate SE#" << room->getId()
                      << " for cluster " << clusterId << "." << std::endl;
        }

        /* The skeleton/free-space centroid is the semantic room centre */
        room->setCentroid(clusterCentroid);

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
         * The semantic room centre is the mean of its wall centroids, where
         * each wall centroid is determined by its fitted plane equation. This
         * guarantees the room marker always sits on the same side of its wall
         * normals as its walls (i.e. inside its own boundary), instead of
         * drifting across a wall because the free-space cluster mean was
         * lopsided. Only fall back to the cluster centroid while the room has
         * no walls yet.
         */
        if (!roomWalls.empty())
        {
            Eigen::Vector3d wallMeanCentroid_World_m = Eigen::Vector3d::Zero();
            std::size_t     validWallCount           = 0U;

            for (ORB_SLAM3::Plane *p_roomWall : roomWalls)
            {
                if (p_roomWall == nullptr || p_roomWall->isBad())
                {
                    continue;
                }

                const Eigen::Vector3d wallCentroid_World_m =
                    p_roomWall->getCentroid().cast<double>();

                if (wallCentroid_World_m.allFinite())
                {
                    wallMeanCentroid_World_m += wallCentroid_World_m;
                    validWallCount++;
                }
            }

            if (validWallCount > 0U)
            {
                const Eigen::Vector3d correctedCentroid_World_m =
                    wallMeanCentroid_World_m /
                    static_cast<double>(validWallCount);

                room->setCentroid(correctedCentroid_World_m);
            }
        }

        /*
         * A disconnected skeleton fragment is not, by itself, a room. When
         * every nearby wall is already uniquely owned by another room, this
         * new candidate has no independent boundary evidence and must not be
         * retained or visualised as a semantic room.
         */
        if (createdRoomCandidate && roomWalls.empty())
        {
            Map *p_currentMap = mpAtlas->GetCurrentMap();

            if (p_currentMap != nullptr)
            {
                p_currentMap->EraseDetectedMapRoom(room);
                p_currentMap->EraseMarkerBasedMapRoom(room);
            }

            room->setBad();

            if (loggedRoomCleanupIds_.insert(room->getId()).second)
            {
                std::cout << "[SemMgr] Retired boundary-less SE#"
                          << room->getId() << " from free-space cluster "
                          << clusterId
                          << "; nearby walls already belong to an established "
                             "room."
                          << std::endl;
            }

            continue;
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
        GeoSemHelpers::createMapFloor(mpAtlas);
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
    std::vector<ORB_SLAM3::Room *> allRooms = mpAtlas->GetAllRooms();

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
        if (!evaluateWallAdmissionEvidence(wall, sysParams).admissible)
        {
            continue;
        }

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
            std::cout << "[SemMgr] Wall " << wall->getId()
                      << " registered as orphan (awaiting passage or room "
                         "association)."
                      << std::endl;
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
    std::unordered_set<int>      mappedWallIds;

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
                            if (p_passage == nullptr)
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
            evaluateWallAdmissionEvidence(p_wall, sysParams);

        if (ownedByLiveRoom || associatedWithPassage || evidence.admissible)
        {
            undefendedWalls_.erase(wallId);
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
            continue;
        }

        const unsigned int retiredAfterCycles = state.unresolvedCycles;

        /* Relationships were checked above under the semantic transaction. */
        const std::map<KeyFrame *, Plane::Observation> wallObservations =
            p_wall->getObservations();

        for (const auto &[p_keyFrame, observation] : wallObservations)
        {
            static_cast<void>(observation);
            if (p_keyFrame != nullptr)
            {
                p_keyFrame->RemoveMapPlane(p_wall);
            }
        }

        for (const auto &[p_keyFrame, observation] : wallObservations)
        {
            static_cast<void>(observation);
            p_wall->eraseObservation(p_keyFrame);
        }

        p_wall->setBad();
        p_currentMap->EraseRoomWallPlane(p_wall);
        p_currentMap->EraseMapPlane(p_wall);
        p_wall->refKeyFrame = nullptr;
        p_wall->SetMap(nullptr);
        undefendedWalls_.erase(wallId);

        if (loggedRetiredWallIds_.insert(wallId).second)
        {
            std::cout << "[SemMgr] Retired undefended Wall#" << wallId
                      << " after " << retiredAfterCycles << " cycles."
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
         * different confirmed owner. */
        for (Room *p_nearOwner : owners)
        {
            for (Passage *p_passage : allPassages)
            {
                if (p_passage == nullptr ||
                    (!p_passage->isPassable() &&
                     !p_passage->getTraversalEvidence()) ||
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
        [](Room *p_room_in, const Room::BoundaryStatus boundaryStatus_in)
    {
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

        std::sort(
            wallSegments.begin(),
            wallSegments.end(),
            [&roomCentroid_Ground_m](const FiniteWallSegment2d &firstSegment,
                                     const FiniteWallSegment2d &secondSegment)
            {
                const Eigen::Vector2d firstMidpoint =
                    0.5 * (firstSegment.start_World_m +
                           firstSegment.end_World_m) -
                    roomCentroid_Ground_m;
                const Eigen::Vector2d secondMidpoint =
                    0.5 * (secondSegment.start_World_m +
                           secondSegment.end_World_m) -
                    roomCentroid_Ground_m;

                return std::atan2(firstMidpoint.y(), firstMidpoint.x()) <
                       std::atan2(secondMidpoint.y(), secondMidpoint.x());
            });

        std::vector<Eigen::Vector2d> boundaryCorners_World_m;
        boundaryCorners_World_m.reserve(wallSegments.size());
        bool hasOpenBoundary = false;

        for (std::size_t wallIndex = 0U; wallIndex < wallSegments.size();
             ++wallIndex)
        {
            const FiniteWallSegment2d &currentWall = wallSegments[wallIndex];
            const FiniteWallSegment2d &nextWall =
                wallSegments[(wallIndex + 1U) % wallSegments.size()];
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
                        topologyParameters.maximumCornerGap_m &&
                    nextCornerGap_m <= topologyParameters.maximumCornerGap_m)
                {
                    boundaryCorners_World_m.push_back(corner_World_m);
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

            if ((nearestEndpointPair->first - nearestEndpointPair->second)
                    .norm() <= topologyParameters.maximumCornerGap_m)
            {
                boundaryCorners_World_m.push_back(
                    0.5 *
                    (nearestEndpointPair->first + nearestEndpointPair->second));
                continue;
            }

            hasOpenBoundary = true;
            break;
        }

        if (hasOpenBoundary ||
            boundaryCorners_World_m.size() != wallSegments.size())
        {
            updateBoundaryStatus(p_room, Room::BoundaryStatus::INCOMPLETE);
            continue;
        }

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
                  << ", hasOpenBoundary="
                  << (hasOpenBoundary ? "true" : "false") << ", selfIntersects="
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
            updateBoundaryStatus(p_room, Room::BoundaryStatus::COMPLETE);
        }
    }
}

void SemanticsManager::recomputeRoomCentroidsFromWalls(void)
{
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
            if (p_wall != nullptr && !p_wall->isBad())
            {
                wallCentroidSum += p_wall->getCentroid().cast<double>();
                wallCount++;
            }
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
        if (p_passage == nullptr)
        {
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
            /* Supporting-wall side evidence is computed from the camera centre
             * of each keyframe that observed that wall. It therefore preserves
             * observation-time provenance when passage confirmation is delayed.
             */
            for (Plane *p_supportingWall : supportingWalls)
            {
                if (p_supportingWall == nullptr || p_supportingWall->isBad())
                {
                    continue;
                }
                const Plane::ObservationSideSnapshot sideSnapshot =
                    p_supportingWall->getObservationSideSnapshot(
                        passageEquation_World);
                if (!sideSnapshot.medianSignedDistance_m.has_value())
                {
                    continue;
                }
                p_passage->setKnownSideDirection(
                    sideSnapshot.medianSignedDistance_m.value() > 0.0
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

            /* Extract the room centroid */
            const Eigen::Vector3d roomCentroid_World_m = p_room->getCentroid();

            /* Determine which side of the passage plane contains the room */
            const double roomSide_m =
                passageEquation_World.head<3>().dot(roomCentroid_World_m) +
                passageEquation_World(3);

            /*!
             * A wall-centred provisional SE does not yet provide enough
             * evidence to form a room-to-passage connection.
             */
            if (std::abs(roomSide_m) <= sideEpsilon_m)
            {
                continue;
            }

            /* Find the distance from the room to the passage */
            const double roomDistance_m =
                (roomCentroid_World_m - passageCentroid_World_m).norm();

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
            std::cout << "[SemMgr] Passage#" << p_passage->getId()
                      << " has 0 associated rooms; camera-side provenance="
                      << (knownSide.hasDirection() ? "known" : "missing") << "."
                      << std::endl;
        }
        else if (associatedRoomCount > 2)
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
         * The depth heuristic (1.7 m) is conservative for typical indoor rooms.
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

                    for (ORB_SLAM3::Room *p_otherRoom : allRooms)
                    {
                        if (p_otherRoom == nullptr || p_otherRoom->isBad() ||
                            p_otherRoom == p_knownRoom ||
                            p_otherRoom->getRoomVariant() ==
                                ORB_SLAM3::Room::roomVariant::UNDEFINED)
                        {
                            continue;
                        }
                        const Eigen::Vector3d otherCentroid =
                            p_otherRoom->getCentroid();
                        const double otherSide =
                            passageNormal.dot(otherCentroid) + passageEq(3);

                        if (otherSide * passageNormal.dot(knownSideDirection) <
                                0.0 &&
                            std::abs(otherSide) > 0.05)
                        {
                            farSideConfirmedRoomExists = true;
                            p_existingFarSideRoom      = p_otherRoom;
                            break;
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
                        constexpr double      estimatedRoomDepth_m = 1.7;
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
                    ORB_SLAM3::Room::roomVariant::UNDEFINED)
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

                Eigen::Vector4d passageEqu_World =
                    p_passage->getGlobalEquation().coeffs();
                const double passNorm = passageEqu_World.head<3>().norm();

                if (std::isfinite(passNorm) && passNorm > 1e-8)
                {
                    passageEqu_World /= passNorm;
                    const Eigen::Vector3d passageNormal =
                        passageEqu_World.head<3>();
                    const Eigen::Vector3d prospectiveCentroid =
                        p_prospectiveRoom->getCentroid();
                    const double passPerspectiveSide =
                        passageNormal.dot(prospectiveCentroid) +
                        passageEqu_World(3);

                    for (ORB_SLAM3::Room *p_otherRoom : allRooms)
                    {
                        if (p_otherRoom == nullptr || p_otherRoom->isBad() ||
                            p_otherRoom == p_prospectiveRoom ||
                            p_otherRoom->getRoomVariant() ==
                                ORB_SLAM3::Room::roomVariant::UNDEFINED)
                        {
                            continue;
                        }
                        const Eigen::Vector3d otherCentroid =
                            p_otherRoom->getCentroid();
                        const double otherSide =
                            passageNormal.dot(otherCentroid) +
                            passageEqu_World(3);

                        if (passPerspectiveSide * otherSide > 0.0)
                        {
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

} // namespace ORB_SLAM3

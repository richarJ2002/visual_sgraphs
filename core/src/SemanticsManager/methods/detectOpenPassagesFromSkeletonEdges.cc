/*!
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
#include <cmath>
#include <limits>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::detectOpenPassagesFromSkeletonEdges(
    const std::vector<vs_graphs::core::geometric::Plane *> &wallPlanes_in)
{
    const types::SystemParams::SemSeg::PassageDetection &passageParameters =
        p_sysParams->semSeg.passageDetection;

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
        skeletonEdges = p_atlas->getSkeletonEdges();

    if (skeletonEdges.empty())
    {
        openPassageEvidence.clear();
        hasSkeletonFingerprint = false;
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

    if (hasSkeletonFingerprint &&
        skeletonFingerprint == lastSkeletonFingerprint)
    {
        return;
    }

    lastSkeletonFingerprint = skeletonFingerprint;
    hasSkeletonFingerprint  = true;

    /* ---------------------------------------------------------------------- *
     * PREPARE THE GROUND PLANE
     * ---------------------------------------------------------------------- */

    vs_graphs::core::geometric::Plane *p_groundPlane =
        p_atlas->getBiggestGroundPlane();

    Eigen::Vector4d groundEquation = Eigen::Vector4d::Zero();

    Eigen::Vector3d groundNormal = Eigen::Vector3d::Zero();

    bool hasValidGroundEquation = false;

    if (p_groundPlane != nullptr && !p_groundPlane->isBad())
    {
        groundEquation = p_groundPlane->getGlobalEquation().coeffs();

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
        openPassageEvidence.clear();
        hasSkeletonFingerprint = false;
        return;
    }

    /* ---------------------------------------------------------------------- *
     * PASSAGE CANDIDATE TYPE
     * ---------------------------------------------------------------------- */

    struct PassageCandidate
    {
        vs_graphs::core::geometric::Plane *p_wall = nullptr;

        Eigen::Vector3d crossingPoint = Eigen::Vector3d::Zero();

        double      openingRadius = 0.0;
        /*! Vertical span of this cycle's crossing cluster, 0 when not
         *  reliably measured (see minimumMeasuredHeightSpan below). Together
         *  with openingRadius, this is the passage size estimate the user
         *  asked for -- previously only door-typed (closed) passages had a
         *  size at all. */
        double      heightSpan_m      = 0.0;
        std::size_t confirmationCount = 0;
        /*! Number of individual skeleton-edge crossings clustered into this
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

    passageCandidates.reserve(wallPlanes_in.size());

    /* ---------------------------------------------------------------------- *
     * FIND CROSSINGS FOR EACH WALL
     * ---------------------------------------------------------------------- */

    for (vs_graphs::core::geometric::Plane *p_wall : wallPlanes_in)
    {
        if (p_wall == nullptr || p_wall->isBad())
        {
            continue;
        }

        const geometric::Plane::GeometrySnapshot wallGeometry =
            p_wall->getGeometrySnapshot();
        const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_wallCloud =
            wallGeometry.supportCloud;

        if (p_wallCloud == nullptr || p_wallCloud->empty())
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
        horizontalWallCoordinates_m.reserve(p_wallCloud->size());

        /* Calculate the finite wall bounds */
        for (const pcl::PointXYZRGBA &point : p_wallCloud->points)
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

            for (const pcl::PointXYZRGBA &wallPointPcl : p_wallCloud->points)
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
            candidate.p_wall        = p_wall;
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

    for (OpenPassageEvidence &evidence : openPassageEvidence)
    {
        evidence.missedUpdateCount++;
    }

    for (PassageCandidate &candidate : passageCandidates)
    {
        OpenPassageEvidence *p_matchingEvidence = nullptr;
        double               nearestEvidenceDistance_m =
            std::numeric_limits<double>::infinity();

        for (OpenPassageEvidence &evidence : openPassageEvidence)
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
                candidate.p_wall->getGlobalEquation().normal();

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
                candidate.p_wall->getGlobalEquation().coeffs();
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
            openPassageEvidence.push_back({candidate.p_wall,
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
        p_matchingEvidence->p_supportingWall  = candidate.p_wall;
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
            candidate.p_wall->getGlobalEquation().coeffs();
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

    openPassageEvidence.erase(
        std::remove_if(
            openPassageEvidence.begin(),
            openPassageEvidence.end(),
            [maximumMissedUpdateCount](const OpenPassageEvidence &evidence)
            {
                return evidence.p_supportingWall == nullptr ||
                       evidence.p_supportingWall->isBad() ||
                       evidence.missedUpdateCount > maximumMissedUpdateCount;
            }),
        openPassageEvidence.end());

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
        if (candidate.p_wall == nullptr || candidate.p_wall->isBad())
        {
            continue;
        }

        Eigen::Vector3d candidateNormal =
            candidate.p_wall->getGlobalEquation().normal();

        if (!candidateNormal.allFinite() || candidateNormal.norm() < 1e-8)
        {
            continue;
        }

        candidateNormal.normalize();

        vs_graphs::core::semantic::Passage *p_matchingPassage         = nullptr;
        bool                                hasAmbiguousNearbyPassage = false;

        Eigen::Vector4d candidateWallEquation =
            candidate.p_wall->getGlobalEquation().coeffs();
        const double candidateWallNormalNorm =
            candidateWallEquation.head<3>().norm();

        if (!candidateWallEquation.allFinite() ||
            candidateWallNormalNorm < 1e-8)
        {
            continue;
        }

        candidateWallEquation /= candidateWallNormalNorm;

        const std::vector<vs_graphs::core::semantic::Passage *>
            existingPassages = p_atlas->getAllPassages();

        for (vs_graphs::core::semantic::Passage *p_existingPassage :
             existingPassages)
        {
            if (p_existingPassage == nullptr)
            {
                continue;
            }

            Eigen::Vector3d existingPassageCentroid{};
            if (p_existingPassage->getCentroid(existingPassageCentroid) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getCentroid cannot fail; continue as before.
            }
            const Eigen::Vector3d existingCentroid =
                existingPassageCentroid.cast<double>();

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

            g2o::Plane3D existingPassageGlobalEquation{};
            if (p_existingPassage->getGlobalEquation(
                    existingPassageGlobalEquation) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getGlobalEquation cannot fail; continue as before.
            }
            Eigen::Vector3d existingNormal =
                existingPassageGlobalEquation.normal();

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
                p_matchingPassage = p_existingPassage;
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
        if (p_matchingPassage != nullptr)
        {
            /*
             * Connected ESDF free space through the wall is stronger evidence
             * than a stale blocked-door classification at the same opening.
             */
            if (p_matchingPassage->setPassable(true) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // setPassable cannot fail; continue as before.
            }
            if (p_matchingPassage->setCentroid(candidate.crossingPoint) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // setCentroid cannot fail; continue as before.
            }

            /*
             * A passage is framed by the wall face that first produced it.
             * The opposite face of the same physical wall is separate evidence
             * (offset by the wall thickness): adding it must NOT rewrite the
             * passage plane, otherwise the passage normal flips between the
             * two faces every cycle and the far-side/prospective decisions
             * flip with it. Only refresh the plane when this face already
             * anchors the passage; otherwise just pair the face.
             */
            std::vector<geometric::Plane *> matchingSupportingWalls{};
            if (p_matchingPassage->getAssociateWalls(matchingSupportingWalls) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getAssociateWalls cannot fail; continue as before.
            }
            const bool isKnownSupportingFace =
                std::find(matchingSupportingWalls.begin(),
                          matchingSupportingWalls.end(),
                          candidate.p_wall) != matchingSupportingWalls.end();

            if (isKnownSupportingFace)
            {
                if (p_matchingPassage->setGlobalEquation(
                        candidate.p_wall->getGlobalEquation()) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // setGlobalEquation cannot fail; continue as before.
                }
            }

            if (p_matchingPassage->addAssociateWall(candidate.p_wall) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // addAssociateWall cannot fail; continue as before.
            }

            /* Open passages previously carried no size estimate at all
             * (only door-typed/blocked passages did) -- diameter from the
             * best-confirmed opening radius, height from the best-confirmed
             * vertical crossing span, floored at a typical-door default
             * while that span is still unmeasured. Never shrinks once a
             * larger estimate has been confirmed (candidate.openingRadius/
             * heightSpan_m already hold the running max -- see the
             * temporal evidence merge above). */
            constexpr double defaultOpenPassageHeight_m = 2.0;
            if (p_matchingPassage->setWidth(2.0 * candidate.openingRadius) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // setWidth cannot fail; continue as before.
            }
            if (p_matchingPassage->setHeight(
                    std::max(candidate.heightSpan_m,
                             defaultOpenPassageHeight_m)) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // setHeight cannot fail; continue as before.
            }

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
        GeoSemHelpers::createMapPassage(p_atlas,
                                        nullptr,
                                        candidate.p_wall,
                                        true,
                                        candidate.crossingPoint);

        /* createMapPassage() returns void; find the passage it just
         * registered (freshly created, so its centroid matches this
         * candidate's crossing point exactly) to size it. Open passages
         * previously carried no size estimate at all -- see the matching
         * branch above for the same estimate's derivation. */
        for (semantic::Passage *p_created : p_atlas->getAllPassages())
        {
            Eigen::Vector3d createdCentroid{};
            if (!(p_created == nullptr) &&
                p_created->getCentroid(createdCentroid) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getCentroid cannot fail; continue as before.
            }
            if (p_created == nullptr ||
                !createdCentroid.isApprox(candidate.crossingPoint, 1e-6))
            {
                continue;
            }
            constexpr double defaultOpenPassageHeight_m = 2.0;
            if (p_created->setWidth(2.0 * candidate.openingRadius) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // setWidth cannot fail; continue as before.
            }
            if (p_created->setHeight(std::max(candidate.heightSpan_m,
                                              defaultOpenPassageHeight_m)) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // setHeight cannot fail; continue as before.
            }
            break;
        }
    }
}

} // namespace core
} // namespace vs_graphs

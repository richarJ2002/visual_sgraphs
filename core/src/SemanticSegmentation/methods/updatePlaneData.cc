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

/*!
 * @file         SemanticSegmentation.cc
 *
 * @brief        Implements segmentation in SemanticSegmentation.h.
 */

#include "SemanticSegmentation.h"

#include "../private_functions.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace vs_graphs
{
namespace core
{

void SemanticSegmentation::updatePlaneData(
    KeyFrame                                             *pKF,
    std::vector<std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
                                      Eigen::Vector4d>>> &clsPlanes)
{
    /* Iterate through each semantic class of planes */
    for (size_t clsId = 0; clsId < clsPlanes.size(); clsId++)
    {
        /* Iterate through each plane in the semantic group */
        for (const auto &planePoint : clsPlanes[clsId])
        {
            /* Get the plane equation of the plane */
            Eigen::Vector4d estimatedPlane = planePoint.second;

            /* Initiate a 3D plane object from the detected plane */
            g2o::Plane3D detectedPlane(estimatedPlane);

            /* Convert the given plane to global coordinates */
            g2o::Plane3D globalEquation = utils::utils::Utils::applyPoseToPlane(
                pKF->getPoseInverse().matrix().cast<double>(),
                detectedPlane);

            /* Extract the point cloud assoicated with the plane */
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud =
                planePoint.first;

            /* Initialize the confidence vector */
            std::vector<double> confidences;

            /* Extract the confidences from each point in the point cloud */
            for (size_t i = 0; i < planeCloud->size(); i++)
            {
                confidences.push_back(
                    static_cast<int>(planeCloud->points[i].a) / 255.0);
            }

            /* Initialize the confidence variable */
            double conf = 0.0;

            /*!
             * Find the average confidence across all the points.
             *
             * @note:       There are two different ways to find the confidences
             *              with a summary below:
             *
             *                  use softmin when dealing with semantic
             *                  confidences double conf =
             *                  utils::utils::Utils::calcSoftMin(confidences);
             *
             *                  use average when dealing with geometric (in this
             *                  case depth) confidences
             */
            if (!confidences.empty())
            {
                conf = std::accumulate(confidences.begin(),
                                       confidences.end(),
                                       0.0) /
                       confidences.size();
            }

            /* Initialize a temporary global point cloud which is empty */
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr globalPlaneCloud(
                new pcl::PointCloud<pcl::PointXYZRGBA>);

            /* Copy the plane cloud to the global point cloud */
            pcl::copyPointCloud(*planeCloud, *globalPlaneCloud);

            /* Transform globalPlaneCloud with the transform of the keyframe */
            pcl::transformPointCloud(
                *globalPlaneCloud,
                *globalPlaneCloud,
                pKF->getPoseInverse().matrix().cast<float>());

            /* Get the semantic type of the observation */
            vs_graphs::core::geometric::Plane::PlaneVariant semanticType =
                utils::utils::Utils::getPlaneTypeFromClassId(clsId);

            /*!
             * Associate the observation using the global plane equation and
             * global point cloud.
             *
             * @note        Performing the complete comparison in the global
             *              frame avoids inconsistencies between plane
             *              equations, centroids and point clouds.
             */
            int matchedPlaneId = utils::utils::Utils::associatePlanes(
                p_atlas->getAllPlanes(),
                globalEquation,
                globalPlaneCloud,
                Eigen::Matrix4d::Identity(),
                semanticType,
                p_sysParams->seg.planeAssociation.ominusThresh,
                -1.0F,
                pKF->getCameraCenter().cast<double>());

            /*!
             * If no mapped plane is associated with current plane
             *
             * TODO:       The cognitive complexity breaches the 3 indentation
             *              rule. Hence, a method/function should be introduced
             *              to help break this section of code down and make it
             *              more readable.
             */
            if (matchedPlaneId == -1)
            {
                /*!
                 * If semantic segmentation is running independetly, determine
                 * whether the observation is sufficiently large to become a
                 * new mapped plane.
                 */
                if (!geoRuns)
                {
                    /*!
                     * Apply an additional geometry check before creating a new
                     * wall.
                     *
                     * @note        Small wall observations may be produced by
                     *              doorframes, furniture edges and segmentation
                     *              noise. Small patches are still permited
                     *              to udpate an existing wall because this
                     *              check is only applied when a matchPlaneId is
                     *              -1.
                     */
                    if (semanticType ==
                        vs_graphs::core::geometric::Plane::PlaneVariant::WALL)
                    {
                        const types::SystemParams::SemSeg::WallCreation
                            &wallCreationParams =
                                p_sysParams->semSeg.wallCreation;

                        WallComponentSupport connectedSupport;

                        if (wallCreationParams.connectivity.enabled)
                        {
                            connectedSupport = findLargestWallComponent(
                                globalPlaneCloud,
                                wallCreationParams.connectivity
                                    .clusterTolerance_m);
                        }
                        else if (globalPlaneCloud != nullptr)
                        {
                            connectedSupport.finitePointCount =
                                globalPlaneCloud->size();
                            connectedSupport.componentRatio = 1.0;
                            connectedSupport.pointIndices.resize(
                                globalPlaneCloud->size());
                            std::iota(connectedSupport.pointIndices.begin(),
                                      connectedSupport.pointIndices.end(),
                                      0);
                        }

                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
                            p_connectedGlobalWallCloud(
                                new pcl::PointCloud<pcl::PointXYZRGBA>);

                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
                            p_connectedCameraWallCloud(
                                new pcl::PointCloud<pcl::PointXYZRGBA>);

                        if (globalPlaneCloud != nullptr &&
                            planeCloud != nullptr)
                        {
                            p_connectedGlobalWallCloud->reserve(
                                connectedSupport.pointIndices.size());
                            p_connectedCameraWallCloud->reserve(
                                connectedSupport.pointIndices.size());

                            for (const int sourcePointIndex :
                                 connectedSupport.pointIndices)
                            {
                                if (sourcePointIndex < 0 ||
                                    static_cast<std::size_t>(
                                        sourcePointIndex) >=
                                        globalPlaneCloud->size() ||
                                    static_cast<std::size_t>(
                                        sourcePointIndex) >= planeCloud->size())
                                {
                                    continue;
                                }

                                p_connectedGlobalWallCloud->push_back(
                                    globalPlaneCloud
                                        ->points[static_cast<std::size_t>(
                                            sourcePointIndex)]);
                                p_connectedCameraWallCloud->push_back(
                                    planeCloud->points[static_cast<std::size_t>(
                                        sourcePointIndex)]);
                            }
                        }

                        /* Compute finite dimensions from connected support. */
                        const std::pair<double, double> wallDimensions =
                            utils::utils::Utils::computePlaneWidthHeight(
                                p_connectedGlobalWallCloud);

                        /* Extract the larger planar dimension */
                        const double majorExtent =
                            std::max(wallDimensions.first,
                                     wallDimensions.second);

                        /* Extract the smaller planar dimension */
                        const double minorExtent =
                            std::min(wallDimensions.first,
                                     wallDimensions.second);

                        /* Compute the approximate observed planar area. */
                        const double observedArea = majorExtent * minorExtent;

                        /*!
                         * Reject unsupported and doorframe-sized wall planes.
                         *
                         * @note        These thresholds apply only to the
                         *              creation of new wall planes. Subsequent
                         *              smaller observations may still update a
                         *              mapped wall.
                         */
                        const bool validConnectivity =
                            !wallCreationParams.connectivity.enabled ||
                            (connectedSupport.pointIndices.size() >=
                                 wallCreationParams.connectivity
                                     .minimumComponentPointCount &&
                             connectedSupport.componentRatio >=
                                 wallCreationParams.connectivity
                                     .minimumComponentRatio);

                        /* Perform all configured new-wall admission checks. */
                        const bool validNewWallGeometry =
                            p_connectedGlobalWallCloud != nullptr &&
                            p_connectedGlobalWallCloud->size() >=
                                wallCreationParams.minimumPointCount &&
                            validConnectivity && std::isfinite(majorExtent) &&
                            std::isfinite(minorExtent) &&
                            std::isfinite(observedArea) &&
                            majorExtent >=
                                wallCreationParams.minimumMajorExtent_m &&
                            minorExtent >=
                                wallCreationParams.minimumMinorExtent_m &&
                            observedArea >= wallCreationParams.minimumArea_m2;

                        /* Reject narrow or small wall fragments */
                        if (!validNewWallGeometry)
                        {
                            std::cout
                                << "[SemSeg] Rejecting new wall "
                                   "candidate: points="
                                << connectedSupport.pointIndices.size() << '/'
                                << connectedSupport.finitePointCount
                                << " connected (ratio "
                                << connectedSupport.componentRatio << ')'
                                << ", dimensions=" << majorExtent << "x"
                                << minorExtent << " m, area=" << observedArea
                                << " m^2." << std::endl;

                            continue;
                        }

                        /* Persist only the validated connected wall support. */
                        globalPlaneCloud = p_connectedGlobalWallCloud;
                        planeCloud       = p_connectedCameraWallCloud;
                    }

                    /* Create a new mapped plane */
                    vs_graphs::core::geometric::Plane *newMapPlane =
                        GeoSemHelpers::createMapPlane(p_atlas,
                                                      pKF,
                                                      detectedPlane,
                                                      planeCloud,
                                                      semanticType,
                                                      conf);

                    /* Confirm that plane creation succeeded */
                    if (newMapPlane == nullptr)
                    {
                        continue;
                    }

                    /* Update the semantic votes of the new plane */
                    updatePlaneSemantics(newMapPlane->getId(), clsId, conf);
                }
            }
            else
            {
                /* Update matched mapped plane with the current observation
                 */
                if (!geoRuns)
                {
                    GeoSemHelpers::updateMapPlane(p_atlas,
                                                  pKF,
                                                  detectedPlane,
                                                  planeCloud,
                                                  matchedPlaneId,
                                                  semanticType,
                                                  conf);
                }
                else
                {
                    /*!
                     * Geometric segmentation already created the plane.
                     * Transform the current observation into the global
                     frame
                     * and append it to the matched mapped plane.
                     */
                    pcl::transformPointCloud(
                        *planeCloud,
                        *planeCloud,
                        pKF->getPoseInverse().matrix().cast<float>());

                    vs_graphs::core::geometric::Plane *matchedPlane =
                        p_atlas->getPlaneById(matchedPlaneId);

                    if (matchedPlane != nullptr && !matchedPlane->isBad() &&
                        !planeCloud->empty())
                    {
                        matchedPlane->setMapClouds(planeCloud);

                        GeoSemHelpers::refitMappedPlaneFromCloud(matchedPlane);
                    }
                }

                /*!
                 * Cast the current semantic observation vote for the matched
                 * plane.
                 */
                updatePlaneSemantics(matchedPlaneId, clsId, conf);
            }
        }
    }

    setFinish();
}

} // namespace core
} // namespace vs_graphs

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
    KeyFrame                                             *p_keyFrame_in,
    std::vector<std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
                                      Eigen::Vector4d>>> &p_clsPlanes_in)
{
    /* Iterate through each semantic class of planes */
    for (size_t clsId = 0; clsId < p_clsPlanes_in.size(); clsId++)
    {
        /* Iterate through each plane in the semantic group */
        for (const auto &p_planePoint : p_clsPlanes_in[clsId])
        {
            /* Get the plane equation of the plane */
            Eigen::Vector4d estimatedPlane = p_planePoint.second;

            /* Initiate a 3D plane object from the detected plane */
            g2o::Plane3D detectedPlane(estimatedPlane);

            /* Convert the given plane to global coordinates */
            g2o::Plane3D globalEquation{};
            if (utils::utils::Utils::applyPoseToPlane(
                    p_keyFrame_in->getPoseInverse().matrix().cast<double>(),
                    detectedPlane,
                    globalEquation) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                // applyPoseToPlane cannot fail; continue as before.
            }

            /* Extract the point cloud assoicated with the plane */
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_planeCloud =
                p_planePoint.first;

            /* Initialize the confidence vector */
            std::vector<double> confidences;

            /* Extract the confidences from each point in the point cloud */
            for (size_t planeCloudIndex = 0;
                 planeCloudIndex < p_planeCloud->size();
                 planeCloudIndex++)
            {
                confidences.push_back(
                    static_cast<int>(p_planeCloud->points[planeCloudIndex].a) /
                    255.0);
            }

            /* Initialize the confidence variable */
            double confidence = 0.0;

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
                confidence = std::accumulate(confidences.begin(),
                                             confidences.end(),
                                             0.0) /
                             confidences.size();
            }

            /* Initialize a temporary global point cloud which is empty */
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_globalPlaneCloud(
                new pcl::PointCloud<pcl::PointXYZRGBA>);

            /* Copy the plane cloud to the global point cloud */
            pcl::copyPointCloud(*p_planeCloud, *p_globalPlaneCloud);

            /* Transform globalPlaneCloud with the transform of the keyframe */
            pcl::transformPointCloud(
                *p_globalPlaneCloud,
                *p_globalPlaneCloud,
                p_keyFrame_in->getPoseInverse().matrix().cast<float>());

            /* Get the semantic type of the observation */
            vs_graphs::core::geometric::Plane::PlaneVariant semanticType{};
            if (utils::utils::Utils::getPlaneTypeFromClassId(clsId,
                                                             semanticType) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                // getPlaneTypeFromClassId cannot fail; continue as before.
            }

            /*!
             * Associate the observation using the global plane equation and
             * global point cloud.
             *
             * @note        Performing the complete comparison in the global
             *              frame avoids inconsistencies between plane
             *              equations, centroids and point clouds.
             */
            int matchedPlaneId{};
            if (utils::utils::Utils::associatePlanes(
                    p_atlas->getAllPlanes(),
                    globalEquation,
                    p_globalPlaneCloud,
                    Eigen::Matrix4d::Identity(),
                    semanticType,
                    p_sysParams->seg.planeAssociation.ominusThresh,
                    matchedPlaneId,
                    -1.0F,
                    p_keyFrame_in->getCameraCenter().cast<double>()) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                // associatePlanes cannot fail; continue as before.
            }

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
                if (!isGeometricSegmentationRunning)
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
                                p_globalPlaneCloud,
                                wallCreationParams.connectivity
                                    .clusterTolerance_m);
                        }
                        else if (p_globalPlaneCloud != nullptr)
                        {
                            connectedSupport.finitePointCount =
                                p_globalPlaneCloud->size();
                            connectedSupport.componentRatio = 1.0;
                            connectedSupport.pointIndices.resize(
                                p_globalPlaneCloud->size());
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

                        if (p_globalPlaneCloud != nullptr &&
                            p_planeCloud != nullptr)
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
                                        p_globalPlaneCloud->size() ||
                                    static_cast<std::size_t>(
                                        sourcePointIndex) >=
                                        p_planeCloud->size())
                                {
                                    continue;
                                }

                                p_connectedGlobalWallCloud->push_back(
                                    p_globalPlaneCloud
                                        ->points[static_cast<std::size_t>(
                                            sourcePointIndex)]);
                                p_connectedCameraWallCloud->push_back(
                                    p_planeCloud
                                        ->points[static_cast<std::size_t>(
                                            sourcePointIndex)]);
                            }
                        }

                        /* Compute finite dimensions from connected support. */
                        std::pair<double, double> wallDimensions{};
                        if (utils::utils::Utils::computePlaneWidthHeight(
                                p_connectedGlobalWallCloud,
                                wallDimensions) !=
                            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
                        {
                            // computePlaneWidthHeight cannot fail; continue as
                            // before.
                        }

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
                        p_globalPlaneCloud = p_connectedGlobalWallCloud;
                        p_planeCloud       = p_connectedCameraWallCloud;
                    }

                    /* Create a new mapped plane */
                    vs_graphs::core::geometric::Plane *p_newMapPlane = nullptr;
                    if (GeoSemHelpers::createMapPlane(p_atlas,
                                                      p_keyFrame_in,
                                                      detectedPlane,
                                                      p_planeCloud,
                                                      p_newMapPlane,
                                                      semanticType,
                                                      confidence) !=
                        GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS)
                    {
                        // createMapPlane cannot fail; continue as before.
                    }

                    /* Confirm that plane creation succeeded */
                    if (p_newMapPlane == nullptr)
                    {
                        continue;
                    }

                    /* Update the semantic votes of the new plane */
                    int newMapPlaneGetId{};
                    if (p_newMapPlane->getId(newMapPlaneGetId) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    updatePlaneSemantics(newMapPlaneGetId, clsId, confidence);
                }
            }
            else
            {
                /* Update matched mapped plane with the current observation
                 */
                if (!isGeometricSegmentationRunning)
                {
                    if (GeoSemHelpers::updateMapPlane(p_atlas,
                                                      p_keyFrame_in,
                                                      detectedPlane,
                                                      p_planeCloud,
                                                      matchedPlaneId,
                                                      semanticType,
                                                      confidence) !=
                        GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS)
                    {
                        // updateMapPlane cannot fail; continue as before.
                    }
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
                        *p_planeCloud,
                        *p_planeCloud,
                        p_keyFrame_in->getPoseInverse().matrix().cast<float>());

                    vs_graphs::core::geometric::Plane *p_matchedPlane =
                        p_atlas->getPlaneById(matchedPlaneId);

                    bool matchedPlaneIsBad{};
                    if ((p_matchedPlane != nullptr) &&
                        p_matchedPlane->isBad(matchedPlaneIsBad) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // isBad cannot fail; continue as before.
                    }
                    if (p_matchedPlane != nullptr && !matchedPlaneIsBad &&
                        !p_planeCloud->empty())
                    {
                        if (p_matchedPlane->setMapClouds(p_planeCloud) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                        {
                            // setMapClouds cannot fail; continue as before.
                        }

                        bool wasPlaneRefit{};
                        if (GeoSemHelpers::refitMappedPlaneFromCloud(
                                p_matchedPlane,
                                wasPlaneRefit) !=
                            GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS)
                        {
                            // refitMappedPlaneFromCloud cannot fail; continue
                            // as before.
                        }
                    }
                }

                /*!
                 * Cast the current semantic observation vote for the matched
                 * plane.
                 */
                updatePlaneSemantics(matchedPlaneId, clsId, confidence);
            }
        }
    }

    setFinish();
}

} // namespace core
} // namespace vs_graphs

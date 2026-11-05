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
 * @file            updatePlaneData.cc
 *
 * @brief           Implements SemanticSegmentation::updatePlaneData(), declared
 *                  in SemanticSegmentation.h.
 */

#include "scene_understanding/semantic_segmentation/objects/SemanticSegmentationClass.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "configuration/types/types.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"

#include "scene_understanding/geo_sem_helpers/geo_sem_helpers.h"
#include "scene_understanding/geo_sem_helpers/objects/GeoSemHelpersStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "mapping/geometry_helpers/utils.h"

#include <numeric>
#include <rclcpp/logging.hpp>

#include "execution/profiling/profiling.h"

namespace vs_graphs
{
namespace core
{
namespace semantic_segmentation
{

SemanticSegmentationStatus SemanticSegmentation::updatePlaneData(
    keyframe::KeyFrame                                             *p_keyFrame_in,
    std::vector<std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
                                      Eigen::Vector4d>>> &p_clsPlanes_in)
{
    PROFILE_FUNCTION();
    /* Iterate through each semantic class of planes */
    for (size_t clsId = 0; clsId < p_clsPlanes_in.size(); clsId++)
    {
        /* Iterate through each plane in the semantic group */
        for (const std::pair<
                 std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>>,
                 Eigen::Vector4d> &p_planePoint : p_clsPlanes_in[clsId])
        {
            /* Get the plane equation of the plane */
            Eigen::Vector4d estimatedPlane = p_planePoint.second;

            /* Initiate a 3D plane object from the detected plane */
            g2o::Plane3D detectedPlane(estimatedPlane);

            /* Convert the given plane to global coordinates */
            g2o::Plane3D globalEquation{};
            Sophus::SE3f keyFramePoseInverse{};
            if (p_keyFrame_in->getPoseInverse(keyFramePoseInverse) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPoseInverse returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (utils::utils::Utils::applyPoseToPlane(
                    keyFramePoseInverse.matrix().cast<double>(),
                    detectedPlane,
                    globalEquation) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: applyPoseToPlane returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
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
                             static_cast<double>(confidences.size());
            }

            /* Initialize a temporary global point cloud which is empty */
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_globalPlaneCloud(
                new pcl::PointCloud<pcl::PointXYZRGBA>);

            /* Copy the plane cloud to the global point cloud */
            pcl::copyPointCloud(*p_planeCloud, *p_globalPlaneCloud);

            /* Transform globalPlaneCloud with the transform of the keyframe */
            Sophus::SE3f keyFramePoseInverse2{};
            if (p_keyFrame_in->getPoseInverse(keyFramePoseInverse2) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPoseInverse returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            pcl::transformPointCloud(
                *p_globalPlaneCloud,
                *p_globalPlaneCloud,
                keyFramePoseInverse2.matrix().cast<float>());

            /* Get the semantic type of the observation */
            vs_graphs::core::geometric::Plane::PlaneVariant semanticType{};
            if (utils::utils::Utils::getPlaneTypeFromClassId(
                    static_cast<int>(clsId),
                    semanticType) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getPlaneTypeFromClassId returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            /*!
             * Associate the observation using the global plane equation and
             * global point cloud.
             *
             * @note            Performing the complete comparison in the global
             *                  frame avoids inconsistencies between plane
             *                  equations, centroids and point clouds.
             */
            int             matchedPlaneId{};
            Eigen::Vector3f keyFrameCameraCenter{};
            if (p_keyFrame_in->getCameraCenter(keyFrameCameraCenter) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCameraCenter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            std::vector<vs_graphs::core::geometric::Plane *> atlasAllPlanes{};
            if (p_atlas->getAllPlanes(atlasAllPlanes) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllPlanes returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            utils::utils::PlaneEvidence observedPlane{};
            observedPlane.planeEquation_world    = globalEquation.coeffs();
            observedPlane.p_supportCloud_world_m = p_globalPlaneCloud;
            observedPlane.planeLabel             = semanticType;
            observedPlane.observerSide =
                Eigen::Vector3d(keyFrameCameraCenter.cast<double>());

            utils::utils::PlaneMatchSettings matchSettings{};
            matchSettings.typeRule =
                utils::utils::PlaneTypeRule::EXPECTED_TYPE_OR_UNDEFINED;
            matchSettings.maximumAngle_rad = static_cast<double>(
                p_sysParams->seg.planeAssociation.ominusThresh);
            matchSettings.maximumPlaneDistance_m = static_cast<double>(
                p_sysParams->seg.planeAssociation.distanceThresh);
            matchSettings.maximumCentroidDistance_m = static_cast<double>(
                p_sysParams->seg.planeAssociation.centroidThresh);
            matchSettings.maximumCloudGap_m = static_cast<double>(
                p_sysParams->seg.planeAssociation.clusterSeparation.tolerance);

            if (utils::utils::Utils::findMatchingPlane(atlasAllPlanes,
                                                       observedPlane,
                                                       matchSettings,
                                                       matchedPlaneId,
                                                       nullptr) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: findMatchingPlane returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            /*!
             * If no mapped plane is associated with current plane, create one.
             * Size and connectivity were already checked by
             * applyPlaneCheckpoint(), for new and matching patches alike.
             */
            if (matchedPlaneId == -1)
            {
                if (!isGeometricSegmentationRunning)
                {
                    /* Create a new mapped plane */
                    vs_graphs::core::geometric::Plane *p_newMapPlane = nullptr;
                    if (utils::geo_sem_helpers::GeoSemHelpers::createMapPlane(p_atlas,
                                                      p_keyFrame_in,
                                                      detectedPlane,
                                                      p_planeCloud,
                                                      p_newMapPlane,
                                                      semanticType,
                                                      confidence) !=
                        utils::geo_sem_helpers::GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: createMapPlane returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                }
            }
            else
            {
                /* Update matched mapped plane with the current observation
                 */
                if (!isGeometricSegmentationRunning)
                {
                    if (utils::geo_sem_helpers::GeoSemHelpers::updateMapPlane(p_atlas,
                                                      p_keyFrame_in,
                                                      detectedPlane,
                                                      p_planeCloud,
                                                      matchedPlaneId,
                                                      semanticType,
                                                      confidence) !=
                        utils::geo_sem_helpers::GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: updateMapPlane returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                }
                else
                {
                    /*!
                     * Geometric segmentation already created the plane.
                     * Transform the current observation into the global
                     * frame
                     * and append it to the matched mapped plane.
                     *
                     * @note            No keyframe observation is recorded
                     *                  here, so this branch adds no label
                     *                  evidence: a plane's label comes only
                     *                  from its keyframe observations.
                     */
                    Sophus::SE3f keyFramePoseInverse3{};
                    if (p_keyFrame_in->getPoseInverse(keyFramePoseInverse3) !=
                        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getPoseInverse returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    pcl::transformPointCloud(
                        *p_planeCloud,
                        *p_planeCloud,
                        keyFramePoseInverse3.matrix().cast<float>());

                    vs_graphs::core::geometric::Plane *p_matchedPlane = nullptr;
                    if (p_atlas->getPlaneById(matchedPlaneId, p_matchedPlane) !=
                        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getPlaneById returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }

                    bool matchedPlaneIsBad{};
                    if ((p_matchedPlane != nullptr) &&
                        p_matchedPlane->isBad(matchedPlaneIsBad) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_matchedPlane != nullptr && !matchedPlaneIsBad &&
                        !p_planeCloud->empty())
                    {
                        if (p_matchedPlane->setMapClouds(p_planeCloud) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: setMapClouds returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }

                        bool wasPlaneRefit{};
                        if (utils::geo_sem_helpers::GeoSemHelpers::refitMappedPlaneFromCloud(
                                p_matchedPlane,
                                wasPlaneRefit) !=
                            utils::geo_sem_helpers::GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS)
                        {
                            /*!
                             * refitMappedPlaneFromCloud cannot fail; continue
                             * as before.
                             */
                        }
                    }
                }
            }
        }
    }

    return SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS;
}

} /* namespace semantic_segmentation */
} /* namespace core */
} /* namespace vs_graphs */

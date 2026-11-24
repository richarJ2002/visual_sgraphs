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
 * @file            propagateSemanticPoseCorrections.cc
 *
 * @brief           Implements
 *                  Utils::propagateSemanticPoseCorrections(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <rclcpp/logging.hpp>
#include <shared_mutex>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus Utils::propagateSemanticPoseCorrections(
    map::Map              *p_map_inout,
    SemanticsManager      *p_semanticsManager_in,
    const KeyFramePoseMap &keyFramePosesBefore_worldToCamera_in,
    const KeyFramePoseMap &keyFramePosesAfter_worldToCamera_in,
    const g2o::Sim3       &fallbackTransform_oldWorldToNewWorld_in)
{
    if (p_map_inout == nullptr)
    {
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }
    unsigned long  mapId{};
    GraphLockScope graphLockScope{};
    /* This edits rooms, passages and storeys in place: exclusive graph lock
     * for the whole pass (re-entrant when the caller already holds it). */
    if ((p_map_inout->getId(mapId) != map::MapStatus::MAP_STATUS_SUCCESS) ||
        (p_semanticsManager_in == nullptr) ||
        (p_semanticsManager_in->lockGraphs(
             {mapId},
             GraphLockMode::GRAPH_LOCK_MODE_EXCLUSIVE,
             graphLockScope) !=
         SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId/lockGraphs returned a failure status "
                     "although they cannot fail; continuing as before.",
                     __func__);
    }

    struct PoseCorrectionNode
    {
        keyframe::KeyFrame *p_keyFrame;
        g2o::Sim3           poseBefore_worldToCamera;
        g2o::Sim3           poseAfter_worldToCamera;
        g2o::Sim3           poseCorrection_oldWorldToNewWorld;
        Eigen::Vector3d     cameraCenter_oldWorld_m;
    };

    const auto isFiniteSim3 = [](const g2o::Sim3 &transform_in)
    {
        return std::isfinite(transform_in.scale()) &&
               std::abs(transform_in.scale()) > 1e-12 &&
               transform_in.translation().allFinite() &&
               transform_in.rotation().coeffs().allFinite();
    };

    std::vector<PoseCorrectionNode> correctionNodes;
    correctionNodes.reserve(keyFramePosesBefore_worldToCamera_in.size());

    for (const auto &[p_keyFrame, poseBefore_worldToCamera] :
         keyFramePosesBefore_worldToCamera_in)
    {
        bool keyFrameIsBad{};
        if (!(p_keyFrame == nullptr) &&
            p_keyFrame->isBad(keyFrameIsBad) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_keyFrame == nullptr || keyFrameIsBad ||
            !isFiniteSim3(poseBefore_worldToCamera))
        {
            continue;
        }

        const KeyFramePoseMap::const_iterator poseAfterIterator =
            keyFramePosesAfter_worldToCamera_in.find(p_keyFrame);

        if (poseAfterIterator == keyFramePosesAfter_worldToCamera_in.end() ||
            !isFiniteSim3(poseAfterIterator->second))
        {
            continue;
        }

        const g2o::Sim3 poseCorrection_oldWorldToNewWorld =
            poseAfterIterator->second.inverse() * poseBefore_worldToCamera;

        const Eigen::Vector3d cameraCenter_oldWorld_m =
            poseBefore_worldToCamera.inverse().map(Eigen::Vector3d::Zero());

        if (!isFiniteSim3(poseCorrection_oldWorldToNewWorld) ||
            !cameraCenter_oldWorld_m.allFinite())
        {
            continue;
        }

        correctionNodes.push_back({p_keyFrame,
                                   poseBefore_worldToCamera,
                                   poseAfterIterator->second,
                                   poseCorrection_oldWorldToNewWorld,
                                   cameraCenter_oldWorld_m});
    }

    std::sort(
        correctionNodes.begin(),
        correctionNodes.end(),
        [](const PoseCorrectionNode &firstNode_in,
           const PoseCorrectionNode &secondNode_in)
        { return firstNode_in.p_keyFrame->id < secondNode_in.p_keyFrame->id; });

    const auto findNodeForKeyFrame =
        [&correctionNodes](const keyframe::KeyFrame *p_keyFrame_in)
        -> const PoseCorrectionNode *
    {
        if (p_keyFrame_in == nullptr)
        {
            return nullptr;
        }

        const std::vector<PoseCorrectionNode>::iterator nodeIterator =
            std::find_if(correctionNodes.begin(),
                         correctionNodes.end(),
                         [p_keyFrame_in](const PoseCorrectionNode &node_in)
                         { return node_in.p_keyFrame == p_keyFrame_in; });

        return nodeIterator == correctionNodes.end() ? nullptr
                                                     : &(*nodeIterator);
    };

    PoseCorrectionField correctionField{};
    correctionField.anchors.reserve(correctionNodes.size());
    for (const PoseCorrectionNode &node : correctionNodes)
    {
        correctionField.anchors.push_back(
            {node.cameraCenter_oldWorld_m,
             node.poseCorrection_oldWorldToNewWorld});
    }
    correctionField.fallbackCorrection_oldWorldToNewWorld =
        fallbackTransform_oldWorldToNewWorld_in;

    std::vector<geometric::Plane *> mapAllPlanes{};
    if (p_map_inout->getAllPlanes(mapAllPlanes) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (geometric::Plane *p_plane : mapAllPlanes)
    {
        bool planeIsBad{};
        if (!(p_plane == nullptr) &&
            p_plane->isBad(planeIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_plane == nullptr || planeIsBad)
        {
            continue;
        }

        Eigen::Vector3d planeCentroid_oldWorld_m{};
        if (p_plane->getCentroid(planeCentroid_oldWorld_m) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        correctionField.planeCentroids_oldWorld_m.insert_or_assign(
            p_plane,
            planeCentroid_oldWorld_m);

        const PoseCorrectionNode *p_selectedNode =
            findNodeForKeyFrame(p_plane->p_refKeyFrame);

        if (p_selectedNode == nullptr)
        {
            double nearestObserverSquaredDistance_m2 =
                std::numeric_limits<double>::infinity();

            std::map<keyframe::KeyFrame *, geometric::Plane::Observation>
                planeGetObservations{};
            if (p_plane->getObservations(planeGetObservations) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getObservations returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (const auto &[p_observingKeyFrame, observation] :
                 planeGetObservations)
            {
                (void)observation;

                const PoseCorrectionNode *p_observerNode =
                    findNodeForKeyFrame(p_observingKeyFrame);

                if (p_observerNode == nullptr)
                {
                    continue;
                }

                const double squaredDistance_m2 =
                    (p_observerNode->cameraCenter_oldWorld_m -
                     planeCentroid_oldWorld_m)
                        .squaredNorm();

                if (squaredDistance_m2 < nearestObserverSquaredDistance_m2)
                {
                    nearestObserverSquaredDistance_m2 = squaredDistance_m2;
                    p_selectedNode                    = p_observerNode;
                }
            }
        }

        const g2o::Sim3 &poseCorrection_oldWorldToNewWorld =
            p_selectedNode != nullptr
                ? p_selectedNode->poseCorrection_oldWorldToNewWorld
                : correctionField.selectCorrectionForPoint(
                      planeCentroid_oldWorld_m);

        if (p_plane->applyTransform(poseCorrection_oldWorldToNewWorld) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: applyTransform returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        correctionField.planeCorrections_oldWorldToNewWorld.insert_or_assign(
            p_plane,
            poseCorrection_oldWorldToNewWorld);
    }

    if ((p_semanticsManager_in == nullptr) ||
        (p_semanticsManager_in->applyPoseCorrectionField(mapId,
                                                         p_map_inout,
                                                         correctionField) !=
         SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: map %lu has no semantics manager or no scene graph in it; "
            "continuing without its passages, rooms and floors.",
            __func__,
            mapId);
    }

    std::vector<std::vector<Eigen::Vector3d>> skeletonClusters_oldWorld_m{};
    if (p_map_inout->getSkeletonClusterPoints(skeletonClusters_oldWorld_m) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSkeletonClusterPoints returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    for (std::vector<Eigen::Vector3d> &skeletonCluster_oldWorld_m :
         skeletonClusters_oldWorld_m)
    {
        if (skeletonCluster_oldWorld_m.empty())
        {
            continue;
        }

        Eigen::Vector3d clusterCentroid_oldWorld_m = Eigen::Vector3d::Zero();
        for (const Eigen::Vector3d &semanticPoint_oldWorld_m :
             skeletonCluster_oldWorld_m)
        {
            clusterCentroid_oldWorld_m += semanticPoint_oldWorld_m;
        }
        clusterCentroid_oldWorld_m /=
            static_cast<double>(skeletonCluster_oldWorld_m.size());

        const g2o::Sim3 &clusterCorrection_oldWorldToNewWorld =
            correctionField.selectCorrectionForPoint(
                clusterCentroid_oldWorld_m);

        for (Eigen::Vector3d &semanticPoint_oldWorld_m :
             skeletonCluster_oldWorld_m)
        {
            semanticPoint_oldWorld_m = clusterCorrection_oldWorldToNewWorld.map(
                semanticPoint_oldWorld_m);
        }
    }

    if (p_map_inout->setSkeletonClusterPoints(skeletonClusters_oldWorld_m) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setSkeletonClusterPoints returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        skeletonEdges_oldWorld_m{};
    if (p_map_inout->getSkeletonEdges(skeletonEdges_oldWorld_m) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSkeletonEdges returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    for (std::pair<Eigen::Vector3d, Eigen::Vector3d> &skeletonEdge_oldWorld_m :
         skeletonEdges_oldWorld_m)
    {
        const Eigen::Vector3d firstEndpoint_oldWorld_m =
            skeletonEdge_oldWorld_m.first;
        const Eigen::Vector3d secondEndpoint_oldWorld_m =
            skeletonEdge_oldWorld_m.second;

        skeletonEdge_oldWorld_m.first =
            correctionField.selectCorrectionForPoint(firstEndpoint_oldWorld_m)
                .map(firstEndpoint_oldWorld_m);
        skeletonEdge_oldWorld_m.second =
            correctionField.selectCorrectionForPoint(secondEndpoint_oldWorld_m)
                .map(secondEndpoint_oldWorld_m);
    }

    if (p_map_inout->setSkeletonEdges(skeletonEdges_oldWorld_m) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setSkeletonEdges returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} /* namespace utils */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */

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
 * @file            reAssociateSemanticPlanes.cc
 *
 * @brief           Implements Utils::reAssociateSemanticPlanes(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"
#include "Utils/Utils/private_functions.h"

#include <algorithm>
#include <iostream>
#include <map>
#include <tuple>

#include "GeoSemHelpers.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

void Utils::reAssociateSemanticPlanes(Atlas *p_atlas_in)
{
    if (p_atlas_in == nullptr)
    {
        return;
    }

    types::SystemParams *p_systemParams = types::SystemParams::getParams();

    bool mergedPlaneInPass = true;

    while (mergedPlaneInPass)
    {
        mergedPlaneInPass = false;

        const std::vector<geometric::Plane *> mappedPlanes =
            p_atlas_in->getAllPlanes();

        for (geometric::Plane *p_candidatePlane : mappedPlanes)
        {
            if (p_candidatePlane == nullptr || p_candidatePlane->isBad() ||
                p_candidatePlane->getPlaneType() ==
                    geometric::Plane::PlaneVariant::UNDEFINED)
            {
                continue;
            }

            std::vector<geometric::Plane *> compatiblePlanes;
            compatiblePlanes.reserve(mappedPlanes.size());

            for (geometric::Plane *p_otherPlane : mappedPlanes)
            {
                if (p_otherPlane == nullptr ||
                    p_otherPlane == p_candidatePlane || p_otherPlane->isBad() ||
                    p_otherPlane->getPlaneType() !=
                        p_candidatePlane->getPlaneType())
                {
                    continue;
                }

                /*
                 * Never merge wall faces observed from opposite sides. Their
                 * equations can be nearly identical when wall thickness is
                 * below the association threshold, but they bound different
                 * rooms and require independent ownership.
                 */
                if (p_candidatePlane->getPlaneType() ==
                    geometric::Plane::PlaneVariant::WALL)
                {
                    const geometric::Plane::GeometrySnapshot candidateGeometry =
                        p_candidatePlane->getGeometrySnapshot();
                    const geometric::Plane::GeometrySnapshot otherGeometry =
                        p_otherPlane->getGeometrySnapshot();
                    Eigen::Vector4d candidateEquation_World =
                        candidateGeometry.equation_World;
                    Eigen::Vector4d otherEquation_World =
                        otherGeometry.equation_World;

                    const double candidateNormalNorm =
                        candidateEquation_World.head<3>().norm();
                    const double otherNormalNorm =
                        otherEquation_World.head<3>().norm();

                    if (candidateEquation_World.allFinite() &&
                        otherEquation_World.allFinite() &&
                        candidateNormalNorm >= 1e-8 && otherNormalNorm >= 1e-8)
                    {
                        candidateEquation_World /= candidateNormalNorm;
                        otherEquation_World /= otherNormalNorm;

                        if (candidateEquation_World.head<3>().dot(
                                otherEquation_World.head<3>()) < 0.0)
                        {
                            otherEquation_World *= -1.0;
                        }

                        const ObservationSideEvidence candidateObservationSide =
                            getMedianObservationSide_World_m(
                                p_candidatePlane,
                                candidateEquation_World);
                        const ObservationSideEvidence otherObservationSide =
                            getMedianObservationSide_World_m(
                                p_otherPlane,
                                otherEquation_World);

                        if (candidateObservationSide.isAmbiguous ||
                            otherObservationSide.isAmbiguous)
                        {
                            continue;
                        }

                        if (candidateObservationSide.medianSignedDistance_m
                                .has_value() &&
                            otherObservationSide.medianSignedDistance_m
                                .has_value() &&
                            candidateObservationSide.medianSignedDistance_m
                                        .value() *
                                    otherObservationSide.medianSignedDistance_m
                                        .value() <
                                0.0)
                        {
                            continue;
                        }

                        /* Finite support is checked once by associatePlanes(),
                         * including nearest-neighbour partial overlap. */
                    }
                }

                compatiblePlanes.push_back(p_otherPlane);
            }

            if (compatiblePlanes.empty())
            {
                continue;
            }

            const bool useWallExtensionDistance =
                p_candidatePlane->getPlaneType() ==
                    geometric::Plane::PlaneVariant::WALL &&
                p_systemParams->semSeg.reassociate.wallExtension.enabled;

            const float maximumFiniteCloudDistance_m =
                useWallExtensionDistance
                    ? p_systemParams->semSeg.reassociate.wallExtension
                          .maximumInPlaneGap_m
                    : -1.0F;

            const geometric::Plane::GeometrySnapshot
                candidateAssociationGeometry =
                    p_candidatePlane->getGeometrySnapshot();
            const int matchedPlaneId = associatePlanes(
                compatiblePlanes,
                g2o::Plane3D(candidateAssociationGeometry.equation_World),
                candidateAssociationGeometry.supportCloud,
                Eigen::Matrix4d::Identity(),
                p_candidatePlane->getPlaneType(),
                p_systemParams->semSeg.reassociate.associationThresh,
                maximumFiniteCloudDistance_m);

            if (matchedPlaneId < 0)
            {
                continue;
            }

            const auto matchedPlaneIterator =
                std::find_if(compatiblePlanes.begin(),
                             compatiblePlanes.end(),
                             [matchedPlaneId](const geometric::Plane *p_plane) {
                                 return p_plane != nullptr &&
                                        p_plane->getId() == matchedPlaneId;
                             });

            if (matchedPlaneIterator == compatiblePlanes.end())
            {
                continue;
            }

            geometric::Plane *p_matchedPlane = *matchedPlaneIterator;

            const geometric::Plane::GeometrySnapshot candidateGeometry =
                p_candidatePlane->getGeometrySnapshot();
            const geometric::Plane::GeometrySnapshot matchedGeometry =
                p_matchedPlane->getGeometrySnapshot();
            const auto candidateEvidence =
                std::make_tuple(p_candidatePlane->getObservationCount(),
                                candidateGeometry.supportCloud != nullptr
                                    ? candidateGeometry.supportCloud->size()
                                    : 0U,
                                -p_candidatePlane->getId());

            const auto matchedEvidence =
                std::make_tuple(p_matchedPlane->getObservationCount(),
                                matchedGeometry.supportCloud != nullptr
                                    ? matchedGeometry.supportCloud->size()
                                    : 0U,
                                -p_matchedPlane->getId());

            geometric::Plane *p_retainedPlane =
                candidateEvidence >= matchedEvidence ? p_candidatePlane
                                                     : p_matchedPlane;

            geometric::Plane *p_retiredPlane =
                p_retainedPlane == p_candidatePlane ? p_matchedPlane
                                                    : p_candidatePlane;

            const geometric::Plane::GeometrySnapshot retiredGeometry =
                p_retiredPlane->getGeometrySnapshot();
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_retiredCloudCopy(
                new pcl::PointCloud<pcl::PointXYZRGBA>);
            if (retiredGeometry.supportCloud != nullptr)
            {
                *p_retiredCloudCopy = *retiredGeometry.supportCloud;
                p_retainedPlane->setMapClouds(p_retiredCloudCopy);
            }

            for (MapPoint *p_mapPoint : p_retiredPlane->getMapPoints())
            {
                if (p_mapPoint != nullptr && !p_mapPoint->isBad())
                {
                    p_retainedPlane->setMapPoints(p_mapPoint);
                }
            }

            const std::map<KeyFrame *, geometric::Plane::Observation>
                retiredObservations = p_retiredPlane->getObservations();

            for (const auto &[p_keyFrame, observation] : retiredObservations)
            {
                if (p_keyFrame == nullptr || p_keyFrame->isBad())
                {
                    continue;
                }

                p_retainedPlane->mergeObservation(p_keyFrame, observation);
            }

            GeoSemHelpers::refitMappedPlaneFromCloud(p_retainedPlane);
            const geometric::Plane::PlaneVariant retainedPlaneType =
                p_retainedPlane->getPlaneType();

            for (semantic::Room *p_room : p_atlas_in->getAllRooms())
            {
                if (p_room == nullptr || p_room->isBad())
                {
                    continue;
                }

                p_room->replaceWall(p_retiredPlane, p_retainedPlane);
                p_room->replaceGroundPlane(p_retiredPlane, p_retainedPlane);
            }

            for (vs_graphs::core::semantic::Passage *p_passage :
                 p_atlas_in->getAllPassages())
            {
                if (p_passage != nullptr)
                {
                    p_passage->replacePlaneAssociation(p_retiredPlane,
                                                       p_retainedPlane);
                }
            }

            for (KeyFrame *p_keyFrame : p_atlas_in->getAllKeyFrames())
            {
                if (p_keyFrame != nullptr && !p_keyFrame->isBad())
                {
                    p_keyFrame->replaceMapPlane(p_retiredPlane,
                                                p_retainedPlane);
                }
            }

            Map *p_currentMap = p_atlas_in->getCurrentMap();

            if (p_currentMap != nullptr)
            {
                p_currentMap->eraseRoomWallPlane(p_retiredPlane);

                if (retainedPlaneType == geometric::Plane::PlaneVariant::WALL)
                {
                    p_currentMap->addRoomWallPlane(p_retainedPlane);
                }
            }

            /*
             * Invalidate only after every graph edge points at the survivor,
             * then remove the retired hypothesis from the map container and
             * ID index so later merge passes cannot rediscover stale state.
             */
            p_retiredPlane->setBad();

            if (p_currentMap != nullptr)
            {
                p_currentMap->eraseMapPlane(p_retiredPlane);
            }

            p_retiredPlane->setMap(nullptr);

            std::cout << "[SemanticMerge] Fused geometric::Plane#"
                      << p_retiredPlane->getId() << " into geometric::Plane#"
                      << p_retainedPlane->getId() << '.' << std::endl;

            mergedPlaneInPass = true;
            break;
        }
    }
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs

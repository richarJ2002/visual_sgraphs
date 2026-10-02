/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            resurfaceProxyFromTransferred.cc
 *
 * @brief           Implements resurfaceProxyFromTransferred(), declared in
 *                  Atlas/private_functions.h.
 */

#include "Atlas.h"

#include "private_functions.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief        Folds a transferred passage into the same-lineage proxy.
 *
 *               The proxy keeps its stable current-map ID and live room
 *               links and adopts the transferred (already in-frame)
 *               geometry, supporting walls, door, known-side direction
 *               and traversal history. Traversal windows are disjoint
 *               (pre- vs post-reset), so counts add. Returns true when
 *               the transferred object must NOT enter the current map
 *               (it retires with the absorbed map instead).
 */
AtlasStatus resurfaceProxyFromTransferred(semantic::Passage *p_proxy_inout,
                                          semantic::Passage *p_transferred_in,
                                          bool              &wasResurfaced_out)
{
    bool proxy_inoutIsRecoveryProxy{};
    if (!(p_proxy_inout == nullptr || p_transferred_in == nullptr ||
          p_proxy_inout == p_transferred_in) &&
        p_proxy_inout->isRecoveryProxy(proxy_inoutIsRecoveryProxy) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isRecoveryProxy returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    bool proxy_inoutIsBad{};
    if (!(p_proxy_inout == nullptr || p_transferred_in == nullptr ||
          p_proxy_inout == p_transferred_in || !proxy_inoutIsRecoveryProxy) &&
        p_proxy_inout->isBad(proxy_inoutIsBad) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    bool transferred_inIsBad{};
    if (!(p_proxy_inout == nullptr || p_transferred_in == nullptr ||
          p_proxy_inout == p_transferred_in || !proxy_inoutIsRecoveryProxy ||
          proxy_inoutIsBad) &&
        p_transferred_in->isBad(transferred_inIsBad) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_proxy_inout == nullptr || p_transferred_in == nullptr ||
        p_proxy_inout == p_transferred_in || !proxy_inoutIsRecoveryProxy ||
        proxy_inoutIsBad || transferred_inIsBad)
    {
        wasResurfaced_out = false;
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }

    Eigen::Vector3d transferredCentroid{};
    if (p_transferred_in->getCentroid(transferredCentroid) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    g2o::Plane3D transferred_inGlobalEquation{};
    if (p_transferred_in->getGlobalEquation(transferred_inGlobalEquation) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const Eigen::Vector4d transferredCoefficients =
        transferred_inGlobalEquation.coeffs();
    const double transferredNormalNorm =
        transferredCoefficients.head<3>().norm();
    double transferredWidth_m{};
    if (p_transferred_in->getWidth(transferredWidth_m) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWidth returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    double transferredHeight_m{};
    if (p_transferred_in->getHeight(transferredHeight_m) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getHeight returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (transferredCentroid.allFinite() &&
        transferredCoefficients.allFinite() && transferredNormalNorm > 1e-8 &&
        std::isfinite(transferredWidth_m) && transferredWidth_m > 0.0 &&
        std::isfinite(transferredHeight_m) && transferredHeight_m > 0.0)
    {
        if (p_proxy_inout->setCentroid(transferredCentroid) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        g2o::Plane3D transferred_inGlobalEquation2{};
        if (p_transferred_in->getGlobalEquation(
                transferred_inGlobalEquation2) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_proxy_inout->setGlobalEquation(transferred_inGlobalEquation2) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_proxy_inout->setWidth(transferredWidth_m) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setWidth returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_proxy_inout->setHeight(transferredHeight_m) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setHeight returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_proxy_inout->setRecoveryProxy(false) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setRecoveryProxy returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
    std::vector<vs_graphs::core::geometric::Plane *>
        transferred_inAssociateWalls{};
    if (p_transferred_in->getAssociateWalls(transferred_inAssociateWalls) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAssociateWalls returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    for (geometric::Plane *p_wall : transferred_inAssociateWalls)
    {
        if (p_wall != nullptr)
        {
            if (p_proxy_inout->addAssociateWall(p_wall) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addAssociateWall returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }
    vs_graphs::core::geometric::Plane *p_proxy_inoutAssociateDoor = nullptr;
    if (p_proxy_inout->getAssociateDoor(p_proxy_inoutAssociateDoor) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAssociateDoor returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    vs_graphs::core::geometric::Plane *p_transferred_inAssociateDoor = nullptr;
    if ((p_proxy_inoutAssociateDoor == nullptr) &&
        p_transferred_in->getAssociateDoor(p_transferred_inAssociateDoor) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAssociateDoor returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_proxy_inoutAssociateDoor == nullptr &&
        p_transferred_inAssociateDoor != nullptr)
    {
        vs_graphs::core::geometric::Plane *p_transferred_inAssociateDoor2 =
            nullptr;
        if (p_transferred_in->getAssociateDoor(
                p_transferred_inAssociateDoor2) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAssociateDoor returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_proxy_inout->setAssociateDoor(p_transferred_inAssociateDoor2) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setAssociateDoor returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
    semantic::Passage::KnownSideProvenance transferredSide{};
    if (p_transferred_in->getKnownSideProvenance(transferredSide) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getKnownSideProvenance returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    semantic::Passage::KnownSideProvenance proxy_inoutKnownSideProvenance{};
    if (p_proxy_inout->getKnownSideProvenance(proxy_inoutKnownSideProvenance) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getKnownSideProvenance returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    bool proxy_inoutKnownSideProvenanceHasDirection{};
    if (proxy_inoutKnownSideProvenance.hasDirection(
            proxy_inoutKnownSideProvenanceHasDirection) !=
        semantic::KnownSideProvenanceStatus::
            KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasDirection returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool transferredSideHasDirection{};
    if ((!proxy_inoutKnownSideProvenanceHasDirection) &&
        transferredSide.hasDirection(transferredSideHasDirection) !=
            semantic::KnownSideProvenanceStatus::
                KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasDirection returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (!proxy_inoutKnownSideProvenanceHasDirection &&
        transferredSideHasDirection)
    {
        if (p_proxy_inout->setKnownSideDirection(
                transferredSide.knownSideDirection_world) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                        "%s: setKnownSideDirection rejected its input; "
                        "continuing as before.",
                        __func__);
        }
    }
    std::size_t transferredKnownToFarCount{};
    if (p_transferred_in->getTraversalKnownToFarCount(
            transferredKnownToFarCount) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getTraversalKnownToFarCount returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    for (std::size_t observationIndex = 0U;
         observationIndex < transferredKnownToFarCount;
         ++observationIndex)
    {
        if (p_proxy_inout->addTraversalObservation(
                semantic::Passage::TraversalDirection::KNOWN_TO_FAR) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: addTraversalObservation returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }
    std::size_t transferredFarToKnownCount{};
    if (p_transferred_in->getTraversalFarToKnownCount(
            transferredFarToKnownCount) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getTraversalFarToKnownCount returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    for (std::size_t observationIndex = 0U;
         observationIndex < transferredFarToKnownCount;
         ++observationIndex)
    {
        if (p_proxy_inout->addTraversalObservation(
                semantic::Passage::TraversalDirection::FAR_TO_KNOWN) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: addTraversalObservation returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }
    std::size_t transferredUnknownCount{};
    if (p_transferred_in->getTraversalUnknownCount(transferredUnknownCount) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getTraversalUnknownCount returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    for (std::size_t observationIndex = 0U;
         observationIndex < transferredUnknownCount;
         ++observationIndex)
    {
        if (p_proxy_inout->addTraversalObservation(
                semantic::Passage::TraversalDirection::UNKNOWN) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: addTraversalObservation returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }
    bool transferred_inIsPassable{};
    if (p_transferred_in->isPassable(transferred_inIsPassable) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isPassable returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (transferred_inIsPassable)
    {
        if (p_proxy_inout->setPassable(true) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPassable returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
    }
    vs_graphs::core::Map *p_proxy_inoutMap = nullptr;
    if (p_proxy_inout->getMap(p_proxy_inoutMap) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    int proxy_inoutId{};
    if (p_proxy_inout->getId(proxy_inoutId) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    unsigned long proxy_inoutMapId{};
    if (p_proxy_inoutMap->getId(proxy_inoutMapId) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    std::cout << "SG_PIPELINE {\"event\":\"passage_resurfaced\","
                 "\"map_id\":"
              << proxy_inoutMapId << ",\"passage_id\":" << proxy_inoutId << "}"
              << std::endl;
    wasResurfaced_out = true;
    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

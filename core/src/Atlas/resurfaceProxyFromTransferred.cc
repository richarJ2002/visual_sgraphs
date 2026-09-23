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

#include "Atlas.h"

#include "private_functions.h"

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
bool resurfaceProxyFromTransferred(semantic::Passage *p_proxy_inout,
                                   semantic::Passage *p_transferred_in)
{
    if (p_proxy_inout == nullptr || p_transferred_in == nullptr ||
        p_proxy_inout == p_transferred_in ||
        !p_proxy_inout->isRecoveryProxy() || p_proxy_inout->isBad() ||
        p_transferred_in->isBad())
    {
        return false;
    }

    const Eigen::Vector3d transferredCentroid = p_transferred_in->getCentroid();
    const Eigen::Vector4d transferredCoefficients =
        p_transferred_in->getGlobalEquation().coeffs();
    const double transferredNormalNorm =
        transferredCoefficients.head<3>().norm();
    const double transferredWidth_m  = p_transferred_in->getWidth();
    const double transferredHeight_m = p_transferred_in->getHeight();
    if (transferredCentroid.allFinite() &&
        transferredCoefficients.allFinite() && transferredNormalNorm > 1e-8 &&
        std::isfinite(transferredWidth_m) && transferredWidth_m > 0.0 &&
        std::isfinite(transferredHeight_m) && transferredHeight_m > 0.0)
    {
        p_proxy_inout->setCentroid(transferredCentroid);
        p_proxy_inout->setGlobalEquation(p_transferred_in->getGlobalEquation());
        p_proxy_inout->setWidth(transferredWidth_m);
        p_proxy_inout->setHeight(transferredHeight_m);
        p_proxy_inout->setRecoveryProxy(false);
    }
    for (geometric::Plane *p_wall : p_transferred_in->getAssociateWalls())
    {
        if (p_wall != nullptr)
        {
            p_proxy_inout->addAssociateWall(p_wall);
        }
    }
    if (p_proxy_inout->getAssociateDoor() == nullptr &&
        p_transferred_in->getAssociateDoor() != nullptr)
    {
        p_proxy_inout->setAssociateDoor(p_transferred_in->getAssociateDoor());
    }
    const semantic::Passage::KnownSideProvenance transferredSide =
        p_transferred_in->getKnownSideProvenance();
    if (!p_proxy_inout->getKnownSideProvenance().hasDirection() &&
        transferredSide.hasDirection())
    {
        p_proxy_inout->setKnownSideDirection(transferredSide.direction_World);
    }
    for (std::size_t observationIndex = 0U;
         observationIndex < p_transferred_in->getTraversalKnownToFarCount();
         ++observationIndex)
    {
        p_proxy_inout->addTraversalObservation(
            semantic::Passage::TraversalDirection::KNOWN_TO_FAR);
    }
    for (std::size_t observationIndex = 0U;
         observationIndex < p_transferred_in->getTraversalFarToKnownCount();
         ++observationIndex)
    {
        p_proxy_inout->addTraversalObservation(
            semantic::Passage::TraversalDirection::FAR_TO_KNOWN);
    }
    for (std::size_t observationIndex = 0U;
         observationIndex < p_transferred_in->getTraversalUnknownCount();
         ++observationIndex)
    {
        p_proxy_inout->addTraversalObservation(
            semantic::Passage::TraversalDirection::UNKNOWN);
    }
    if (p_transferred_in->isPassable())
    {
        p_proxy_inout->setPassable(true);
    }
    std::cout << "SG_PIPELINE {\"event\":\"passage_resurfaced\","
                 "\"map_id\":"
              << p_proxy_inout->getMap()->getId()
              << ",\"passage_id\":" << p_proxy_inout->getId() << "}"
              << std::endl;
    return true;
}

} // namespace core
} // namespace vs_graphs

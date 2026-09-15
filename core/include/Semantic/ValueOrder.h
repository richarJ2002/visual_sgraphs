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

/*!
 * @file            ValueOrder.h
 *
 * @brief           Declares total-order-safe comparison primitives shared by
 *                   every module that must sort semantic value types
 *                   deterministically (SemanticGraphSnapshot and
 *                   SemanticCanonicalSerialization). This is a clean public
 *                   boundary rather than either module's private header, so
 *                   that no module needs to include a sibling module's
 *                   private_functions.h merely to agree on one canonical
 *                   order (CPP_CODING_STANDARD.md Section 5.4).
 */

#ifndef SEMANTIC_VALUE_ORDER_H
#define SEMANTIC_VALUE_ORDER_H

#include <cstdint>

#include <Eigen/Core>

#include "Semantic/SemanticGraphSnapshot/objects/EntityRef.h"
#include "Semantic/SemanticGraphSnapshot/objects/RawPlaneRef.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*! @brief Maps a finite-or-not-finite double to a std::uint64_t key whose
 *  ordinary unsigned integer order matches the IEEE 754 `totalOrder`
 *  predicate: -0.0 orders immediately before +0.0, every negative value
 *  before every non-negative value, and NaN payloads collate consistently
 *  (all negative NaNs before all finite/infinite values before all positive
 *  NaNs) rather than participating in the undefined/unordered comparisons
 *  ordinary `<` gives for NaN. Never uses std::hash, wall-clock data, or
 *  pointer addresses. */
std::uint64_t doubleTotalOrderKey(double value_in);

/*! @brief Strict weak "less than" for two doubles that remains a valid total
 *  order even when either value is NaN, infinite, or signed zero; see
 *  doubleTotalOrderKey(). */
bool isDoubleLess(double lhs_in, double rhs_in);

/*! @brief Componentwise isDoubleLess() over (x, y, z), most-significant
 *  component first. */
bool isVector3dLess(const Eigen::Vector3d &lhs_in,
                    const Eigen::Vector3d &rhs_in);

/*! @brief Componentwise isDoubleLess() over (x, y, z, w) (Eigen's raw
 *  coefficient order for a 4-vector), most-significant component first. */
bool isVector4dLess(const Eigen::Vector4d &lhs_in,
                    const Eigen::Vector4d &rhs_in);

/*! @brief Strict weak "less than" comparing every EntityRef field in a fixed
 *  canonical order: key presence, then key value; reason; localId presence,
 *  then value; isLive presence, then value; livenessUnavailableReason. */
bool isEntityRefLess(const EntityRef &lhs_in, const EntityRef &rhs_in);

/*! @brief Same as isEntityRefLess(), for RawPlaneRef: canonical order is
 *  mapId presence/value, planeId, isLive, planeType, reason, wallKey
 *  presence/value. */
bool isRawPlaneRefLess(const RawPlaneRef &lhs_in, const RawPlaneRef &rhs_in);

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_VALUE_ORDER_H

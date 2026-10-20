/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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
 * @file            VerificationStatusEnum.h
 *
 * @brief           Declares VerificationStatus, the abstract
 *                  verification-result event.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_VERIFICATION_STATUS_ENUM_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_VERIFICATION_STATUS_ENUM_H

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*!
 * @brief           Abstract verification-result event.
 *
 *                  Production remains UNAVAILABLE until a future typed
 *                  geometric-verifier producer supplies a result. Tests
 *                  may inject deterministic values. Only the verdict
 *                  and confidence are consumed by the transition
 *                  engine.
 */
enum class VerificationStatus
{
    UNAVAILABLE,
    PASS,
    REJECTED
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_VERIFICATION_STATUS_ENUM_H \
        */

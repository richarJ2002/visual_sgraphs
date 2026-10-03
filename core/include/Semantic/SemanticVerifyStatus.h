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
 * @file            SemanticVerifyStatus.h
 *
 * @brief           Declares the status returned by every SemanticVerify
 *                  operation.
 */

#ifndef SEMANTIC_VERIFY_STATUS_H
#define SEMANTIC_VERIFY_STATUS_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*!
 * @brief           Result of a SemanticVerify operation. Values are fixed and
 *                  never reordered.
 */
enum class SemanticVerifyStatus : std::uint8_t
{
    /*!
     * @brief           The operation completed and every output was written.
     */
    SEMANTIC_VERIFY_STATUS_SUCCESS = 0U,

    /*!
     * @brief           An input was null, repeated or not finite; the object
     *                  and the outputs were left unchanged.
     */
    SEMANTIC_VERIFY_STATUS_INVALID_ARGUMENT = 1U,

    /*!
     * @brief           A numerical step did not converge or produced a
     *                  non-finite value.
     */
    SEMANTIC_VERIFY_STATUS_NUMERICAL_FAILURE = 2U
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_VERIFY_STATUS_H

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
 * @file            doubleTotalOrderKey.cc
 *
 * @brief           Implements doubleTotalOrderKey(), declared in
 *                  Semantic/ValueOrder.h.
 */

#include "Semantic/ValueOrder.h"

#include <cstdint>
#include <cstring>

namespace ORB_SLAM3
{
namespace semantic
{

std::uint64_t doubleTotalOrderKey(double value_in)
{
    /* Reinterpret the IEEE 754 bit pattern as an unsigned integer, then
     * remap it so ordinary unsigned comparison matches the IEEE 754
     * `totalOrder` predicate. std::memcpy, not a union or
     * reinterpret_cast<uint64_t &>, avoids strict-aliasing undefined
     * behaviour.
     *
     * IEEE 754 double bit layout, most significant bit first: 1 sign bit,
     * 11 exponent bits, 52 mantissa bits. For a non-negative value (sign
     * bit 0), that bit pattern already sorts the same way as the value
     * itself when compared as an unsigned integer (larger exponent/mantissa
     * -> larger magnitude -> larger unsigned value), including +0.0
     * (all-zero bits, smallest non-negative key) and +infinity/positive NaN
     * (maximal exponent, sorting after every finite positive value). For a
     * negative value (sign bit 1), the *raw* bit pattern sorts backwards
     * (a larger-magnitude negative number has a *larger* raw unsigned
     * value, but must receive a *smaller* key), and -0.0's raw bits (only
     * the sign bit set) must sort immediately below +0.0's raw bits (all
     * zero), not far above them.
     *
     * The standard fix: if the sign bit is set, flip every bit (mask =
     * all-ones); otherwise, flip only the sign bit (mask = 0x8000...).
     * Negative values then land, in reverse raw order, entirely below
     * 0x8000000000000000, and non-negative values land, in their original
     * relative order, entirely at or above it -- so -0.0 (raw
     * 0x8000000000000000, mask all-ones -> key 0x7FFFFFFFFFFFFFFF) sorts
     * immediately below +0.0 (raw 0x0, mask 0x8000... -> key
     * 0x8000000000000000), and every negative NaN sorts below every finite
     * value, which sorts below every positive NaN -- a single consistent,
     * total, and reproducible order with no pointer address, wall-clock
     * value, unordered iteration, or std::hash involved. */
    std::uint64_t bits = 0U;
    std::memcpy(&bits, &value_in, sizeof(bits));

    const std::uint64_t signBitSet = bits >> 63U;
    const std::uint64_t mask =
        (signBitSet != 0U) ? 0xFFFFFFFFFFFFFFFFULL : 0x8000000000000000ULL;

    return bits ^ mask;
}

} // namespace semantic
} // namespace ORB_SLAM3

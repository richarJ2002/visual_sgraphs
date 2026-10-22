/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
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
 * Derived from the DUtils library (DLib) file, in the copy distributed
 * with DBoW2 in ORB-SLAM2,
 * core/Thirdparty/DBoW2/DUtils/Timestamp.h
 * (enum Timestamp::tOptions); its notice follows, then the LICENSE.txt it names
 * (core/Thirdparty/DBoW2/LICENSE.txt).
 *
 * File: Timestamp.h
 * Author: Dorian Galvez-Lopez
 * Date: March 2009
 * Description: timestamping functions
 * License: see the LICENSE.txt file
 *
 * DBoW2: bag-of-words library for C++ with generic descriptors
 *
 * Copyright (c) 2015 Dorian Galvez-Lopez <http://doriangalvez.com>
 * (Universidad de Zaragoza)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL COPYRIGHT HOLDERS OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * @file            TimestampOptionEnum.h
 *
 * @brief           Declares the initial-time options of a time stamp.
 */

#ifndef VS_GRAPHS_CORE_SRC_D_UTILS_TIMESTAMP_OBJECTS_TIMESTAMP_OPTION_ENUM_H
#define VS_GRAPHS_CORE_SRC_D_UTILS_TIMESTAMP_OBJECTS_TIMESTAMP_OPTION_ENUM_H

namespace vs_graphs::core::d_utils::timestamp
{

/*!
 * @brief           What a new Timestamp is set to.
 *
 *                  The values are bit flags; the Timestamp constructor tests
 *                  TIMESTAMP_OPTION_CURRENT_TIME before
 *                  TIMESTAMP_OPTION_ZERO. The underlying type is the one GCC
 *                  gave the upstream unscoped enumeration.
 */
enum class TimestampOption : unsigned int
{
    /*!
     * @brief           Leaves both time parts uninitialised.
     */
    TIMESTAMP_OPTION_NONE = 0U,

    /*!
     * @brief           Sets the time stamp to the current wall-clock time.
     */
    TIMESTAMP_OPTION_CURRENT_TIME = 0x1U,

    /*!
     * @brief           Sets both time parts to zero.
     */
    TIMESTAMP_OPTION_ZERO = 0x2U
};

} /* namespace vs_graphs::core::d_utils::timestamp */

#endif /* VS_GRAPHS_CORE_SRC_D_UTILS_TIMESTAMP_OBJECTS_TIMESTAMP_OPTION_ENUM_H \
        */

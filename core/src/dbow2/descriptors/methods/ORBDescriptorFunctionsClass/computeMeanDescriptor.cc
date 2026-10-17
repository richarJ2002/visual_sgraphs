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
 * Derived from DBoW2 as distributed with ORB-SLAM2; original notices:
 */
/*!
 * File: FORB.cpp
 * Date: June 2012
 * Author: Dorian Galvez-Lopez
 * Description: functions for ORB descriptors
 * License: see the LICENSE.txt file
 *
 */

/*!
 * DBoW2: bag-of-words library for C++ with generic descriptors
 *
 * Copyright (c) 2015 Dorian Galvez-Lopez <http://doriangalvez.com> (Universidad
 * de Zaragoza) All rights reserved.
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
 * @file            computeMeanDescriptor.cc
 *
 * @brief           Implements computeMeanDescriptor, declared in
 *                  ORBDescriptorFunctionsClass.h.
 */

#include "dbow2/descriptors/objects/ORBDescriptorFunctionsClass.h"

#include <cstddef>
#include <vector>

namespace vs_graphs::core::dbow2::descriptors
{

void ORBDescriptorFunctions::computeMeanDescriptor(
    const std::vector<DescriptorPointer> &descriptorPointers_in,
    Descriptor                           &meanDescriptor_inout)
{
    if (descriptorPointers_in.empty())
    {
        meanDescriptor_inout.release();
        return;
    }
    else if (descriptorPointers_in.size() == 1)
    {
        meanDescriptor_inout = descriptorPointers_in[0]->clone();
    }
    else
    {
        std::vector<int> bitCounts(
            ORBDescriptorFunctions::DESCRIPTOR_LENGTH_BYTES * 8,
            0);

        for (std::size_t descriptorIndex = 0;
             descriptorIndex < descriptorPointers_in.size();
             descriptorIndex++)
        {
            const cv::Mat &currentDescriptor =
                *descriptorPointers_in[descriptorIndex];
            const unsigned char *p_descriptorByte =
                currentDescriptor.ptr<unsigned char>();

            for (int byteIndex = 0; byteIndex < currentDescriptor.cols;
                 byteIndex++, p_descriptorByte++)
            {
                if (*p_descriptorByte & (1 << 7))
                {
                    bitCounts[byteIndex * 8]++;
                }
                if (*p_descriptorByte & (1 << 6))
                {
                    bitCounts[byteIndex * 8 + 1]++;
                }
                if (*p_descriptorByte & (1 << 5))
                {
                    bitCounts[byteIndex * 8 + 2]++;
                }
                if (*p_descriptorByte & (1 << 4))
                {
                    bitCounts[byteIndex * 8 + 3]++;
                }
                if (*p_descriptorByte & (1 << 3))
                {
                    bitCounts[byteIndex * 8 + 4]++;
                }
                if (*p_descriptorByte & (1 << 2))
                {
                    bitCounts[byteIndex * 8 + 5]++;
                }
                if (*p_descriptorByte & (1 << 1))
                {
                    bitCounts[byteIndex * 8 + 6]++;
                }
                if (*p_descriptorByte & (1))
                {
                    bitCounts[byteIndex * 8 + 7]++;
                }
            }
        }

        /* Preserve the upstream write into any buffer shared by this header. */
        meanDescriptor_inout =
            cv::Mat::zeros(1,
                           ORBDescriptorFunctions::DESCRIPTOR_LENGTH_BYTES,
                           CV_8U);
        unsigned char *p_descriptorByte =
            meanDescriptor_inout.ptr<unsigned char>();

        const int MAJORITY_COUNT =
            static_cast<int>(descriptorPointers_in.size()) / 2 +
            descriptorPointers_in.size() % 2;
        for (std::size_t bitIndex = 0; bitIndex < bitCounts.size(); bitIndex++)
        {
            if (bitCounts[bitIndex] >= MAJORITY_COUNT)
            {
                *p_descriptorByte |= 1 << (7 - (bitIndex % 8));
            }

            if (bitIndex % 8 == 7)
            {
                p_descriptorByte++;
            }
        }
    }
}

} /* namespace vs_graphs::core::dbow2::descriptors */

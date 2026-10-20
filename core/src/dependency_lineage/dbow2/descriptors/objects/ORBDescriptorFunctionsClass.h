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
 * File: FORB.h
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
 * @file            ORBDescriptorFunctionsClass.h
 *
 * @brief           Declares ORBDescriptorFunctionsClass.
 */

#ifndef VS_GRAPHS_CORE_SRC_DBOW2_DESCRIPTORS_OBJECTS_ORB_DESCRIPTOR_FUNCTIONS_CLASS_H
#define VS_GRAPHS_CORE_SRC_DBOW2_DESCRIPTORS_OBJECTS_ORB_DESCRIPTOR_FUNCTIONS_CLASS_H

#include <array>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

namespace vs_graphs::core::dbow2::descriptors
{

/*!
 * @brief           ORB descriptor operations over shared OpenCV matrices.
 *
 * @note            Descriptors are caller-owned 1x32 CV_8U matrices; no
 *                  synchronization is provided.
 */
class ORBDescriptorFunctions
{
  public:
    /*!
     * @brief           OpenCV header sharing its caller-owned byte storage.
     */
    using Descriptor = cv::Mat;
    /*!
     * @brief           Borrowed non-null descriptor address; owner must outlive
     *                  its use.
     */
    using DescriptorPointer = const Descriptor *;
    /*!
     * @brief           Frozen vocabulary descriptor width in bytes.
     */
    static constexpr int DESCRIPTOR_LENGTH_BYTES = 32;
    /*!
     * @brief           Computes the bitwise majority descriptor, retaining
     *                  buffer aliasing.
     *
     * @param[in]       descriptorPointers_in
     *                  Borrowed descriptor addresses.
     *
     * @param[in,out]   meanDescriptor_inout
     *                  Shared matrix buffer to update.
     *
     * @note            Shared cv::Mat storage is reused exactly as upstream; no
     *                  input validation is added.
     */
    static void          computeMeanDescriptor(
                 const std::vector<DescriptorPointer> &descriptorPointers_in,
                 Descriptor                           &meanDescriptor_inout);
    /*!
     * @brief           Computes the 256-bit Hamming distance.
     *
     * @param[in]       leftHandSideDescriptor_in
     *                  First 32-byte descriptor.
     *
     * @param[in]       rightHandSideDescriptor_in
     *                  Second 32-byte descriptor.
     *
     * @return          Number of differing descriptor bits.
     */
    static int computeDistance(const Descriptor &leftHandSideDescriptor_in,
                               const Descriptor &rightHandSideDescriptor_in);
    /*!
     * @brief           Formats bytes as integers with the original trailing
     *                  spaces.
     *
     * @param[in]       orbDescriptor_in
     *                  32-byte descriptor.
     *
     * @return          Space-separated integer bytes.
     */
    static std::string toString(const Descriptor &orbDescriptor_in);
    /*!
     * @brief           Parses bytes, retaining old bytes where parsing fails.
     *
     * @param[in,out]   orbDescriptor_inout
     *                  Existing shared descriptor storage.
     *
     * @param[in]       descriptorText_in
     *                  Space-separated integer bytes.
     *
     * @note            Shared cv::Mat storage is reused exactly as upstream; no
     *                  input validation is added.
     */
    static void        fromString(Descriptor        &orbDescriptor_inout,
                                  const std::string &descriptorText_in);
    /*!
     * @brief           Writes descriptors as float bits with unchanged buffer
     *                  reuse.
     *
     * @param[in]       orbDescriptors_in
     *                  Input descriptor sequence.
     *
     * @param[in,out]   bitMatrix_inout
     *                  Reusable CV_32F output buffer.
     *
     * @note            Shared cv::Mat storage is reused exactly as upstream; no
     *                  input validation is added.
     */
    static void toMat32F(const std::vector<Descriptor> &orbDescriptors_in,
                         cv::Mat                       &bitMatrix_inout);
    /*!
     * @brief           Writes 32 bytes per descriptor into the reusable matrix.
     *
     * @param[in]       orbDescriptors_in
     *                  Input descriptor sequence.
     *
     * @param[in,out]   byteMatrix_inout
     *                  Reusable CV_8U output buffer.
     *
     * @note            Shared cv::Mat storage is reused exactly as upstream; no
     *                  input validation is added.
     */
    static void toMat8U(const std::vector<Descriptor> &orbDescriptors_in,
                        cv::Mat                       &byteMatrix_inout);
    /*!
     * @brief           Returns the 32 descriptor bytes in their original order.
     *
     * @param[in]       orbDescriptor_in
     *                  32-byte descriptor.
     *
     * @return          The descriptor's 32 bytes.
     */
    [[nodiscard]] static std::array<unsigned char, DESCRIPTOR_LENGTH_BYTES>
                toArray8U(const Descriptor &orbDescriptor_in);
    /*!
     * @brief           Copies 32 bytes into the existing descriptor buffer.
     *
     * @param[in,out]   orbDescriptor_inout
     *                  Existing shared descriptor storage.
     *
     * @param[in]       p_byteArray_in
     *                  Borrowed address of at least 32 readable bytes.
     *
     * @note            Shared cv::Mat storage is reused exactly as upstream; no
     *                  input validation is added.
     */
    static void fromArray8U(Descriptor          &orbDescriptor_inout,
                            const unsigned char *p_byteArray_in);
};

static_assert(ORBDescriptorFunctions::DESCRIPTOR_LENGTH_BYTES == 32,
              "ORB descriptor width is part of the vocabulary format");

} /* namespace vs_graphs::core::dbow2::descriptors */

#endif /* VS_GRAPHS_CORE_SRC_DBOW2_DESCRIPTORS_OBJECTS_ORB_DESCRIPTOR_FUNCTIONS_CLASS_H \
        */

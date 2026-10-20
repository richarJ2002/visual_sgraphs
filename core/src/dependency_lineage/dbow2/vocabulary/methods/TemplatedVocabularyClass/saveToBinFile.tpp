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
 * This is a modified version of TemplatedVocabulary.h from DBoW2 (see below).
 * Added functions: Save and Load from text files without using cv::FileStorage.
 * Date: August 2015
 * Raúl Mur-Artal
 */

/*!
 * File: TemplatedVocabulary.h
 * Date: February 2011
 * Author: Dorian Galvez-Lopez
 * Description: templated vocabulary
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
 * @file            saveToBinFile.tpp
 *
 * @brief           Implements saveToBinFile, declared in
 *                  TemplatedVocabularyClass.h.
 */

namespace vs_graphs::core::dbow2::vocabulary
{

template <class DescriptorType, class DescriptorFunctionsType>
void TemplatedVocabulary<DescriptorType, DescriptorFunctionsType>::
    saveToBinFile(const std::string &fileName_in) const
{
    std::fstream outputFile;
    outputFile.open(fileName_in.c_str(), std::ios_base::out);
    outputFile.write(reinterpret_cast<const char *>(&branchingFactor),
                     sizeof(branchingFactor));
    outputFile.write(reinterpret_cast<const char *>(&depthLevels),
                     sizeof(depthLevels));
    outputFile.write(reinterpret_cast<const char *>(&scoringType),
                     sizeof(scoringType));
    outputFile.write(reinterpret_cast<const char *>(&weightingType),
                     sizeof(weightingType));
    unsigned int  nodeCount = treeNodes.size();
    /* nbiter=2; //for test */
    unsigned char leafByte   = 1;
    unsigned char branchByte = 0;
    /* unsigned char tab[10]={1,2,4,6,10,100,200,201,202,255}; f.write((const */
    /* char*)tab,10); //for test */
    for (std::size_t nodeIndex = 1; nodeIndex < nodeCount - 1; nodeIndex++)
    {
        const VocabularyNode<DescriptorType> &treeNode = treeNodes[nodeIndex];
        outputFile.write(reinterpret_cast<const char *>(&treeNode.parentNodeId),
                         sizeof(treeNode.parentNodeId));
        if (treeNode.isLeaf())
        {
            outputFile.write(reinterpret_cast<const char *>(&leafByte),
                             sizeof(leafByte));
        }
        else
        {
            outputFile.write(reinterpret_cast<const char *>(&branchByte),
                             sizeof(branchByte));
        }
        const std::array<unsigned char,
                         DescriptorFunctionsType::DESCRIPTOR_LENGTH_BYTES>
            descriptorBytes = DescriptorFunctionsType::toArray8U(
                treeNode.nodeDescriptor); /* the number of elements is stored in
                                             F::L */

        outputFile.write(reinterpret_cast<const char *>(descriptorBytes.data()),
                         static_cast<std::streamsize>(
                             DescriptorFunctionsType::DESCRIPTOR_LENGTH_BYTES));
        outputFile.write(reinterpret_cast<const char *>(&treeNode.wordWeight),
                         sizeof(treeNode.wordWeight));
    }
    outputFile.close();
}

} /* namespace vs_graphs::core::dbow2::vocabulary */

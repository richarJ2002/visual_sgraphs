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
 * @file            load.tpp
 *
 * @brief           Implements load, declared in TemplatedVocabularyClass.h.
 */

namespace vs_graphs::core::dbow2::vocabulary
{

template <class DescriptorType, class DescriptorFunctionsType>
void TemplatedVocabulary<DescriptorType, DescriptorFunctionsType>::load(
    const std::string &fileName_in)
{
    cv::FileStorage fileStorage(fileName_in.c_str(), cv::FileStorage::READ);
    if (!fileStorage.isOpened())
    {
        throw std::string("Could not open file ") + fileName_in;
    }

    this->load(fileStorage);
}

template <class DescriptorType, class DescriptorFunctionsType>
void TemplatedVocabulary<DescriptorType, DescriptorFunctionsType>::load(
    const cv::FileStorage &fileStorage_in,
    const std::string     &nodeName_in)
{
    wordNodes.clear();
    treeNodes.clear();

    cv::FileNode vocabularyFileNode = fileStorage_in[nodeName_in];

    branchingFactor = static_cast<int>(vocabularyFileNode["k"]);
    depthLevels     = static_cast<int>(vocabularyFileNode["L"]);
    scoringType     = static_cast<ScoringType>(
        (static_cast<int>(vocabularyFileNode["scoringType"])));
    weightingType = static_cast<WeightingType>(
        (static_cast<int>(vocabularyFileNode["weightingType"])));

    createScoringObject();

    /* nodes */
    cv::FileNode fileNodes = vocabularyFileNode["nodes"];

    treeNodes.resize(fileNodes.size() + 1); /* +1 to include root */
    treeNodes[0].nodeId = 0;

    for (unsigned int fileNodeIndex = 0; fileNodeIndex < fileNodes.size();
         fileNodeIndex++)
    {
        feature_vector::NodeId nodeId =
            static_cast<int>(fileNodes[fileNodeIndex]["nodeId"]);
        feature_vector::NodeId parentNodeId =
            static_cast<int>(fileNodes[fileNodeIndex]["parentId"]);
        bow_vector::WordValue wordWeight = static_cast<bow_vector::WordValue>(
            fileNodes[fileNodeIndex]["weight"]);
        std::string descriptorText =
            static_cast<std::string>(fileNodes[fileNodeIndex]["descriptor"]);

        treeNodes[nodeId].nodeId       = nodeId;
        treeNodes[nodeId].parentNodeId = parentNodeId;
        treeNodes[nodeId].wordWeight   = wordWeight;
        treeNodes[parentNodeId].childNodeIds.push_back(nodeId);

        DescriptorFunctionsType::fromString(treeNodes[nodeId].nodeDescriptor,
                                            descriptorText);
    }

    /* words */
    fileNodes = vocabularyFileNode["words"];

    wordNodes.resize(fileNodes.size());

    for (unsigned int fileNodeIndex = 0; fileNodeIndex < fileNodes.size();
         fileNodeIndex++)
    {
        feature_vector::NodeId wordId =
            static_cast<int>(fileNodes[fileNodeIndex]["wordId"]);
        feature_vector::NodeId nodeId =
            static_cast<int>(fileNodes[fileNodeIndex]["nodeId"]);

        treeNodes[nodeId].wordId = wordId;
        wordNodes[wordId]        = &treeNodes[nodeId];
    }
}

} /* namespace vs_graphs::core::dbow2::vocabulary */

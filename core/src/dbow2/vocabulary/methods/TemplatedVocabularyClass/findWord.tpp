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
 * @file            findWord.tpp
 *
 * @brief           Implements findWord, declared in
 *                  TemplatedVocabularyClass.h.
 */

namespace vs_graphs::core::dbow2::vocabulary
{

template <class DescriptorType, class DescriptorFunctionsType>
void TemplatedVocabulary<DescriptorType, DescriptorFunctionsType>::findWord(
    const DescriptorType   &featureDescriptor_in,
    bow_vector::WordId     &wordId_out,
    bow_vector::WordValue  &wordWeight_out,
    feature_vector::NodeId *p_nodeId_out,
    int                     levelsUp_in) const
{
    /* propagate the feature down the tree */
    std::vector<feature_vector::NodeId> candidateNodeIds;
    typename std::vector<feature_vector::NodeId>::const_iterator nodeIterator;

    /* level at which the node must be stored in nid, if given */
    const int NODE_LEVEL = depthLevels - levelsUp_in;
    if (NODE_LEVEL <= 0 && p_nodeId_out != nullptr)
    {
        *p_nodeId_out = 0;
    } /* root */

    feature_vector::NodeId finalNodeId  = 0; /* root */
    int                    currentLevel = 0;

    do
    {
        currentLevel++;
        candidateNodeIds = treeNodes[finalNodeId].childNodeIds;
        finalNodeId      = candidateNodeIds[0];

        double bestDistance = DescriptorFunctionsType::computeDistance(
            featureDescriptor_in,
            treeNodes[finalNodeId].nodeDescriptor);

        for (nodeIterator = candidateNodeIds.begin() + 1;
             nodeIterator != candidateNodeIds.end();
             nodeIterator++)
        {
            feature_vector::NodeId candidateNodeId = *nodeIterator;
            double candidateDistance = DescriptorFunctionsType::computeDistance(
                featureDescriptor_in,
                treeNodes[candidateNodeId].nodeDescriptor);
            if (candidateDistance < bestDistance)
            {
                bestDistance = candidateDistance;
                finalNodeId  = candidateNodeId;
            }
        }

        if (p_nodeId_out != nullptr && currentLevel == NODE_LEVEL)
        {
            *p_nodeId_out = finalNodeId;
        }
    }
    while (!treeNodes[finalNodeId].isLeaf());

    /* turn node id into word id */
    wordId_out     = treeNodes[finalNodeId].wordId;
    wordWeight_out = treeNodes[finalNodeId].wordWeight;
}

} /* namespace vs_graphs::core::dbow2::vocabulary */

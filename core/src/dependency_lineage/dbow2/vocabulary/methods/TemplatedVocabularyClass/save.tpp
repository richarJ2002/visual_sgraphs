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
 * @file            save.tpp
 *
 * @brief           Implements save, declared in TemplatedVocabularyClass.h.
 */

namespace vs_graphs::core::dbow2::vocabulary
{

template <class DescriptorType, class DescriptorFunctionsType>
void TemplatedVocabulary<DescriptorType, DescriptorFunctionsType>::save(
    const std::string &fileName_in) const
{
    cv::FileStorage fileStorage(fileName_in.c_str(), cv::FileStorage::WRITE);
    if (!fileStorage.isOpened())
    {
        throw std::string("Could not open file ") + fileName_in;
    }

    save(fileStorage);
}

template <class DescriptorType, class DescriptorFunctionsType>
void TemplatedVocabulary<DescriptorType, DescriptorFunctionsType>::save(
    cv::FileStorage   &fileStorage_inout,
    const std::string &nodeName_in) const
{
    /* Format YAML: */
    /* vocabulary */
    /* { */
    /* k: */
    /* L: */
    /* scoringType: */
    /* weightingType: */
    /* nodes */
    /* [ */
    /* { */
    /* nodeId: */
    /* parentId: */
    /* weight: */
    /* descriptor: */
    /* } */
    /* ] */
    /* words */
    /* [ */
    /* { */
    /* wordId: */
    /* nodeId: */
    /* } */
    /* ] */
    /* } */
    /*  */
    /* The root node (index 0) is not included in the node vector */
    /*  */

    fileStorage_inout << nodeName_in << "{";

    fileStorage_inout << "k" << branchingFactor;
    fileStorage_inout << "L" << depthLevels;
    fileStorage_inout << "scoringType" << static_cast<int>(scoringType);
    fileStorage_inout << "weightingType" << static_cast<int>(weightingType);

    /* tree */
    fileStorage_inout << "nodes" << "[";
    std::vector<feature_vector::NodeId> parentNodeIds, childNodeIds;
    std::vector<feature_vector::NodeId>::const_iterator childIterator;

    parentNodeIds.push_back(0); /* root */

    while (!parentNodeIds.empty())
    {
        feature_vector::NodeId parentNodeId = parentNodeIds.back();
        parentNodeIds.pop_back();

        const VocabularyNode<DescriptorType> &parentNode =
            treeNodes[parentNodeId];
        childNodeIds = parentNode.childNodeIds;

        for (childIterator = childNodeIds.begin();
             childIterator != childNodeIds.end();
             childIterator++)
        {
            const VocabularyNode<DescriptorType> &childNode =
                treeNodes[*childIterator];

            /* save node data */
            fileStorage_inout << "{:";
            fileStorage_inout << "nodeId" << static_cast<int>(childNode.nodeId);
            fileStorage_inout << "parentId" << static_cast<int>(parentNodeId);
            fileStorage_inout << "weight"
                              << static_cast<double>(childNode.wordWeight);
            fileStorage_inout
                << "descriptor"
                << DescriptorFunctionsType::toString(childNode.nodeDescriptor);
            fileStorage_inout << "}";

            /* add to parent list */
            if (!childNode.isLeaf())
            {
                parentNodeIds.push_back(*childIterator);
            }
        }
    }

    fileStorage_inout << "]"; /* nodes */

    /* words */
    fileStorage_inout << "words" << "[";

    typename std::vector<VocabularyNode<DescriptorType> *>::const_iterator
        wordIterator;
    for (wordIterator = wordNodes.begin(); wordIterator != wordNodes.end();
         wordIterator++)
    {
        bow_vector::WordId wordId = wordIterator - wordNodes.begin();
        fileStorage_inout << "{:";
        fileStorage_inout << "wordId" << static_cast<int>(wordId);
        fileStorage_inout << "nodeId"
                          << static_cast<int>((*wordIterator)->nodeId);
        fileStorage_inout << "}";
    }

    fileStorage_inout << "]"; /* words */

    fileStorage_inout << "}";
}

} /* namespace vs_graphs::core::dbow2::vocabulary */

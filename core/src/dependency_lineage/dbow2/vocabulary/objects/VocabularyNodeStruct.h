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
 * @file            VocabularyNodeStruct.h
 *
 * @brief           Declares VocabularyNodeStruct.
 */

#ifndef VS_GRAPHS_CORE_SRC_DBOW2_VOCABULARY_OBJECTS_VOCABULARY_NODE_STRUCT_H
#define VS_GRAPHS_CORE_SRC_DBOW2_VOCABULARY_OBJECTS_VOCABULARY_NODE_STRUCT_H

#include <vector>

#include "dependency_lineage/dbow2/bow_vector/bow_vector.h"
#include "dependency_lineage/dbow2/feature_vector/feature_vector.h"

namespace vs_graphs::core::dbow2::vocabulary
{

/*!
 * @brief           Vocabulary tree node with unchanged field order and
 *                  shared descriptors.
 *
 * @tparam          DescriptorType
 *                  Descriptor value/header type with upstream copy semantics.
 *
 * @note            DescriptorType is the caller's descriptor header/value
 *                  type; CV matrices share storage as upstream.
 */
template <class DescriptorType> struct VocabularyNode
{
  public:
    /*!
     * @brief           Node index in the owning tree.
     */
    feature_vector::NodeId              nodeId;
    /*!
     * @brief           Weight used when this node is a word.
     */
    bow_vector::WordValue               wordWeight;
    /*!
     * @brief           Owned ordered child identifiers.
     */
    std::vector<feature_vector::NodeId> childNodeIds;
    /*!
     * @brief           Parent index; the root has no meaningful parent.
     */
    feature_vector::NodeId              parentNodeId;
    /*!
     * @brief           Descriptor header/value, retaining its original storage
     *                  semantics.
     */
    DescriptorType                      nodeDescriptor;
    /*!
     * @brief           Word index when this node is a leaf.
     */
    bow_vector::WordId                  wordId;
    /*!
     * @brief           Creates the root/default node.
     */
    VocabularyNode() :
        nodeId(0),
        wordWeight(0),
        parentNodeId(0),
        wordId(0)
    {}
    /*!
     * @brief           Creates a node with its tree identifier.
     *
     * @param[in]       nodeId_in
     *                  Index in the tree.
     */
    VocabularyNode(feature_vector::NodeId nodeId_in) :
        nodeId(nodeId_in),
        wordWeight(0),
        parentNodeId(0),
        wordId(0)
    {}
    /*!
     * @brief           Reports whether this node has no children.
     *
     * @return          Whether the child identifier sequence is empty.
     */
    inline bool isLeaf() const;
};

} /* namespace vs_graphs::core::dbow2::vocabulary */

#include "dependency_lineage/dbow2/vocabulary/methods/VocabularyNodeStruct/isLeaf.tpp"

#endif /* VS_GRAPHS_CORE_SRC_DBOW2_VOCABULARY_OBJECTS_VOCABULARY_NODE_STRUCT_H \
        */

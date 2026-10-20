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
 * @file            TemplatedVocabularyClass.h
 *
 * @brief           Declares TemplatedVocabularyClass.
 */

#ifndef VS_GRAPHS_CORE_SRC_DBOW2_VOCABULARY_OBJECTS_TEMPLATED_VOCABULARY_CLASS_H
#define VS_GRAPHS_CORE_SRC_DBOW2_VOCABULARY_OBJECTS_TEMPLATED_VOCABULARY_CLASS_H

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include "dependency_lineage/d_utils/random/random.h"
#include "dependency_lineage/dbow2/bow_vector/bow_vector.h"
#include "dependency_lineage/dbow2/feature_vector/feature_vector.h"
#include "dependency_lineage/dbow2/scoring/scoring.h"
#include "dependency_lineage/dbow2/vocabulary/objects/ScoringTypeEnum.h"
#include "dependency_lineage/dbow2/vocabulary/objects/VocabularyNodeStruct.h"
#include "dependency_lineage/dbow2/vocabulary/objects/WeightingTypeEnum.h"

namespace vs_graphs::core::dbow2::vocabulary
{

/*!
 * @brief           Hierarchical visual-word vocabulary retaining the
 *                  complete upstream API.
 *
 * @tparam          DescriptorType
 *                  Descriptor value/header type copied into vocabulary nodes.
 *
 * @tparam          DescriptorFunctionsType
 *                  Supplies computeMeanDescriptor, computeDistance, toString,
 *                  fromString, toArray8U, fromArray8U and
 *                  DESCRIPTOR_LENGTH_BYTES.
 *
 * @note            DescriptorFunctionsType supplies ORB-compatible
 *                  descriptor operations and byte width. This object owns
 *                  its raw scoring pointer; wordNodes borrow treeNodes
 *                  addresses. No internal synchronization is provided;
 *                  callers protect mutation and descriptor-buffer sharing.
 */
template <class DescriptorType, class DescriptorFunctionsType>
class TemplatedVocabulary
{
  public:
    /*!
     * @brief           Creates an empty vocabulary with its scoring object.
     *
     * @param[in]       branchingFactor_in
     *                  Number of child clusters per internal node.
     *
     * @param[in]       depthLevels_in
     *                  Maximum training depth.
     *
     * @param[in]       weightingType_in
     *                  Weighting scheme.
     *
     * @param[in]       scoringType_in
     *                  Scoring scheme.
     */
    TemplatedVocabulary(
        int           branchingFactor_in = 10,
        int           depthLevels_in     = 5,
        WeightingType weightingType_in   = WeightingType::WEIGHTING_TYPE_TF_IDF,
        ScoringType   scoringType_in     = ScoringType::SCORING_TYPE_L1_NORM) :
        branchingFactor(branchingFactor_in),
        depthLevels(depthLevels_in),
        weightingType(weightingType_in),
        scoringType(scoringType_in),
        p_scoringObject(nullptr)
    {
        createScoringObject();
    }
    /*!
     * @brief           Loads a vocabulary through OpenCV FileStorage.
     *
     * @param[in]       fileName_in
     *                  Vocabulary file path.
     *
     * @note            Throws the original std::string when FileStorage cannot
     *                  open; OpenCV/allocation exceptions propagate.
     */
    TemplatedVocabulary(const std::string &fileName_in) :
        p_scoringObject(nullptr)
    {
        load(fileName_in);
    }
    /*!
     * @brief           Loads a vocabulary through OpenCV FileStorage.
     *
     * @param[in]       fileName_in
     *                  Vocabulary file path.
     *
     * @note            Throws the original std::string when FileStorage cannot
     *                  open; OpenCV/allocation exceptions propagate.
     */
    TemplatedVocabulary(const char *fileName_in) :
        p_scoringObject(nullptr)
    {
        load(fileName_in);
    }
    /*!
     * @brief           Copies a vocabulary using the original assignment
     *                  operation.
     *
     * @param[in]       otherVocabulary_in
     *                  Vocabulary to copy; descriptor storage retains upstream
     *                  sharing.
     */
    TemplatedVocabulary(
        const TemplatedVocabulary<DescriptorType, DescriptorFunctionsType>
            &otherVocabulary_in) :
        p_scoringObject(nullptr)
    {
        *this = otherVocabulary_in;
    }
    /*!
     * @brief           Deletes the owned scoring object and releases tree
     *                  storage.
     */
    virtual ~TemplatedVocabulary()
    {
        delete p_scoringObject;
    }
    /*!
     * @brief           Copies configuration and tree, rebuilding word pointers.
     *
     * @param[in]       otherVocabulary_in
     *                  Vocabulary to copy; descriptor storage retains upstream
     *                  sharing.
     *
     * @return          This vocabulary by reference.
     *
     * @note            Retains the upstream clear/copy/recreate sequence and
     *                  raw scoring ownership, including self-assignment
     *                  behavior.
     */
    TemplatedVocabulary<DescriptorType, DescriptorFunctionsType> &operator=(
        const TemplatedVocabulary<DescriptorType, DescriptorFunctionsType>
            &otherVocabulary_in);
    /*!
     * @brief           Trains the vocabulary from descriptors grouped by image.
     *
     * @param[in]       trainingFeatures_in
     *                  Training descriptors grouped by source image.
     */
    virtual void create(
        const std::vector<std::vector<DescriptorType>> &trainingFeatures_in);
    /*!
     * @brief           Trains with the given branching factor and depth.
     *
     * @param[in]       trainingFeatures_in
     *                  Training descriptors grouped by source image.
     *
     * @param[in]       branchingFactor_in
     *                  Number of child clusters per internal node.
     *
     * @param[in]       depthLevels_in
     *                  Maximum training depth.
     */
    virtual void create(
        const std::vector<std::vector<DescriptorType>> &trainingFeatures_in,
        int                                             branchingFactor_in,
        int                                             depthLevels_in);
    /*!
     * @brief           Trains with the given tree, weighting and scoring
     *                  settings.
     *
     * @param[in]       trainingFeatures_in
     *                  Training descriptors grouped by source image.
     *
     * @param[in]       branchingFactor_in
     *                  Number of child clusters per internal node.
     *
     * @param[in]       depthLevels_in
     *                  Maximum training depth.
     *
     * @param[in]       weightingType_in
     *                  Weighting scheme.
     *
     * @param[in]       scoringType_in
     *                  Scoring scheme.
     */
    virtual void create(
        const std::vector<std::vector<DescriptorType>> &trainingFeatures_in,
        int                                             branchingFactor_in,
        int                                             depthLevels_in,
        WeightingType                                   weightingType_in,
        ScoringType                                     scoringType_in);
    /*!
     * @brief           Returns the number of vocabulary leaf words.
     *
     * @return          Number of word nodes, narrowed to unsigned int as
     *                  upstream.
     */
    virtual inline unsigned int                 getWordCount() const;
    /*!
     * @brief           Reports whether the vocabulary has no words.
     *
     * @return          Whether the word-node sequence is empty.
     */
    virtual inline bool                         isEmpty() const;
    /*!
     * @brief           Computes a new normalized bag-of-words vector.
     *
     * @param[in]       featureDescriptors_in
     *                  Input feature descriptors in their original order.
     *
     * @return          Word weights, or an empty map for an empty vocabulary.
     */
    [[nodiscard]] virtual bow_vector::BowVector computeBagOfWords(
        const std::vector<DescriptorType> &featureDescriptors_in) const;
    /*!
     * @brief           Computes word weights and feature-node associations.
     *
     * @param[in]       featureDescriptors_in
     *                  Input feature descriptors in their original order.
     *
     * @param[out]      bowVector_out
     *                  Cleared and filled ordered word weights.
     *
     * @param[out]      featureVector_out
     *                  Cleared and filled node-to-feature associations.
     *
     * @param[in]       levelsUp_in
     *                  Ancestor levels above a word's tree leaf.
     */
    virtual void computeBagOfWords(
        const std::vector<DescriptorType> &featureDescriptors_in,
        bow_vector::BowVector             &bowVector_out,
        feature_vector::FeatureVector     &featureVector_out,
        int                                levelsUp_in) const;
    /*!
     * @brief           Finds the word for a descriptor, guarding empty
     *                  vocabularies.
     *
     * @param[in]       featureDescriptor_in
     *                  Descriptor used to traverse the tree.
     *
     * @return          Matched word index, or 0 when no vocabulary is loaded.
     *
     * @note            Returns 0 for an empty vocabulary, otherwise uses the
     *                  unchecked traversal.
     */
    virtual bow_vector::WordId
                  findWordId(const DescriptorType &featureDescriptor_in) const;
    /*!
     * @brief           Scores the two word vectors with the selected scheme.
     *
     * @param[in]       leftHandSideBowVector_in
     *                  First ordered, appropriately normalized word map.
     *
     * @param[in]       rightHandSideBowVector_in
     *                  Second ordered, appropriately normalized word map.
     *
     * @return          The scoring object's result.
     */
    inline double computeScore(
        const bow_vector::BowVector &leftHandSideBowVector_in,
        const bow_vector::BowVector &rightHandSideBowVector_in) const;
    /*!
     * @brief           Walks upward from a word's leaf to its ancestor.
     *
     * @param[in]       wordId_in
     *                  Valid word index; unchecked as upstream.
     *
     * @param[in]       levelsUp_in
     *                  Ancestor levels above a word's tree leaf.
     *
     * @return          Ancestor node identifier, stopping at the root.
     */
    virtual feature_vector::NodeId getParentNode(bow_vector::WordId wordId_in,
                                                 int levelsUp_in) const;
    /*!
     * @brief           Collects all leaf-word identifiers below a tree node.
     *
     * @param[in]       nodeId_in
     *                  Valid tree node index; unchecked as upstream.
     *
     * @return          Word identifiers in the original traversal order.
     */
    [[nodiscard]] std::vector<bow_vector::WordId>
               getWordsFromNode(feature_vector::NodeId nodeId_in) const;
    /*!
     * @brief           Returns the configured number of child clusters.
     *
     * @return          Stored branching factor.
     */
    inline int getBranchingFactor() const;
    /*!
     * @brief           Returns the configured maximum tree depth.
     *
     * @return          Stored depth levels.
     */
    inline int getDepthLevels() const;
    /*!
     * @brief           Computes the mean leaf depth.
     *
     * @return          Mean depth, using double division then float narrowing.
     *
     * @note            Returns the old double division narrowed to float; an
     *                  empty vocabulary is not handled.
     */
    float      getEffectiveLevels() const;
    /*!
     * @brief           Returns a word's descriptor header or value.
     *
     * @param[in]       wordId_in
     *                  Valid word index; unchecked as upstream.
     *
     * @return          Descriptor copy with its existing sharing semantics.
     *
     * @note            Returns the descriptor by value; cv::Mat still shares
     *                  the node's buffer.
     */
    virtual inline DescriptorType
        getWordDescriptor(bow_vector::WordId wordId_in) const;
    /*!
     * @brief           Returns a word's stored weight.
     *
     * @param[in]       wordId_in
     *                  Valid word index; unchecked as upstream.
     *
     * @return          Stored word weight.
     */
    virtual inline bow_vector::WordValue
                         getWordWeight(bow_vector::WordId wordId_in) const;
    /*!
     * @brief           Returns the configured weighting scheme.
     *
     * @return          Stored weighting type.
     */
    inline WeightingType getWeightingType() const;
    /*!
     * @brief           Returns the configured scoring scheme.
     *
     * @return          Stored scoring type.
     */
    inline ScoringType   getScoringType() const;
    /*!
     * @brief           Sets the weighting scheme without retraining.
     *
     * @param[in]       weightingType_in
     *                  Weighting scheme.
     */
    inline void          setWeightingType(WeightingType weightingType_in);
    /*!
     * @brief           Replaces the owned scoring object for a new scheme.
     *
     * @param[in]       scoringType_in
     *                  Scoring scheme.
     */
    void                 setScoringType(ScoringType scoringType_in);
    /*!
     * @brief           Loads the native text vocabulary representation.
     *
     * @param[in]       fileName_in
     *                  Vocabulary file path.
     *
     * @return          True at the original success exit, false on header
     *                  rejection.
     *
     * @note            Returns false for the original header rejection; partial
     *                  mutation, eof handling and messages are unchanged.
     */
    bool                 loadFromTextFile(const std::string &fileName_in);
    /*!
     * @brief           Writes the native text vocabulary representation.
     *
     * @param[in]       fileName_in
     *                  Vocabulary file path.
     */
    void                 saveToTextFile(const std::string &fileName_in) const;
    /*!
     * @brief           Loads native binary fields with the upstream stream
     *                  rules.
     *
     * @param[in]       fileName_in
     *                  Vocabulary file path.
     *
     * @return          False on the original stream/header failures, otherwise
     *                  true.
     *
     * @note            Keeps text-mode open, eof handling and partial mutation.
     *                  Build-only verification does not establish file
     *                  equivalence.
     */
    bool                 loadFromBinFile(const std::string &fileName_in);
    /*!
     * @brief           Writes the native binary vocabulary fields.
     *
     * @param[in]       fileName_in
     *                  Vocabulary file path.
     *
     * @note            Keeps original native field widths/order and the
     *                  upstream last-node loop bound.
     */
    void                 saveToBinFile(const std::string &fileName_in) const;
    /*!
     * @brief           Opens FileStorage and saves the vocabulary.
     *
     * @param[in]       fileName_in
     *                  Vocabulary file path.
     *
     * @note            Throws the original std::string when FileStorage cannot
     *                  open; OpenCV/allocation exceptions propagate.
     */
    void                 save(const std::string &fileName_in) const;
    /*!
     * @brief           Opens FileStorage and loads the vocabulary.
     *
     * @param[in]       fileName_in
     *                  Vocabulary file path.
     *
     * @note            Throws the original std::string when FileStorage cannot
     *                  open; OpenCV/allocation exceptions propagate.
     */
    void                 load(const std::string &fileName_in);
    /*!
     * @brief           Appends the vocabulary tree and words to FileStorage.
     *
     * @param[in,out]   fileStorage_inout
     *                  Open file storage to append the vocabulary to.
     *
     * @param[in]       nodeName_in
     *                  Vocabulary node key.
     */
    virtual void         save(cv::FileStorage   &fileStorage_inout,
                              const std::string &nodeName_in = "vocabulary") const;
    /*!
     * @brief           Replaces the vocabulary from a FileStorage node.
     *
     * @param[in]       fileStorage_in
     *                  Open file storage to read.
     *
     * @param[in]       nodeName_in
     *                  Vocabulary node key.
     */
    virtual void         load(const cv::FileStorage &fileStorage_in,
                              const std::string     &nodeName_in = "vocabulary");
    /*!
     * @brief           Zeros the weights of words below a threshold.
     *
     * @param[in]       minimumWeight_in
     *                  Words below this threshold have their weight set to
     *                  zero.
     *
     * @return          Number of words below the threshold, including already-
     *                  zero ones.
     *
     * @note            The old weight is forgotten; later lower thresholds
     *                  cannot restore stopped words.
     */
    virtual int          stopWords(double minimumWeight_in);

  protected:
    /*!
     * @brief           Borrowed non-null pointer into the caller's training
     *                  descriptors.
     */
    using DescriptorPointer = const DescriptorType *;
    /*!
     * @brief           Deletes and recreates the owned raw scoring object.
     */
    void                                         createScoringObject();
    /*!
     * @brief           Collects borrowed pointers into training descriptors.
     *
     * @param[in]       trainingFeatures_in
     *                  Training descriptors grouped by source image.
     *
     * @return          Descriptor addresses in image and feature order.
     */
    [[nodiscard]] std::vector<DescriptorPointer> collectFeaturePointers(
        const std::vector<std::vector<DescriptorType>> &trainingFeatures_in)
        const;
    /*!
     * @brief           Traverses to the best matching word and optional
     *                  ancestor.
     *
     * @param[in]       featureDescriptor_in
     *                  Descriptor used to traverse the tree.
     *
     * @param[out]      wordId_out
     *                  Matched leaf word index.
     *
     * @param[out]      wordWeight_out
     *                  Matched leaf's weight.
     *
     * @param[out]      p_nodeId_out
     *                  Nullable borrowed output for the requested ancestor
     *                  index.
     *
     * @param[in]       levelsUp_in
     *                  Ancestor levels above a word's tree leaf.
     *
     * @note            Precondition: nonempty valid tree and valid descriptor.
     *                  Output assignment order is unchanged.
     */
    virtual void findWord(const DescriptorType   &featureDescriptor_in,
                          bow_vector::WordId     &wordId_out,
                          bow_vector::WordValue  &wordWeight_out,
                          feature_vector::NodeId *p_nodeId_out = nullptr,
                          int                     levelsUp_in  = 0) const;
    /*!
     * @brief           Finds the word using the unchecked tree traversal.
     *
     * @param[in]       featureDescriptor_in
     *                  Descriptor used to traverse the tree.
     *
     * @return          Matched word identifier.
     *
     * @note            Precondition: nonempty valid tree. No public empty guard
     *                  is added here.
     */
    [[nodiscard]] virtual bow_vector::WordId
         findWordIdUnchecked(const DescriptorType &featureDescriptor_in) const;
    /*!
     * @brief           Clusters one tree level and recursively trains its
     *                  children.
     *
     * @param[in]       parentNodeId_in
     *                  Parent tree node for this training level.
     *
     * @param[in]       descriptorPointers_in
     *                  Borrowed descriptor addresses; descriptors must outlive
     *                  training.
     *
     * @param[in]       currentLevel_in
     *                  Current tree depth.
     *
     * @note            Preserves shared-buffer mean writes, training aliasing
     *                  and the upstream convergence loop.
     */
    void runHierarchicalKMeansStep(
        feature_vector::NodeId                parentNodeId_in,
        const std::vector<DescriptorPointer> &descriptorPointers_in,
        int                                   currentLevel_in);
    /*!
     * @brief           Seeds clusters with the overridable upstream algorithm.
     *
     * @param[in]       descriptorPointers_in
     *                  Borrowed descriptor addresses; descriptors must outlive
     *                  training.
     *
     * @return          Cluster descriptors sharing their original input
     *                  storage.
     *
     * @note            May be overridden for other seeding algorithms; returned
     *                  cv::Mat headers still share input storage.
     */
    [[nodiscard]] virtual std::vector<DescriptorType> initiateClusters(
        const std::vector<DescriptorPointer> &descriptorPointers_in) const;
    /*!
     * @brief           Seeds clusters with the original k-means++ draw
     *                  sequence.
     *
     * @param[in]       descriptorPointers_in
     *                  Borrowed descriptor addresses; descriptors must outlive
     *                  training.
     *
     * @return          Selected cluster descriptors in draw order.
     *
     * @note            Consumes the same process-wide DUtils RNG draws in the
     *                  same order; input must be nonempty.
     */
    [[nodiscard]] std::vector<DescriptorType> initiateClustersKMeansPlusPlus(
        const std::vector<DescriptorPointer> &descriptorPointers_in) const;
    /*!
     * @brief           Builds ordered borrowed word pointers for leaf nodes.
     */
    void createWords();
    /*!
     * @brief           Assigns unit or inverse-document-frequency word weights.
     *
     * @param[in]       trainingFeatures_in
     *                  Training descriptors grouped by source image.
     */
    void setNodeWeights(
        const std::vector<std::vector<DescriptorType>> &trainingFeatures_in);
    /*!
     * @brief           Frozen int branching-factor field.
     */
    int                                           branchingFactor;
    /*!
     * @brief           Frozen int depth field.
     */
    int                                           depthLevels;
    /*!
     * @brief           Frozen weighting scheme field.
     */
    WeightingType                                 weightingType;
    /*!
     * @brief           Frozen scoring scheme field.
     */
    ScoringType                                   scoringType;
    /*!
     * @brief           Owned nullable raw scoring pointer; deleted/replaced by
     *                  this vocabulary as upstream.
     */
    /* CPP §9 deviation: preserve raw ownership, approved D116, 2026-10-13. */
    scoring::GeneralScoring                      *p_scoringObject;
    /*!
     * @brief           Owned ordered tree nodes; growth may invalidate word-
     *                  node addresses.
     */
    std::vector<VocabularyNode<DescriptorType>>   treeNodes;
    /*!
     * @brief           Borrowed pointers into treeNodes; wordNodes[id]->wordId
     *                  equals id.
     */
    std::vector<VocabularyNode<DescriptorType> *> wordNodes;
};

} /* namespace vs_graphs::core::dbow2::vocabulary */

#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/collectFeaturePointers.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/computeBagOfWords.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/computeScore.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/create.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/createScoringObject.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/createWords.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/findWord.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/findWordId.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/findWordIdUnchecked.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/getBranchingFactor.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/getDepthLevels.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/getEffectiveLevels.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/getParentNode.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/getScoringType.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/getWeightingType.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/getWordCount.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/getWordDescriptor.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/getWordWeight.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/getWordsFromNode.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/initiateClusters.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/initiateClustersKMeansPlusPlus.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/isEmpty.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/load.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/loadFromBinFile.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/loadFromTextFile.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/operators/assignment/fromVocabulary.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/runHierarchicalKMeansStep.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/save.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/saveToBinFile.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/saveToTextFile.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/setNodeWeights.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/setScoringType.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/setWeightingType.tpp"
#include "dependency_lineage/dbow2/vocabulary/methods/TemplatedVocabularyClass/stopWords.tpp"

#endif /* VS_GRAPHS_CORE_SRC_DBOW2_VOCABULARY_OBJECTS_TEMPLATED_VOCABULARY_CLASS_H \
        */

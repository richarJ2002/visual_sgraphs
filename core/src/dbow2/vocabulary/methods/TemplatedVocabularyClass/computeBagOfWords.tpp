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
 * @file            computeBagOfWords.tpp
 *
 * @brief           Implements computeBagOfWords, declared in
 *                  TemplatedVocabularyClass.h.
 */

namespace vs_graphs::core::dbow2::vocabulary
{

template <class DescriptorType, class DescriptorFunctionsType>
bow_vector::BowVector
    TemplatedVocabulary<DescriptorType, DescriptorFunctionsType>::
        computeBagOfWords(
            const std::vector<DescriptorType> &featureDescriptors_in) const
{
    bow_vector::BowVector bowVector;
    bowVector.clear();

    if (isEmpty())
    {
        return bowVector;
    }

    /* normalize */
    bow_vector::NormType normType;
    bool mustNormalize = p_scoringObject->mustNormalize(normType);

    typename std::vector<DescriptorType>::const_iterator descriptorIterator;

    if (weightingType == WeightingType::WEIGHTING_TYPE_TF ||
        weightingType == WeightingType::WEIGHTING_TYPE_TF_IDF)
    {
        for (descriptorIterator = featureDescriptors_in.begin();
             descriptorIterator < featureDescriptors_in.end();
             descriptorIterator++)
        {
            bow_vector::WordId    wordId;
            bow_vector::WordValue wordWeight;
            /* w is the idf value if TF_IDF, 1 if TF */

            findWord(*descriptorIterator, wordId, wordWeight);

            /* not stopped */
            if (wordWeight > 0)
            {
                bowVector.addWeight(wordId, wordWeight);
            }
        }

        if (!bowVector.empty() && !mustNormalize)
        {
            /* unnecessary when normalizing */
            const double WORD_COUNT = bowVector.size();
            for (bow_vector::BowVector::iterator wordIterator =
                     bowVector.begin();
                 wordIterator != bowVector.end();
                 wordIterator++)
            {
                wordIterator->second /= WORD_COUNT;
            }
        }
    }
    else /* IDF || BINARY */
    {
        for (descriptorIterator = featureDescriptors_in.begin();
             descriptorIterator < featureDescriptors_in.end();
             descriptorIterator++)
        {
            bow_vector::WordId    wordId;
            bow_vector::WordValue wordWeight;
            /* w is idf if IDF, or 1 if BINARY */

            findWord(*descriptorIterator, wordId, wordWeight);

            /* not stopped */
            if (wordWeight > 0)
            {
                bowVector.addIfNotExist(wordId, wordWeight);
            }

        } /* if add_features */
    } /* if m_weighting == ... */

    if (mustNormalize)
    {
        bowVector.normalize(normType);
    }
    return bowVector;
}

template <class DescriptorType, class DescriptorFunctionsType>
void TemplatedVocabulary<DescriptorType, DescriptorFunctionsType>::
    computeBagOfWords(const std::vector<DescriptorType> &featureDescriptors_in,
                      bow_vector::BowVector             &bowVector_out,
                      feature_vector::FeatureVector     &featureVector_out,
                      int                                levelsUp_in) const
{
    bowVector_out.clear();
    featureVector_out.clear();

    if (isEmpty()) /* safe for subclasses */
    {
        return;
    }

    /* normalize */
    bow_vector::NormType normType;
    bool mustNormalize = p_scoringObject->mustNormalize(normType);

    typename std::vector<DescriptorType>::const_iterator descriptorIterator;

    if (weightingType == WeightingType::WEIGHTING_TYPE_TF ||
        weightingType == WeightingType::WEIGHTING_TYPE_TF_IDF)
    {
        unsigned int featureIndex = 0;
        for (descriptorIterator = featureDescriptors_in.begin();
             descriptorIterator < featureDescriptors_in.end();
             descriptorIterator++, featureIndex++)
        {
            bow_vector::WordId     wordId;
            feature_vector::NodeId nodeId;
            bow_vector::WordValue  wordWeight;
            /* w is the idf value if TF_IDF, 1 if TF */

            findWord(*descriptorIterator,
                     wordId,
                     wordWeight,
                     &nodeId,
                     levelsUp_in);

            if (wordWeight > 0) /* not stopped */
            {
                bowVector_out.addWeight(wordId, wordWeight);
                featureVector_out.addFeature(nodeId, featureIndex);
            }
        }

        if (!bowVector_out.empty() && !mustNormalize)
        {
            /* unnecessary when normalizing */
            const double WORD_COUNT = bowVector_out.size();
            for (bow_vector::BowVector::iterator wordIterator =
                     bowVector_out.begin();
                 wordIterator != bowVector_out.end();
                 wordIterator++)
            {
                wordIterator->second /= WORD_COUNT;
            }
        }
    }
    else /* IDF || BINARY */
    {
        unsigned int featureIndex = 0;
        for (descriptorIterator = featureDescriptors_in.begin();
             descriptorIterator < featureDescriptors_in.end();
             descriptorIterator++, featureIndex++)
        {
            bow_vector::WordId     wordId;
            feature_vector::NodeId nodeId;
            bow_vector::WordValue  wordWeight;
            /* w is idf if IDF, or 1 if BINARY */

            findWord(*descriptorIterator,
                     wordId,
                     wordWeight,
                     &nodeId,
                     levelsUp_in);

            if (wordWeight > 0) /* not stopped */
            {
                bowVector_out.addIfNotExist(wordId, wordWeight);
                featureVector_out.addFeature(nodeId, featureIndex);
            }
        }
    } /* if m_weighting == ... */

    if (mustNormalize)
    {
        bowVector_out.normalize(normType);
    }
}

} /* namespace vs_graphs::core::dbow2::vocabulary */

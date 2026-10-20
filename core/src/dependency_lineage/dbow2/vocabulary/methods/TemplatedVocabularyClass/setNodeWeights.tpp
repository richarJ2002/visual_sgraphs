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
 * @file            setNodeWeights.tpp
 *
 * @brief           Implements setNodeWeights, declared in
 *                  TemplatedVocabularyClass.h.
 */

namespace vs_graphs::core::dbow2::vocabulary
{

template <class DescriptorType, class DescriptorFunctionsType>
void TemplatedVocabulary<DescriptorType, DescriptorFunctionsType>::
    setNodeWeights(
        const std::vector<std::vector<DescriptorType>> &trainingFeatures_in)
{
    const unsigned int WORD_COUNT     = wordNodes.size();
    const unsigned int DOCUMENT_COUNT = trainingFeatures_in.size();

    if (weightingType == WeightingType::WEIGHTING_TYPE_TF ||
        weightingType == WeightingType::WEIGHTING_TYPE_BINARY)
    {
        /* idf part must be 1 always */
        for (unsigned int wordIndex = 0; wordIndex < WORD_COUNT; wordIndex++)
        {
            wordNodes[wordIndex]->wordWeight = 1;
        }
    }
    else if (weightingType == WeightingType::WEIGHTING_TYPE_IDF ||
             weightingType == WeightingType::WEIGHTING_TYPE_TF_IDF)
    {
        /* IDF and TF-IDF: we calculte the idf path now */

        /* Note: this actually calculates the idf part of the tf-idf score. */
        /* The complete tf-idf score is calculated in ::transform */

        std::vector<unsigned int> documentCounts(WORD_COUNT, 0);
        std::vector<bool>         isCounted(WORD_COUNT, false);

        typename std::vector<std::vector<DescriptorType>>::const_iterator
                                                             imageIterator;
        typename std::vector<DescriptorType>::const_iterator descriptorIterator;

        for (imageIterator = trainingFeatures_in.begin();
             imageIterator != trainingFeatures_in.end();
             imageIterator++)
        {
            std::fill(isCounted.begin(), isCounted.end(), false);

            for (descriptorIterator = imageIterator->begin();
                 descriptorIterator < imageIterator->end();
                 descriptorIterator++)
            {
                bow_vector::WordId wordId =
                    findWordIdUnchecked(*descriptorIterator);

                if (!isCounted[wordId])
                {
                    documentCounts[wordId]++;
                    isCounted[wordId] = true;
                }
            }
        }

        /* set ln(N/Ni) */
        for (unsigned int wordIndex = 0; wordIndex < WORD_COUNT; wordIndex++)
        {
            if (documentCounts[wordIndex] > 0)
            {
                wordNodes[wordIndex]->wordWeight =
                    std::log(static_cast<double>(DOCUMENT_COUNT) /
                             static_cast<double>(documentCounts[wordIndex]));
            } /* else // This cannot occur if using kmeans++ */
        }
    }
}

} /* namespace vs_graphs::core::dbow2::vocabulary */

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
 * @file            withVocabulary.tpp
 *
 * @brief           Implements withVocabulary, declared in
 *                  TemplatedVocabularyClass.h.
 */

namespace vs_graphs::core::dbow2::vocabulary
{

template <class DescriptorType, class DescriptorFunctionsType>
std::ostream &operator<<(
    std::ostream &outputStream_inout,
    const TemplatedVocabulary<DescriptorType, DescriptorFunctionsType>
        &visualVocabulary_in)
{
    outputStream_inout << "Vocabulary: k = "
                       << visualVocabulary_in.getBranchingFactor()
                       << ", L = " << visualVocabulary_in.getDepthLevels()
                       << ", Weighting = ";

    switch (visualVocabulary_in.getWeightingType())
    {
    case WeightingType::WEIGHTING_TYPE_TF_IDF:
        outputStream_inout << "tf-idf";
        break;
    case WeightingType::WEIGHTING_TYPE_TF:
        outputStream_inout << "tf";
        break;
    case WeightingType::WEIGHTING_TYPE_IDF:
        outputStream_inout << "idf";
        break;
    case WeightingType::WEIGHTING_TYPE_BINARY:
        outputStream_inout << "binary";
        break;
    }

    outputStream_inout << ", Scoring = ";
    switch (visualVocabulary_in.getScoringType())
    {
    case ScoringType::SCORING_TYPE_L1_NORM:
        outputStream_inout << "L1-norm";
        break;
    case ScoringType::SCORING_TYPE_L2_NORM:
        outputStream_inout << "L2-norm";
        break;
    case ScoringType::SCORING_TYPE_CHI_SQUARE:
        outputStream_inout << "Chi square distance";
        break;
    case ScoringType::SCORING_TYPE_KL:
        outputStream_inout << "KL-divergence";
        break;
    case ScoringType::SCORING_TYPE_BHATTACHARYYA:
        outputStream_inout << "Bhattacharyya coefficient";
        break;
    case ScoringType::SCORING_TYPE_DOT_PRODUCT:
        outputStream_inout << "Dot product";
        break;
    }

    outputStream_inout << ", Number of words = "
                       << visualVocabulary_in.getWordCount();

    return outputStream_inout;
}

} /* namespace vs_graphs::core::dbow2::vocabulary */

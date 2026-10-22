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
 * @file            initiateClustersKMeansPlusPlus.tpp
 *
 * @brief           Implements initiateClustersKMeansPlusPlus, declared in
 *                  TemplatedVocabularyClass.h.
 */

namespace vs_graphs::core::dbow2::vocabulary
{

template <class DescriptorType, class DescriptorFunctionsType>
std::vector<DescriptorType>
    TemplatedVocabulary<DescriptorType, DescriptorFunctionsType>::
        initiateClustersKMeansPlusPlus(
            const std::vector<DescriptorPointer> &descriptorPointers_in) const
{
    /* Implements kmeans++ seeding algorithm */
    /* Algorithm: */
    /* 1. Choose one center uniformly at random from among the data points. */
    /* 2. For each data point x, compute D(x), the distance between x and the */
    /* nearest */
    /* center that has already been chosen. */
    /* 3. Add one new data point as a center. Each point x is chosen with */
    /* probability */
    /* proportional to D(x)^2. */
    /* 4. Repeat Steps 2 and 3 until k centers have been chosen. */
    /* 5. Now that the initial centers have been chosen, proceed using standard
     */
    /* k-means */
    /* clustering. */

    vs_graphs::core::d_utils::random::Random::seedGeneratorOnce();

    std::vector<DescriptorType> clusterDescriptors;
    clusterDescriptors.resize(0);
    clusterDescriptors.reserve(branchingFactor);
    std::vector<double> minimumDistances(descriptorPointers_in.size(),
                                         std::numeric_limits<double>::max());

    /* 1. */

    int featureIndex =
        vs_graphs::core::d_utils::random::Random::generateUniformInteger(
            0,
            descriptorPointers_in.size() - 1);

    /* create first cluster */
    clusterDescriptors.push_back(*descriptorPointers_in[featureIndex]);

    /* compute the initial distances */
    typename std::vector<DescriptorPointer>::const_iterator descriptorIterator;
    std::vector<double>::iterator                           distanceIterator;
    distanceIterator = minimumDistances.begin();
    for (descriptorIterator = descriptorPointers_in.begin();
         descriptorIterator != descriptorPointers_in.end();
         descriptorIterator++, distanceIterator++)
    {
        *distanceIterator =
            DescriptorFunctionsType::computeDistance(*(*descriptorIterator),
                                                     clusterDescriptors.back());
    }

    while (static_cast<int>(clusterDescriptors.size()) < branchingFactor)
    {
        /* 2. */
        distanceIterator = minimumDistances.begin();
        for (descriptorIterator = descriptorPointers_in.begin();
             descriptorIterator != descriptorPointers_in.end();
             descriptorIterator++, distanceIterator++)
        {
            if (*distanceIterator > 0)
            {
                double clusterDistance =
                    DescriptorFunctionsType::computeDistance(
                        *(*descriptorIterator),
                        clusterDescriptors.back());
                if (clusterDistance < *distanceIterator)
                {
                    *distanceIterator = clusterDistance;
                }
            }
        }

        /* 3. */
        double distanceSum = std::accumulate(minimumDistances.begin(),
                                             minimumDistances.end(),
                                             0.0);

        if (distanceSum > 0)
        {
            double sampledDistance;
            do
            {
                sampledDistance = vs_graphs::core::d_utils::random::Random::
                    generateUniformValue<double>(0, distanceSum);
            }
            while (sampledDistance == 0.0);

            double accumulatedDistance = 0;
            for (distanceIterator = minimumDistances.begin();
                 distanceIterator != minimumDistances.end();
                 distanceIterator++)
            {
                accumulatedDistance += *distanceIterator;
                if (accumulatedDistance >= sampledDistance)
                {
                    break;
                }
            }

            if (distanceIterator == minimumDistances.end())
            {
                featureIndex = descriptorPointers_in.size() - 1;
            }
            else
            {
                featureIndex = distanceIterator - minimumDistances.begin();
            }

            clusterDescriptors.push_back(*descriptorPointers_in[featureIndex]);

        } /* if dist_sum > 0 */
        else
        {
            break;
        }

    } /* while(used_clusters < m_k) */
    return clusterDescriptors;
}

} /* namespace vs_graphs::core::dbow2::vocabulary */

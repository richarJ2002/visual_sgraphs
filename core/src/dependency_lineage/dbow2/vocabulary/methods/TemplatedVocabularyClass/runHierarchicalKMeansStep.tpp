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
 * @file            runHierarchicalKMeansStep.tpp
 *
 * @brief           Implements runHierarchicalKMeansStep, declared in
 *                  TemplatedVocabularyClass.h.
 */

namespace vs_graphs::core::dbow2::vocabulary
{

template <class DescriptorType, class DescriptorFunctionsType>
void TemplatedVocabulary<DescriptorType, DescriptorFunctionsType>::
    runHierarchicalKMeansStep(
        feature_vector::NodeId                parentNodeId_in,
        const std::vector<DescriptorPointer> &descriptorPointers_in,
        int                                   currentLevel_in)
{
    if (descriptorPointers_in.empty())
    {
        return;
    }

    /* features associated to each cluster */
    std::vector<DescriptorType> clusterDescriptors;
    std::vector<std::vector<unsigned int>>
        clusterGroups; /* groups[i] = [j1, j2, ...] */
    /* j1, j2, ... indices of descriptors associated to cluster i */

    clusterDescriptors.reserve(branchingFactor);
    clusterGroups.reserve(branchingFactor);

    /* const int msizes[] = { m_k, descriptors.size() }; */
    /* cv::SparseMat assoc(2, msizes, CV_8U); */
    /* cv::SparseMat last_assoc(2, msizes, CV_8U); */
    /* // assoc.row(cluster_idx).col(descriptor_idx) = 1 iif associated */

    if (static_cast<int>(descriptorPointers_in.size()) <= branchingFactor)
    {
        /* trivial case: one cluster per feature */
        clusterGroups.resize(descriptorPointers_in.size());

        for (unsigned int clusterIndex = 0;
             clusterIndex < descriptorPointers_in.size();
             clusterIndex++)
        {
            clusterGroups[clusterIndex].push_back(clusterIndex);
            clusterDescriptors.push_back(*descriptorPointers_in[clusterIndex]);
        }
    }
    else
    {
        /* select clusters and groups with kmeans */

        bool isFirstIteration = true;
        bool shouldContinue   = true;

        /* to check if clusters move after iterations */
        std::vector<int> lastAssociations, currentAssociations;

        while (shouldContinue)
        {
            /* 1. Calculate clusters */

            if (isFirstIteration)
            {
                /* random sample */
                clusterDescriptors = initiateClusters(descriptorPointers_in);
            }
            else
            {
                /* calculate cluster centres */

                for (unsigned int clusterIndex = 0;
                     clusterIndex < clusterDescriptors.size();
                     clusterIndex++)
                {
                    std::vector<DescriptorPointer> clusterDescriptorPointers;
                    clusterDescriptorPointers.reserve(
                        clusterGroups[clusterIndex].size());

                    /*!
                     *
                     * for(unsigned int d = 0; d < descriptors.size(); ++d)
                     * {
                     * if( assoc.find<unsigned char>(c, d) )
                     * {
                     * cluster_descriptors.push_back(descriptors[d]);
                     * }
                     * }
                     *
                     */

                    std::vector<unsigned int>::const_iterator groupIterator;
                    for (groupIterator = clusterGroups[clusterIndex].begin();
                         groupIterator != clusterGroups[clusterIndex].end();
                         groupIterator++)
                    {
                        clusterDescriptorPointers.push_back(
                            descriptorPointers_in[*groupIterator]);
                    }

                    DescriptorFunctionsType::computeMeanDescriptor(
                        clusterDescriptorPointers,
                        clusterDescriptors[clusterIndex]);
                }

            } /* if(!first_time) */

            /* 2. Associate features with clusters */

            /* calculate distances to cluster centers */
            clusterGroups.clear();
            clusterGroups.resize(clusterDescriptors.size(),
                                 std::vector<unsigned int>());
            currentAssociations.resize(descriptorPointers_in.size());

            /* assoc.clear(); */

            typename std::vector<DescriptorPointer>::const_iterator
                descriptorIterator;
            /* unsigned int d = 0; */
            for (descriptorIterator = descriptorPointers_in.begin();
                 descriptorIterator != descriptorPointers_in.end();
                 descriptorIterator++) /* , ++d) */
            {
                double bestDistance = DescriptorFunctionsType::computeDistance(
                    *(*descriptorIterator),
                    clusterDescriptors[0]);
                unsigned int bestClusterIndex = 0;

                for (unsigned int clusterIndex = 1;
                     clusterIndex < clusterDescriptors.size();
                     clusterIndex++)
                {
                    double clusterDistance =
                        DescriptorFunctionsType::computeDistance(
                            *(*descriptorIterator),
                            clusterDescriptors[clusterIndex]);
                    if (clusterDistance < bestDistance)
                    {
                        bestDistance     = clusterDistance;
                        bestClusterIndex = clusterIndex;
                    }
                }

                /* assoc.ref<unsigned char>(icluster, d) = 1; */

                clusterGroups[bestClusterIndex].push_back(
                    descriptorIterator - descriptorPointers_in.begin());
                currentAssociations[descriptorIterator -
                                    descriptorPointers_in.begin()] =
                    bestClusterIndex;
            }

            /* kmeans++ ensures all the clusters has any feature associated with
             */
            /* them */

            /* 3. check convergence */
            if (isFirstIteration)
            {
                isFirstIteration = false;
            }
            else
            {
                /* goon = !eqUChar(last_assoc, assoc); */

                shouldContinue = false;
                for (unsigned int clusterIndex = 0;
                     clusterIndex < currentAssociations.size();
                     clusterIndex++)
                {
                    if (currentAssociations[clusterIndex] !=
                        lastAssociations[clusterIndex])
                    {
                        shouldContinue = true;
                        break;
                    }
                }
            }

            if (shouldContinue)
            {
                /* copy last feature-cluster association */
                lastAssociations = currentAssociations;
                /* last_assoc = assoc.clone(); */
            }

        } /* while(goon) */

    } /* if must run kmeans */

    /* create nodes */
    for (unsigned int clusterIndex = 0;
         clusterIndex < clusterDescriptors.size();
         clusterIndex++)
    {
        feature_vector::NodeId newNodeId = treeNodes.size();
        treeNodes.push_back(VocabularyNode<DescriptorType>(newNodeId));
        treeNodes.back().nodeDescriptor = clusterDescriptors[clusterIndex];
        treeNodes.back().parentNodeId   = parentNodeId_in;
        treeNodes[parentNodeId_in].childNodeIds.push_back(newNodeId);
    }

    /* go on with the next level */
    if (currentLevel_in < depthLevels)
    {
        /* iterate again with the resulting clusters */
        const std::vector<feature_vector::NodeId> &childNodeIds =
            treeNodes[parentNodeId_in].childNodeIds;
        for (unsigned int clusterIndex = 0;
             clusterIndex < clusterDescriptors.size();
             clusterIndex++)
        {
            feature_vector::NodeId newNodeId = childNodeIds[clusterIndex];

            std::vector<DescriptorPointer> childFeaturePointers;
            childFeaturePointers.reserve(clusterGroups[clusterIndex].size());

            std::vector<unsigned int>::const_iterator groupIterator;
            for (groupIterator = clusterGroups[clusterIndex].begin();
                 groupIterator != clusterGroups[clusterIndex].end();
                 groupIterator++)
            {
                childFeaturePointers.push_back(
                    descriptorPointers_in[*groupIterator]);
            }

            if (childFeaturePointers.size() > 1)
            {
                runHierarchicalKMeansStep(newNodeId,
                                          childFeaturePointers,
                                          currentLevel_in + 1);
            }
        }
    }
}

} /* namespace vs_graphs::core::dbow2::vocabulary */

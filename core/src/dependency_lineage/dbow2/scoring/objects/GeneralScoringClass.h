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
 * File: ScoringObject.h
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: functions to compute bow scores
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
 * @file            GeneralScoringClass.h
 *
 * @brief           Declares GeneralScoringClass.
 */

#ifndef VS_GRAPHS_CORE_SRC_DBOW2_SCORING_OBJECTS_GENERAL_SCORING_CLASS_H
#define VS_GRAPHS_CORE_SRC_DBOW2_SCORING_OBJECTS_GENERAL_SCORING_CLASS_H

#include "dependency_lineage/dbow2/bow_vector/bow_vector.h"

namespace vs_graphs::core::dbow2::scoring
{

/*!
 * @brief           Polymorphic stateless bag-of-words scoring interface.
 */
class GeneralScoring
{
  public:
    /*!
     * @brief           Computes the upstream score between ordered word maps.
     *
     * @param[in]       leftHandSideBowVector_in
     *                  First vector, already normalized as required.
     *
     * @param[in]       rightHandSideBowVector_in
     *                  Second vector, already normalized as required.
     *
     * @return          Score in the selected upstream scheme; KL is asymmetric
     *                  and unscaled.
     */
    virtual double computeScore(
        const bow_vector::BowVector &leftHandSideBowVector_in,
        const bow_vector::BowVector &rightHandSideBowVector_in) const = 0;
    /*!
     * @brief           Reports whether normalization is required and its norm.
     *
     * @param[out]      normType_out
     *                  Norm is assigned even when normalization is not
     *                  required.
     *
     * @return          Whether both input vectors must be normalized.
     */
    virtual bool mustNormalize(bow_vector::NormType &normType_out) const = 0;
    /*!
     * @brief           Natural log of double epsilon; coupled to WordValue's
     *                  double type.
     */
    static const double LOG_EPSILON;
    /*!
     * @brief           Permits deletion through the scoring base pointer.
     */
    virtual ~GeneralScoring() {}
};

} /* namespace vs_graphs::core::dbow2::scoring */

#endif /* VS_GRAPHS_CORE_SRC_DBOW2_SCORING_OBJECTS_GENERAL_SCORING_CLASS_H */

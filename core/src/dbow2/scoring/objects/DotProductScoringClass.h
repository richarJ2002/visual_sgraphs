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
 * @file            DotProductScoringClass.h
 *
 * @brief           Declares DotProductScoringClass.
 */

#ifndef VS_GRAPHS_CORE_SRC_DBOW2_SCORING_OBJECTS_DOT_PRODUCT_SCORING_CLASS_H
#define VS_GRAPHS_CORE_SRC_DBOW2_SCORING_OBJECTS_DOT_PRODUCT_SCORING_CLASS_H

#include "dbow2/scoring/objects/GeneralScoringClass.h"

namespace vs_graphs::core::dbow2::scoring
{

/*!
 * @brief           Dot-product scoring using the unchanged DBoW2 equation.
 */
class DotProductScoring : public GeneralScoring
{
  public:
    /*!
     * @brief           Computes the unscaled dot product of ordered word maps.
     *
     * @param[in]       leftHandSideBowVector_in
     *                  First word vector.
     *
     * @param[in]       rightHandSideBowVector_in
     *                  Second word vector.
     *
     * @return          Unscaled upstream dot product.
     */
    double computeScore(
        const bow_vector::BowVector &leftHandSideBowVector_in,
        const bow_vector::BowVector &rightHandSideBowVector_in) const override;
    /*!
     * @brief           Reports that normalization is not required.
     *
     * @param[out]      normType_out
     *                  Assigned the L1 norm even though it is not required.
     *
     * @return          False; normalization is not required.
     */
    inline bool
        mustNormalize(bow_vector::NormType &normType_out) const override;
};

} /* namespace vs_graphs::core::dbow2::scoring */

#include "dbow2/scoring/methods/DotProductScoringClass/mustNormalize.h"

#endif /* VS_GRAPHS_CORE_SRC_DBOW2_SCORING_OBJECTS_DOT_PRODUCT_SCORING_CLASS_H \
        */

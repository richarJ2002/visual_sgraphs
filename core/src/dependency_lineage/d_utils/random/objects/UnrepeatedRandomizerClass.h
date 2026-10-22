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
 * Derived from the DUtils library (DLib) file, in the copy distributed
 * with DBoW2 in ORB-SLAM2,
 * core/Thirdparty/DBoW2/DUtils/Random.h
 * (class Random::UnrepeatedRandomizer); its notice follows, then the
 * LICENSE.txt it names (core/Thirdparty/DBoW2/LICENSE.txt).
 *
 * File: Random.h
 * Project: DUtils library
 * Author: Dorian Galvez-Lopez
 * Date: April 2010, November 2011
 * Description: manages pseudo-random numbers
 * License: see the LICENSE.txt file
 *
 * DBoW2: bag-of-words library for C++ with generic descriptors
 *
 * Copyright (c) 2015 Dorian Galvez-Lopez <http://doriangalvez.com>
 * (Universidad de Zaragoza)
 * All rights reserved.
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
 * @file            UnrepeatedRandomizerClass.h
 *
 * @brief           Declares the non-repeating pseudo-random integer source.
 */

#ifndef VS_GRAPHS_CORE_SRC_D_UTILS_RANDOM_OBJECTS_UNREPEATED_RANDOMIZER_CLASS_H
#define VS_GRAPHS_CORE_SRC_D_UTILS_RANDOM_OBJECTS_UNREPEATED_RANDOMIZER_CLASS_H

#include <vector>

namespace vs_graphs::core::d_utils::random
{

/*!
 * @brief           Draws every integer of a closed range once, in random
 *                  order, then starts again.
 *
 *                  The values come from Random::generateUniformInteger(),
 *                  so they follow the process-wide std::rand() sequence.
 *                  drawNextValue() calls Random::seedGeneratorOnce(), which
 *                  seeds that sequence with the current time unless a
 *                  seedGeneratorOnce() overload has seeded it already.
 */
class UnrepeatedRandomizer
{
  public:
    /*!
     * @brief           Creates a randomizer over [minimumValue_in,
     *                  maximumValue_in] with every value still to draw.
     *
     * @param[in]       minimumValue_in
     *                  One bound of the range, included.
     *
     * @param[in]       maximumValue_in
     *                  The other bound of the range, included; the bounds
     *                  may be given in either order.
     */
    UnrepeatedRandomizer(int minimumValue_in, int maximumValue_in)
    {
        if (minimumValue_in <= maximumValue_in)
        {
            minimumValue = minimumValue_in;
            maximumValue = maximumValue_in;
        }
        else
        {
            minimumValue = maximumValue_in;
            maximumValue = minimumValue_in;
        }

        createValues();
    }

    /*!
     * @brief           Destroys the randomizer.
     */
    ~UnrepeatedRandomizer() {}

    /*!
     * @brief           Copies a randomizer, including the values it has not
     *                  drawn yet.
     *
     * @param[in]       otherRandomizer_in
     *                  Randomizer to copy.
     */
    UnrepeatedRandomizer(const UnrepeatedRandomizer &otherRandomizer_in)
    {
        *this = otherRandomizer_in;
    }

    /*!
     * @brief           Copies a randomizer, including the values it has not
     *                  drawn yet.
     *
     * @param[in]       otherRandomizer_in
     *                  Randomizer to copy; assigning a randomizer to itself
     *                  changes nothing.
     *
     * @return          This randomizer.
     */
    UnrepeatedRandomizer &
        operator=(const UnrepeatedRandomizer &otherRandomizer_in);

    /*!
     * @brief           Draws a value not drawn since the range was last
     *                  filled.
     *
     * @return          A value of the range; when every value has been drawn,
     *                  the range is filled again first.
     */
    int drawNextValue();

    /*!
     * @brief           Tells whether every value of the range has been drawn.
     *
     * @return          True when no value is left; the next drawNextValue()
     *                  then fills the range again.
     */
    bool isEmpty() const;

    /*!
     * @brief           Counts the values not drawn yet.
     *
     * @return          Number of values left, converted to unsigned int.
     */
    unsigned int getRemainingCount() const;

    /*!
     * @brief           Makes every value of the range available again when
     *                  any has been drawn.
     */
    void reset();

  protected:
    /*!
     * @brief           Fills availableValues with every value of the range,
     *                  in increasing order.
     */
    void createValues();

    /*!
     * @brief           Smallest value of the range, included.
     */
    int minimumValue;

    /*!
     * @brief           Largest value of the range, included.
     */
    int maximumValue;

    /*!
     * @brief           Values not drawn yet; drawNextValue() removes the value
     *                  it returns.
     */
    std::vector<int> availableValues;
};

} /* namespace vs_graphs::core::d_utils::random */

#include "dependency_lineage/d_utils/random/methods/UnrepeatedRandomizerClass/getRemainingCount.h"
#include "dependency_lineage/d_utils/random/methods/UnrepeatedRandomizerClass/isEmpty.h"

#endif /* VS_GRAPHS_CORE_SRC_D_UTILS_RANDOM_OBJECTS_UNREPEATED_RANDOMIZER_CLASS_H \
        */

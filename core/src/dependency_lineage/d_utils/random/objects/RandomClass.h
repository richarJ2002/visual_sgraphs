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
 * (class Random); its notice follows, then the LICENSE.txt it names
 * (core/Thirdparty/DBoW2/LICENSE.txt).
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
 * @file            RandomClass.h
 *
 * @brief           Declares the pseudo-random number functions.
 */

#ifndef VS_GRAPHS_CORE_SRC_D_UTILS_RANDOM_OBJECTS_RANDOM_CLASS_H
#define VS_GRAPHS_CORE_SRC_D_UTILS_RANDOM_OBJECTS_RANDOM_CLASS_H

#include <cmath>
#include <cstdlib>

namespace vs_graphs::core::d_utils::random
{

/*!
 * @brief           Pseudo-random numbers drawn from the C library generator.
 *
 *                  Every function reads or reseeds the one process-wide
 *                  std::rand() sequence, so a seed set here also changes the
 *                  numbers any other std::rand() user receives. The class has
 *                  only static members.
 *
 * @note            Not synchronised: isAlreadySeeded is a plain static bool,
 *                  so calling seedGeneratorOnce() from two threads at once is
 *                  a data race.
 */
class Random
{
  public:
    /*!
     * @brief           Seeds std::rand() with the current wall-clock time.
     *
     * @note            The seed is the Unix time in whole seconds; two calls
     *                  within the same second give the same sequence. Does
     *                  not mark the generator as seeded for
     *                  seedGeneratorOnce().
     */
    static void seedGenerator();

    /*!
     * @brief           Seeds std::rand() with the current wall-clock time,
     *                  unless a seedGeneratorOnce() overload has already
     *                  seeded it.
     */
    static void seedGeneratorOnce();

    /*!
     * @brief           Seeds std::rand() with the given value.
     *
     * @param[in]       seedValue_in
     *                  Seed passed to std::srand(), converted to unsigned.
     *
     * @note            Does not mark the generator as seeded for
     *                  seedGeneratorOnce().
     */
    static void seedGenerator(int seedValue_in);

    /*!
     * @brief           Seeds std::rand() with the given value, unless a
     *                  seedGeneratorOnce() overload has already seeded it.
     *
     * @param[in]       seedValue_in
     *                  Seed passed to std::srand() on the first call.
     */
    static void seedGeneratorOnce(int seedValue_in);

    /*!
     * @brief           Draws a uniform value in [0, 1].
     *
     * @tparam          ValueType
     *                  Floating-point type of the value (the project
     *                  instantiates double).
     *
     * @return          std::rand() / RAND_MAX in ValueType; both 0 and 1 can
     *                  occur.
     */
    template <class ValueType> static ValueType generateUniformValue();

    /*!
     * @brief           Draws a uniform value between two bounds.
     *
     * @tparam          ValueType
     *                  Floating-point type of the value (the project
     *                  instantiates double).
     *
     * @param[in]       minimumValue_in
     *                  Lower bound, included.
     *
     * @param[in]       maximumValue_in
     *                  Upper bound, included.
     *
     * @return          u * (maximumValue_in - minimumValue_in) +
     *                  minimumValue_in, where u is generateUniformValue().
     */
    template <class ValueType>
    static ValueType generateUniformValue(ValueType minimumValue_in,
                                          ValueType maximumValue_in);

    /*!
     * @brief           Draws a uniform integer between two bounds.
     *
     * @param[in]       minimumValue_in
     *                  Lower bound, included.
     *
     * @param[in]       maximumValue_in
     *                  Upper bound, included.
     *
     * @return          An integer in [minimumValue_in, maximumValue_in].
     *
     * @pre             minimumValue_in <= maximumValue_in, and
     *                  maximumValue_in - minimumValue_in + 1 fits in an int;
     *                  no other case is handled.
     */
    static int generateUniformInteger(int minimumValue_in, int maximumValue_in);

    /*!
     * @brief           Draws a value from a normal distribution.
     *
     * @tparam          ValueType
     *                  Floating-point type of the value (the project
     *                  instantiates double).
     *
     * @param[in]       meanValue_in
     *                  Mean of the distribution.
     *
     * @param[in]       standardDeviation_in
     *                  Standard deviation of the distribution, in the unit of
     *                  meanValue_in.
     *
     * @return          meanValue_in plus a standard normal sample (polar
     *                  Box-Muller transformation) times standardDeviation_in.
     */
    template <class ValueType>
    static ValueType generateGaussianValue(ValueType meanValue_in,
                                           ValueType standardDeviation_in);

  private:
    /*!
     * @brief           True once a seedGeneratorOnce() overload has seeded
     *                  std::rand(); never reset. Defined once, in
     *                  static_members.cc.
     */
    static bool isAlreadySeeded;
};

} /* namespace vs_graphs::core::d_utils::random */

#include "dependency_lineage/d_utils/random/methods/RandomClass/generateGaussianValue.tpp"
#include "dependency_lineage/d_utils/random/methods/RandomClass/generateUniformValue.tpp"

#endif /* VS_GRAPHS_CORE_SRC_D_UTILS_RANDOM_OBJECTS_RANDOM_CLASS_H */

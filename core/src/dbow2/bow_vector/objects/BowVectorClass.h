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
 * File: BowVector.h
 * Date: March 2011
 * Author: Dorian Galvez-Lopez
 * Description: bag of words vector
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
 * @file            BowVectorClass.h
 *
 * @brief           Declares BowVectorClass.
 */

#ifndef VS_GRAPHS_CORE_SRC_DBOW2_BOW_VECTOR_OBJECTS_BOW_VECTOR_CLASS_H
#define VS_GRAPHS_CORE_SRC_DBOW2_BOW_VECTOR_OBJECTS_BOW_VECTOR_CLASS_H

#include <cstddef>
#include <map>
#include <ostream>
#include <string>

#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/map.hpp>

#include "dbow2/bow_vector/objects/NormTypeEnum.h"
#include "dbow2/bow_vector/objects/WordIdAlias.h"
#include "dbow2/bow_vector/objects/WordValueAlias.h"

namespace vs_graphs::core::dbow2::bow_vector
{

/*!
 * @brief           Ordered BowVector map with unchanged Boost base
 *                  serialization.
 *
 * @note            No internal synchronization; callers serialize access to
 *                  mutation.
 */
class BowVector : public std::map<WordId, WordValue>
{
  public:
    /*!
     * @brief           Creates an empty map.
     */
    BowVector() {}
    /*!
     * @brief           Releases the map entries.
     */
    ~BowVector() {}
    /*!
     * @brief           Adds a weight, accumulating an existing word.
     *
     * @param[in]       wordId_in
     *                  Word key.
     *
     * @param[in]       wordValue_in
     *                  Weight to add.
     */
    void addWeight(WordId wordId_in, WordValue wordValue_in);
    /*!
     * @brief           Adds a weight only when the word is absent.
     *
     * @param[in]       wordId_in
     *                  Word key.
     *
     * @param[in]       wordValue_in
     *                  Initial weight.
     */
    void addIfNotExist(WordId wordId_in, WordValue wordValue_in);
    /*!
     * @brief           Normalizes weights when their norm is positive.
     *
     * @param[in]       normType_in
     *                  Norm to use.
     */
    void normalize(NormType normType_in);
    /*!
     * @brief           Writes a dense space-separated Matlab vector.
     *
     * @param[in]       fileName_in
     *                  Destination path.
     *
     * @param[in]       wordCount_in
     *                  Length of the dense vector.
     *
     * @note            Stream failure behavior is inherited unchanged from
     *                  upstream.
     */
    void saveToMatlabFile(const std::string &fileName_in,
                          std::size_t        wordCount_in) const;
    /*!
     * @brief           Writes the ordered entries to a stream.
     *
     * @param[in,out]   outputStream_inout
     *                  Stream to append to.
     *
     * @param[in]       bowVector_in
     *                  Map to print.
     *
     * @return          The same stream.
     */
    friend std::ostream &operator<<(std::ostream    &outputStream_inout,
                                    const BowVector &bowVector_in);

  private:
    friend class boost::serialization::access;
    /*!
     * @brief           Archives exactly the original std::map base.
     *
     * @param[in,out]   ar
     *                  Boost archive.
     *
     * @param[in]       version
     *                  Unused Boost version.
     */
    template <class Archive>
    void serialize(Archive &ar, [[maybe_unused]] const int version);
};

} /* namespace vs_graphs::core::dbow2::bow_vector */

#include "dbow2/bow_vector/methods/serialize.tpp"

#endif /* VS_GRAPHS_CORE_SRC_DBOW2_BOW_VECTOR_OBJECTS_BOW_VECTOR_CLASS_H */

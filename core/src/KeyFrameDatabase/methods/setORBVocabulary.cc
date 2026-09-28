/*!
 * This file is part of ORB-SLAM3.
 * Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details:
 * https://www.gnu.org/licenses/
 */

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include <mutex>

using namespace std;

namespace vs_graphs
{
namespace core
{

void KeyFrameDatabase::setORBVocabulary(ORBVocabulary *p_orbVocabulary_in)
{
    ORBVocabulary **p_vocabularySlot;
    p_vocabularySlot  = (ORBVocabulary **)(&p_vocabulary);
    *p_vocabularySlot = p_orbVocabulary_in;

    invertedFile.clear();
    invertedFile.resize(p_vocabulary->size());
}

} // namespace core
} // namespace vs_graphs

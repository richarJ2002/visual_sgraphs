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

/*!
 * @file            compFirst.cc
 *
 * @brief           Implements compFirst(), declared in
 *                  KeyFrameDatabase/private_functions.h.
 */

#include "keyframe_database/objects/KeyFrameDatabaseClass.h"

#include "keyframe/keyframe.h"

#include <mutex>

#include "private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace keyframe_database
{

bool compFirst(const std::pair<float, keyframe::KeyFrame *> &firstScoredCandidate_in,
               const std::pair<float, keyframe::KeyFrame *> &secondScoredCandidate_in)
{
    return firstScoredCandidate_in.first > secondScoredCandidate_in.first;
}

} /* namespace keyframe_database */
} /* namespace core */
} /* namespace vs_graphs */

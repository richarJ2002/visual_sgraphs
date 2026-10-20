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
 * @file            setORBVocabulary.cc
 *
 * @brief           Implements KeyFrameDatabase::setORBVocabulary(), declared in
 *                  KeyFrameDatabase.h.
 */

#include "place_recognition_and_loop_closure/keyframe_database/objects/KeyFrameDatabaseClass.h"

#include "map_state/keyframe/keyframe.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{
namespace keyframe_database
{

KeyFrameDatabaseStatus
    KeyFrameDatabase::setORBVocabulary(ORBVocabulary *p_orbVocabulary_in)
{
    std::unique_lock<std::mutex> lock(databaseMutex);
    p_vocabulary = p_orbVocabulary_in;

    invertedFile.clear();
    invertedFile.resize(p_vocabulary->getWordCount());

    return KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS;
}

} /* namespace keyframe_database */
} /* namespace core */
} /* namespace vs_graphs */

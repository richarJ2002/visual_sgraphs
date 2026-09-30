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
 * @file            erase.cc
 *
 * @brief           Implements KeyFrameDatabase::erase(), declared in
 *                  KeyFrameDatabase.h.
 */

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

KeyFrameDatabaseStatus KeyFrameDatabase::erase(KeyFrame *p_keyFrame_in)
{
    std::unique_lock<std::mutex> lock(databaseMutex);

    // Erase elements in the Inverse File for the entry
    for (DBoW2::BowVector::const_iterator
             wordIt  = p_keyFrame_in->bowVector.begin(),
             wordEnd = p_keyFrame_in->bowVector.end();
         wordIt != wordEnd;
         wordIt++)
    {
        // List of keyframes that share the word
        std::list<KeyFrame *> &keyFramesForWord = invertedFile[wordIt->first];

        for (std::list<KeyFrame *>::iterator
                 keyFrameIt  = keyFramesForWord.begin(),
                 keyFrameEnd = keyFramesForWord.end();
             keyFrameIt != keyFrameEnd;
             keyFrameIt++)
        {
            if (p_keyFrame_in == *keyFrameIt)
            {
                keyFramesForWord.erase(keyFrameIt);
                break;
            }
        }
    }

    return KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

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
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

KeyFrameDatabaseStatus KeyFrameDatabase::clearMap(Map *p_map_in)
{
    std::unique_lock<std::mutex> lock(databaseMutex);

    // Erase elements in the Inverse File for the entry
    for (std::vector<std::list<KeyFrame *>>::iterator
             invertedFileRowIt  = invertedFile.begin(),
             invertedFileRowEnd = invertedFile.end();
         invertedFileRowIt != invertedFileRowEnd;
         invertedFileRowIt++)
    {
        // List of keyframes that share the word
        std::list<KeyFrame *> &keyFramesForWord = *invertedFileRowIt;

        for (std::list<KeyFrame *>::iterator
                 keyFrameIt  = keyFramesForWord.begin(),
                 keyFrameEnd = keyFramesForWord.end();
             keyFrameIt != keyFrameEnd;)
        {
            KeyFrame *p_candidateKeyFrame    = *keyFrameIt;
            Map      *p_candidateKeyFrameMap = nullptr;
            if (p_candidateKeyFrame->getMap(p_candidateKeyFrameMap) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_map_in == p_candidateKeyFrameMap)
            {
                keyFrameIt = keyFramesForWord.erase(keyFrameIt);
                // Dont delete the KF because the class Map clean all the KF
                // when it is destroyed
            }
            else
            {
                ++keyFrameIt;
            }
        }
    }

    return KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

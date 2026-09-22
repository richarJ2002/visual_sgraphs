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

void KeyFrameDatabase::clearMap(Map *pMap)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for (std::vector<list<KeyFrame *>>::iterator vit  = invertedFile.begin(),
                                                 vend = invertedFile.end();
         vit != vend;
         vit++)
    {
        // List of keyframes that share the word
        list<KeyFrame *> &lKFs = *vit;

        for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end();
             lit != lend;)
        {
            KeyFrame *pKFi = *lit;
            if (pMap == pKFi->getMap())
            {
                lit = lKFs.erase(lit);
                // Dont delete the KF because the class Map clean all the KF
                // when it is destroyed
            }
            else
            {
                ++lit;
            }
        }
    }
}

} // namespace core
} // namespace vs_graphs

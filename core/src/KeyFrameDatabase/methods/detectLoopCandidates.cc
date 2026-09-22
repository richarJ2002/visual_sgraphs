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

vector<KeyFrame *> KeyFrameDatabase::detectLoopCandidates(KeyFrame *pKF,
                                                          float     minScore)
{
    set<KeyFrame *>  spConnectedKeyFrames = pKF->getConnectedKeyFrames();
    list<KeyFrame *> lKFsSharingWords;

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    {
        unique_lock<mutex> lock(mMutex);

        for (DBoW2::BowVector::const_iterator vit  = pKF->bowVector.begin(),
                                              vend = pKF->bowVector.end();
             vit != vend;
             vit++)
        {
            list<KeyFrame *> &lKFs = invertedFile[vit->first];

            for (list<KeyFrame *>::iterator lit  = lKFs.begin(),
                                            lend = lKFs.end();
                 lit != lend;
                 lit++)
            {
                KeyFrame *pKFi = *lit;
                if (pKFi->getMap() ==
                    pKF->getMap()) // For consider a loop candidate it a
                                   // candidate it must be in the same map
                {
                    if (pKFi->loopQuery != pKF->mnId)
                    {
                        pKFi->loopWords = 0;
                        if (!spConnectedKeyFrames.count(pKFi))
                        {
                            pKFi->loopQuery = pKF->mnId;
                            lKFsSharingWords.push_back(pKFi);
                        }
                    }
                    pKFi->loopWords++;
                }
            }
        }
    }

    if (lKFsSharingWords.empty())
        return vector<KeyFrame *>();

    list<pair<float, KeyFrame *>> lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    int maxCommonWords = 0;
    for (list<KeyFrame *>::iterator lit  = lKFsSharingWords.begin(),
                                    lend = lKFsSharingWords.end();
         lit != lend;
         lit++)
    {
        if ((*lit)->loopWords > maxCommonWords)
            maxCommonWords = (*lit)->loopWords;
    }

    int minCommonWords = maxCommonWords * 0.8f;

    int nscores = 0;

    // Compute similarity score. Retain the matches whose score is higher than
    // minScore
    for (list<KeyFrame *>::iterator lit  = lKFsSharingWords.begin(),
                                    lend = lKFsSharingWords.end();
         lit != lend;
         lit++)
    {
        KeyFrame *pKFi = *lit;

        if (pKFi->loopWords > minCommonWords)
        {
            nscores++;

            float si = p_vocabulary->score(pKF->bowVector, pKFi->bowVector);

            pKFi->loopScore = si;
            if (si >= minScore)
                lScoreAndMatch.push_back(make_pair(si, pKFi));
        }
    }

    if (lScoreAndMatch.empty())
        return vector<KeyFrame *>();

    list<pair<float, KeyFrame *>> lAccScoreAndMatch;
    float                         bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    for (list<pair<float, KeyFrame *>>::iterator it    = lScoreAndMatch.begin(),
                                                 itend = lScoreAndMatch.end();
         it != itend;
         it++)
    {
        KeyFrame          *pKFi     = it->second;
        vector<KeyFrame *> vpNeighs = pKFi->getBestCovisibilityKeyFrames(10);

        float     bestScore = it->first;
        float     accScore  = it->first;
        KeyFrame *pBestKF   = pKFi;
        for (vector<KeyFrame *>::iterator vit  = vpNeighs.begin(),
                                          vend = vpNeighs.end();
             vit != vend;
             vit++)
        {
            KeyFrame *pKF2 = *vit;
            if (pKF2->loopQuery == pKF->mnId &&
                pKF2->loopWords > minCommonWords)
            {
                accScore += pKF2->loopScore;
                if (pKF2->loopScore > bestScore)
                {
                    pBestKF   = pKF2;
                    bestScore = pKF2->loopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
        if (accScore > bestAccScore)
            bestAccScore = accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f * bestAccScore;

    set<KeyFrame *>    spAlreadyAddedKF;
    vector<KeyFrame *> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    for (list<pair<float, KeyFrame *>>::iterator
             it    = lAccScoreAndMatch.begin(),
             itend = lAccScoreAndMatch.end();
         it != itend;
         it++)
    {
        if (it->first > minScoreToRetain)
        {
            KeyFrame *pKFi = it->second;
            if (!spAlreadyAddedKF.count(pKFi))
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpLoopCandidates;
}

} // namespace core
} // namespace vs_graphs

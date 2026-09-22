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

void KeyFrameDatabase::detectBestCandidates(KeyFrame           *pKF,
                                            vector<KeyFrame *> &vpLoopCand,
                                            vector<KeyFrame *> &vpMergeCand,
                                            int                 nMinWords)
{
    list<KeyFrame *> lKFsSharingWords;
    set<KeyFrame *>  spConnectedKF;

    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(mMutex);

        spConnectedKF = pKF->getConnectedKeyFrames();

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
                if (spConnectedKF.find(pKFi) != spConnectedKF.end())
                {
                    continue;
                }
                if (pKFi->placeRecognitionQuery != pKF->mnId)
                {
                    pKFi->placeRecognitionWords = 0;
                    pKFi->placeRecognitionQuery = pKF->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->placeRecognitionWords++;
            }
        }
    }
    if (lKFsSharingWords.empty())
        return;

    // Only compare against those keyframes that share enough words
    int maxCommonWords = 0;
    for (list<KeyFrame *>::iterator lit  = lKFsSharingWords.begin(),
                                    lend = lKFsSharingWords.end();
         lit != lend;
         lit++)
    {
        if ((*lit)->placeRecognitionWords > maxCommonWords)
            maxCommonWords = (*lit)->placeRecognitionWords;
    }

    int minCommonWords = maxCommonWords * 0.8f;

    if (minCommonWords < nMinWords)
    {
        minCommonWords = nMinWords;
    }

    list<pair<float, KeyFrame *>> lScoreAndMatch;

    int nscores = 0;

    // Compute similarity score.
    for (list<KeyFrame *>::iterator lit  = lKFsSharingWords.begin(),
                                    lend = lKFsSharingWords.end();
         lit != lend;
         lit++)
    {
        KeyFrame *pKFi = *lit;

        if (pKFi->placeRecognitionWords > minCommonWords)
        {
            nscores++;
            float si = p_vocabulary->score(pKF->bowVector, pKFi->bowVector);
            pKFi->placeRecognitionScore = si;
            lScoreAndMatch.push_back(make_pair(si, pKFi));
        }
    }

    if (lScoreAndMatch.empty())
        return;

    list<pair<float, KeyFrame *>> lAccScoreAndMatch;
    float                         bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for (list<pair<float, KeyFrame *>>::iterator it    = lScoreAndMatch.begin(),
                                                 itend = lScoreAndMatch.end();
         it != itend;
         it++)
    {
        KeyFrame          *pKFi     = it->second;
        vector<KeyFrame *> vpNeighs = pKFi->getBestCovisibilityKeyFrames(10);

        float     bestScore = it->first;
        float     accScore  = bestScore;
        KeyFrame *pBestKF   = pKFi;
        for (vector<KeyFrame *>::iterator vit  = vpNeighs.begin(),
                                          vend = vpNeighs.end();
             vit != vend;
             vit++)
        {
            KeyFrame *pKF2 = *vit;
            if (pKF2->placeRecognitionQuery != pKF->mnId)
                continue;

            accScore += pKF2->placeRecognitionScore;
            if (pKF2->placeRecognitionScore > bestScore)
            {
                pBestKF   = pKF2;
                bestScore = pKF2->placeRecognitionScore;
            }
        }
        lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
        if (accScore > bestAccScore)
            bestAccScore = accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float           minScoreToRetain = 0.75f * bestAccScore;
    set<KeyFrame *> spAlreadyAddedKF;
    vpLoopCand.reserve(lAccScoreAndMatch.size());
    vpMergeCand.reserve(lAccScoreAndMatch.size());
    for (list<pair<float, KeyFrame *>>::iterator
             it    = lAccScoreAndMatch.begin(),
             itend = lAccScoreAndMatch.end();
         it != itend;
         it++)
    {
        const float &si = it->first;
        if (si > minScoreToRetain)
        {
            KeyFrame *pKFi = it->second;
            if (!spAlreadyAddedKF.count(pKFi))
            {
                if (pKF->getMap() == pKFi->getMap())
                {
                    vpLoopCand.push_back(pKFi);
                }
                else
                {
                    vpMergeCand.push_back(pKFi);
                }
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }
}

} // namespace core
} // namespace vs_graphs

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

KeyFrameDatabase::KeyFrameDatabase(const ORBVocabulary &voc) :
    p_vocabulary(&voc)
{
    invertedFile.resize(voc.size());
}

void KeyFrameDatabase::add(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

    for (DBoW2::BowVector::const_iterator vit  = pKF->bowVector.begin(),
                                          vend = pKF->bowVector.end();
         vit != vend;
         vit++)
        invertedFile[vit->first].push_back(pKF);
}

void KeyFrameDatabase::erase(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for (DBoW2::BowVector::const_iterator vit  = pKF->bowVector.begin(),
                                          vend = pKF->bowVector.end();
         vit != vend;
         vit++)
    {
        // List of keyframes that share the word
        list<KeyFrame *> &lKFs = invertedFile[vit->first];

        for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end();
             lit != lend;
             lit++)
        {
            if (pKF == *lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}

void KeyFrameDatabase::clear()
{
    invertedFile.clear();
    invertedFile.resize(p_vocabulary->size());
}

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

void KeyFrameDatabase::detectCandidates(KeyFrame           *pKF,
                                        float               minScore,
                                        vector<KeyFrame *> &vpLoopCand,
                                        vector<KeyFrame *> &vpMergeCand)
{
    set<KeyFrame *>  spConnectedKeyFrames = pKF->getConnectedKeyFrames();
    list<KeyFrame *> lKFsSharingWordsLoop, lKFsSharingWordsMerge;

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
                            lKFsSharingWordsLoop.push_back(pKFi);
                        }
                    }
                    pKFi->loopWords++;
                }
                else if (!pKFi->getMap()->isBad())
                {
                    if (pKFi->mergeQuery != pKF->mnId)
                    {
                        pKFi->mergeWords = 0;
                        if (!spConnectedKeyFrames.count(pKFi))
                        {
                            pKFi->mergeQuery = pKF->mnId;
                            lKFsSharingWordsMerge.push_back(pKFi);
                        }
                    }
                    pKFi->mergeWords++;
                }
            }
        }
    }

    if (lKFsSharingWordsLoop.empty() && lKFsSharingWordsMerge.empty())
        return;

    if (!lKFsSharingWordsLoop.empty())
    {
        list<pair<float, KeyFrame *>> lScoreAndMatch;

        // Only compare against those keyframes that share enough words
        int maxCommonWords = 0;
        for (list<KeyFrame *>::iterator lit  = lKFsSharingWordsLoop.begin(),
                                        lend = lKFsSharingWordsLoop.end();
             lit != lend;
             lit++)
        {
            if ((*lit)->loopWords > maxCommonWords)
                maxCommonWords = (*lit)->loopWords;
        }

        int minCommonWords = maxCommonWords * 0.8f;

        int nscores = 0;

        // Compute similarity score. Retain the matches whose score is higher
        // than minScore
        for (list<KeyFrame *>::iterator lit  = lKFsSharingWordsLoop.begin(),
                                        lend = lKFsSharingWordsLoop.end();
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

        if (!lScoreAndMatch.empty())
        {
            list<pair<float, KeyFrame *>> lAccScoreAndMatch;
            float                         bestAccScore = minScore;

            // Lets now accumulate score by covisibility
            for (list<pair<float, KeyFrame *>>::iterator
                     it    = lScoreAndMatch.begin(),
                     itend = lScoreAndMatch.end();
                 it != itend;
                 it++)
            {
                KeyFrame          *pKFi = it->second;
                vector<KeyFrame *> vpNeighs =
                    pKFi->getBestCovisibilityKeyFrames(10);

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

            // Return all those keyframes with a score higher than
            // 0.75*bestScore
            float minScoreToRetain = 0.75f * bestAccScore;

            set<KeyFrame *> spAlreadyAddedKF;
            vpLoopCand.reserve(lAccScoreAndMatch.size());

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
                        vpLoopCand.push_back(pKFi);
                        spAlreadyAddedKF.insert(pKFi);
                    }
                }
            }
        }
    }

    if (!lKFsSharingWordsMerge.empty())
    {
        list<pair<float, KeyFrame *>> lScoreAndMatch;

        // Only compare against those keyframes that share enough words
        int maxCommonWords = 0;
        for (list<KeyFrame *>::iterator lit  = lKFsSharingWordsMerge.begin(),
                                        lend = lKFsSharingWordsMerge.end();
             lit != lend;
             lit++)
        {
            if ((*lit)->mergeWords > maxCommonWords)
                maxCommonWords = (*lit)->mergeWords;
        }

        int minCommonWords = maxCommonWords * 0.8f;

        int nscores = 0;

        // Compute similarity score. Retain the matches whose score is higher
        // than minScore
        for (list<KeyFrame *>::iterator lit  = lKFsSharingWordsMerge.begin(),
                                        lend = lKFsSharingWordsMerge.end();
             lit != lend;
             lit++)
        {
            KeyFrame *pKFi = *lit;

            if (pKFi->mergeWords > minCommonWords)
            {
                nscores++;

                float si = p_vocabulary->score(pKF->bowVector, pKFi->bowVector);

                pKFi->mergeScore = si;
                if (si >= minScore)
                    lScoreAndMatch.push_back(make_pair(si, pKFi));
            }
        }

        if (!lScoreAndMatch.empty())
        {
            list<pair<float, KeyFrame *>> lAccScoreAndMatch;
            float                         bestAccScore = minScore;

            // Lets now accumulate score by covisibility
            for (list<pair<float, KeyFrame *>>::iterator
                     it    = lScoreAndMatch.begin(),
                     itend = lScoreAndMatch.end();
                 it != itend;
                 it++)
            {
                KeyFrame          *pKFi = it->second;
                vector<KeyFrame *> vpNeighs =
                    pKFi->getBestCovisibilityKeyFrames(10);

                float     bestScore = it->first;
                float     accScore  = it->first;
                KeyFrame *pBestKF   = pKFi;
                for (vector<KeyFrame *>::iterator vit  = vpNeighs.begin(),
                                                  vend = vpNeighs.end();
                     vit != vend;
                     vit++)
                {
                    KeyFrame *pKF2 = *vit;
                    if (pKF2->mergeQuery == pKF->mnId &&
                        pKF2->mergeWords > minCommonWords)
                    {
                        accScore += pKF2->mergeScore;
                        if (pKF2->mergeScore > bestScore)
                        {
                            pBestKF   = pKF2;
                            bestScore = pKF2->mergeScore;
                        }
                    }
                }

                lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
                if (accScore > bestAccScore)
                    bestAccScore = accScore;
            }

            // Return all those keyframes with a score higher than
            // 0.75*bestScore
            float minScoreToRetain = 0.75f * bestAccScore;

            set<KeyFrame *> spAlreadyAddedKF;
            vpMergeCand.reserve(lAccScoreAndMatch.size());

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
                        vpMergeCand.push_back(pKFi);
                        spAlreadyAddedKF.insert(pKFi);
                    }
                }
            }
        }
    }

    for (DBoW2::BowVector::const_iterator vit  = pKF->bowVector.begin(),
                                          vend = pKF->bowVector.end();
         vit != vend;
         vit++)
    {
        list<KeyFrame *> &lKFs = invertedFile[vit->first];

        for (list<KeyFrame *>::iterator lit = lKFs.begin(), lend = lKFs.end();
             lit != lend;
             lit++)
        {
            KeyFrame *pKFi   = *lit;
            pKFi->loopQuery  = -1;
            pKFi->mergeQuery = -1;
        }
    }
}

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

bool compFirst(const pair<float, KeyFrame *> &a,
               const pair<float, KeyFrame *> &b)
{
    return a.first > b.first;
}

void KeyFrameDatabase::detectNBestCandidates(KeyFrame           *pKF,
                                             vector<KeyFrame *> &vpLoopCand,
                                             vector<KeyFrame *> &vpMergeCand,
                                             int                 nNumCandidates)
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

                if (pKFi->placeRecognitionQuery != pKF->mnId)
                {
                    pKFi->placeRecognitionWords = 0;
                    if (!spConnectedKF.count(pKFi))
                    {

                        pKFi->placeRecognitionQuery = pKF->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
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

    lAccScoreAndMatch.sort(compFirst);

    vpLoopCand.reserve(nNumCandidates);
    vpMergeCand.reserve(nNumCandidates);
    set<KeyFrame *>                         spAlreadyAddedKF;
    int                                     i  = 0;
    list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin();
    while (i < lAccScoreAndMatch.size() &&
           (vpLoopCand.size() < nNumCandidates ||
            vpMergeCand.size() < nNumCandidates))
    {
        KeyFrame *pKFi = it->second;
        if (pKFi->isBad())
        {
            i++;
            it++;
            continue;
        }

        if (!spAlreadyAddedKF.count(pKFi))
        {
            if (pKF->getMap() == pKFi->getMap() &&
                vpLoopCand.size() < nNumCandidates)
            {
                vpLoopCand.push_back(pKFi);
            }
            else if (pKF->getMap() != pKFi->getMap() &&
                     vpMergeCand.size() < nNumCandidates &&
                     !pKFi->getMap()->isBad())
            {
                vpMergeCand.push_back(pKFi);
            }
            spAlreadyAddedKF.insert(pKFi);
        }
        i++;
        it++;
    }
}

vector<KeyFrame *> KeyFrameDatabase::detectRelocalizationCandidates(Frame *F,
                                                                    Map   *pMap)
{
    list<KeyFrame *> lKFsSharingWords;

    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(mMutex);

        for (DBoW2::BowVector::const_iterator vit  = F->bowVector.begin(),
                                              vend = F->bowVector.end();
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
                if (pKFi->relocQuery != F->mnId)
                {
                    pKFi->relocWords = 0;
                    pKFi->relocQuery = F->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->relocWords++;
            }
        }
    }
    if (lKFsSharingWords.empty())
        return vector<KeyFrame *>();

    // Only compare against those keyframes that share enough words
    int maxCommonWords = 0;
    for (list<KeyFrame *>::iterator lit  = lKFsSharingWords.begin(),
                                    lend = lKFsSharingWords.end();
         lit != lend;
         lit++)
    {
        if ((*lit)->relocWords > maxCommonWords)
            maxCommonWords = (*lit)->relocWords;
    }

    int minCommonWords = maxCommonWords * 0.8f;

    list<pair<float, KeyFrame *>> lScoreAndMatch;

    int nscores = 0;

    // Compute similarity score.
    for (list<KeyFrame *>::iterator lit  = lKFsSharingWords.begin(),
                                    lend = lKFsSharingWords.end();
         lit != lend;
         lit++)
    {
        KeyFrame *pKFi = *lit;

        if (pKFi->relocWords > minCommonWords)
        {
            nscores++;
            float si = p_vocabulary->score(F->bowVector, pKFi->bowVector);
            pKFi->relocScore = si;
            lScoreAndMatch.push_back(make_pair(si, pKFi));
        }
    }

    if (lScoreAndMatch.empty())
        return vector<KeyFrame *>();

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
            if (pKF2->relocQuery != F->mnId)
                continue;

            accScore += pKF2->relocScore;
            if (pKF2->relocScore > bestScore)
            {
                pBestKF   = pKF2;
                bestScore = pKF2->relocScore;
            }
        }
        lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
        if (accScore > bestAccScore)
            bestAccScore = accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float              minScoreToRetain = 0.75f * bestAccScore;
    set<KeyFrame *>    spAlreadyAddedKF;
    vector<KeyFrame *> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
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
            if (pKFi->getMap() != pMap)
                continue;
            if (!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}

void KeyFrameDatabase::setORBVocabulary(ORBVocabulary *pORBVoc)
{
    ORBVocabulary **ptr;
    ptr  = (ORBVocabulary **)(&p_vocabulary);
    *ptr = pORBVoc;

    invertedFile.clear();
    invertedFile.resize(p_vocabulary->size());
}

} // namespace core
} // namespace vs_graphs

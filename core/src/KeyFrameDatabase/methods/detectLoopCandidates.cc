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

vector<KeyFrame *>
    KeyFrameDatabase::detectLoopCandidates(KeyFrame *p_currentKeyFrame_in,
                                           float     minScore_in)
{
    set<KeyFrame *> connectedKeyFrames =
        p_currentKeyFrame_in->getConnectedKeyFrames();
    list<KeyFrame *> keyFramesSharingWords;

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    {
        unique_lock<mutex> lock(databaseMutex);

        for (DBoW2::BowVector::const_iterator
                 wordIt  = p_currentKeyFrame_in->bowVector.begin(),
                 wordEnd = p_currentKeyFrame_in->bowVector.end();
             wordIt != wordEnd;
             wordIt++)
        {
            list<KeyFrame *> &keyFramesForWord = invertedFile[wordIt->first];

            for (list<KeyFrame *>::iterator
                     keyFrameIt  = keyFramesForWord.begin(),
                     keyFrameEnd = keyFramesForWord.end();
                 keyFrameIt != keyFrameEnd;
                 keyFrameIt++)
            {
                KeyFrame *p_candidateKeyFrame = *keyFrameIt;
                if (p_candidateKeyFrame->getMap() ==
                    p_currentKeyFrame_in
                        ->getMap()) // For consider a loop candidate it a
                                    // candidate it must be in the same map
                {
                    if (p_candidateKeyFrame->loopQuery !=
                        p_currentKeyFrame_in->id)
                    {
                        p_candidateKeyFrame->loopWords = 0;
                        if (!connectedKeyFrames.count(p_candidateKeyFrame))
                        {
                            p_candidateKeyFrame->loopQuery =
                                p_currentKeyFrame_in->id;
                            keyFramesSharingWords.push_back(
                                p_candidateKeyFrame);
                        }
                    }
                    p_candidateKeyFrame->loopWords++;
                }
            }
        }
    }

    if (keyFramesSharingWords.empty())
        return vector<KeyFrame *>();

    list<pair<float, KeyFrame *>> scoredCandidates;

    // Only compare against those keyframes that share enough words
    int maxCommonWordCount = 0;
    for (list<KeyFrame *>::iterator keyFrameIt  = keyFramesSharingWords.begin(),
                                    keyFrameEnd = keyFramesSharingWords.end();
         keyFrameIt != keyFrameEnd;
         keyFrameIt++)
    {
        if ((*keyFrameIt)->loopWords > maxCommonWordCount)
            maxCommonWordCount = (*keyFrameIt)->loopWords;
    }

    int minCommonWordCount = maxCommonWordCount * 0.8f;

    int scoredCandidateCount = 0;

    // Compute similarity score. Retain the matches whose score is higher than
    // minScore
    for (list<KeyFrame *>::iterator keyFrameIt  = keyFramesSharingWords.begin(),
                                    keyFrameEnd = keyFramesSharingWords.end();
         keyFrameIt != keyFrameEnd;
         keyFrameIt++)
    {
        KeyFrame *p_candidateKeyFrame = *keyFrameIt;

        if (p_candidateKeyFrame->loopWords > minCommonWordCount)
        {
            scoredCandidateCount++;

            float candidateScore =
                p_vocabulary->score(p_currentKeyFrame_in->bowVector,
                                    p_candidateKeyFrame->bowVector);

            p_candidateKeyFrame->loopScore = candidateScore;
            if (candidateScore >= minScore_in)
                scoredCandidates.push_back(
                    make_pair(candidateScore, p_candidateKeyFrame));
        }
    }

    if (scoredCandidates.empty())
        return vector<KeyFrame *>();

    list<pair<float, KeyFrame *>> accumulatedScoredCandidates;
    float                         bestAccumulatedScore = minScore_in;

    // Lets now accumulate score by covisibility
    for (list<pair<float, KeyFrame *>>::iterator
             scoredCandidateIt  = scoredCandidates.begin(),
             scoredCandidateEnd = scoredCandidates.end();
         scoredCandidateIt != scoredCandidateEnd;
         scoredCandidateIt++)
    {
        KeyFrame          *p_candidateKeyFrame = scoredCandidateIt->second;
        vector<KeyFrame *> covisibilityNeighborKeyFrames =
            p_candidateKeyFrame->getBestCovisibilityKeyFrames(10);

        float     bestGroupScore        = scoredCandidateIt->first;
        float     accumulatedScore      = scoredCandidateIt->first;
        KeyFrame *p_bestScoringKeyFrame = p_candidateKeyFrame;
        for (vector<KeyFrame *>::iterator
                 wordIt  = covisibilityNeighborKeyFrames.begin(),
                 wordEnd = covisibilityNeighborKeyFrames.end();
             wordIt != wordEnd;
             wordIt++)
        {
            KeyFrame *p_neighborKeyFrame = *wordIt;
            if (p_neighborKeyFrame->loopQuery == p_currentKeyFrame_in->id &&
                p_neighborKeyFrame->loopWords > minCommonWordCount)
            {
                accumulatedScore += p_neighborKeyFrame->loopScore;
                if (p_neighborKeyFrame->loopScore > bestGroupScore)
                {
                    p_bestScoringKeyFrame = p_neighborKeyFrame;
                    bestGroupScore        = p_neighborKeyFrame->loopScore;
                }
            }
        }

        accumulatedScoredCandidates.push_back(
            make_pair(accumulatedScore, p_bestScoringKeyFrame));
        if (accumulatedScore > bestAccumulatedScore)
            bestAccumulatedScore = accumulatedScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f * bestAccumulatedScore;

    set<KeyFrame *>    alreadyAddedKeyFrames;
    vector<KeyFrame *> loopCandidateKeyFrames;
    loopCandidateKeyFrames.reserve(accumulatedScoredCandidates.size());

    for (list<pair<float, KeyFrame *>>::iterator
             scoredCandidateIt  = accumulatedScoredCandidates.begin(),
             scoredCandidateEnd = accumulatedScoredCandidates.end();
         scoredCandidateIt != scoredCandidateEnd;
         scoredCandidateIt++)
    {
        if (scoredCandidateIt->first > minScoreToRetain)
        {
            KeyFrame *p_candidateKeyFrame = scoredCandidateIt->second;
            if (!alreadyAddedKeyFrames.count(p_candidateKeyFrame))
            {
                loopCandidateKeyFrames.push_back(p_candidateKeyFrame);
                alreadyAddedKeyFrames.insert(p_candidateKeyFrame);
            }
        }
    }

    return loopCandidateKeyFrames;
}

} // namespace core
} // namespace vs_graphs

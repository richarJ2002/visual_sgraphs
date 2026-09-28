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
    KeyFrameDatabase::detectRelocalizationCandidates(Frame *p_frame_in,
                                                     Map   *p_map_in)
{
    list<KeyFrame *> keyFramesSharingWords;

    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(databaseMutex);

        for (DBoW2::BowVector::const_iterator
                 wordIt  = p_frame_in->bowVector.begin(),
                 wordEnd = p_frame_in->bowVector.end();
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
                if (p_candidateKeyFrame->relocQuery != p_frame_in->id)
                {
                    p_candidateKeyFrame->relocWords = 0;
                    p_candidateKeyFrame->relocQuery = p_frame_in->id;
                    keyFramesSharingWords.push_back(p_candidateKeyFrame);
                }
                p_candidateKeyFrame->relocWords++;
            }
        }
    }
    if (keyFramesSharingWords.empty())
        return vector<KeyFrame *>();

    // Only compare against those keyframes that share enough words
    int maxCommonWordCount = 0;
    for (list<KeyFrame *>::iterator keyFrameIt  = keyFramesSharingWords.begin(),
                                    keyFrameEnd = keyFramesSharingWords.end();
         keyFrameIt != keyFrameEnd;
         keyFrameIt++)
    {
        if ((*keyFrameIt)->relocWords > maxCommonWordCount)
            maxCommonWordCount = (*keyFrameIt)->relocWords;
    }

    int minCommonWordCount = maxCommonWordCount * 0.8f;

    list<pair<float, KeyFrame *>> scoredCandidates;

    int scoredCandidateCount = 0;

    // Compute similarity score.
    for (list<KeyFrame *>::iterator keyFrameIt  = keyFramesSharingWords.begin(),
                                    keyFrameEnd = keyFramesSharingWords.end();
         keyFrameIt != keyFrameEnd;
         keyFrameIt++)
    {
        KeyFrame *p_candidateKeyFrame = *keyFrameIt;

        if (p_candidateKeyFrame->relocWords > minCommonWordCount)
        {
            scoredCandidateCount++;
            float candidateScore =
                p_vocabulary->score(p_frame_in->bowVector,
                                    p_candidateKeyFrame->bowVector);
            p_candidateKeyFrame->relocScore = candidateScore;
            scoredCandidates.push_back(
                make_pair(candidateScore, p_candidateKeyFrame));
        }
    }

    if (scoredCandidates.empty())
        return vector<KeyFrame *>();

    list<pair<float, KeyFrame *>> accumulatedScoredCandidates;
    float                         bestAccumulatedScore = 0;

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
        float     accumulatedScore      = bestGroupScore;
        KeyFrame *p_bestScoringKeyFrame = p_candidateKeyFrame;
        for (vector<KeyFrame *>::iterator
                 wordIt  = covisibilityNeighborKeyFrames.begin(),
                 wordEnd = covisibilityNeighborKeyFrames.end();
             wordIt != wordEnd;
             wordIt++)
        {
            KeyFrame *p_neighborKeyFrame = *wordIt;
            if (p_neighborKeyFrame->relocQuery != p_frame_in->id)
                continue;

            accumulatedScore += p_neighborKeyFrame->relocScore;
            if (p_neighborKeyFrame->relocScore > bestGroupScore)
            {
                p_bestScoringKeyFrame = p_neighborKeyFrame;
                bestGroupScore        = p_neighborKeyFrame->relocScore;
            }
        }
        accumulatedScoredCandidates.push_back(
            make_pair(accumulatedScore, p_bestScoringKeyFrame));
        if (accumulatedScore > bestAccumulatedScore)
            bestAccumulatedScore = accumulatedScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float              minScoreToRetain = 0.75f * bestAccumulatedScore;
    set<KeyFrame *>    alreadyAddedKeyFrames;
    vector<KeyFrame *> relocalizationCandidateKeyFrames;
    relocalizationCandidateKeyFrames.reserve(
        accumulatedScoredCandidates.size());
    for (list<pair<float, KeyFrame *>>::iterator
             scoredCandidateIt  = accumulatedScoredCandidates.begin(),
             scoredCandidateEnd = accumulatedScoredCandidates.end();
         scoredCandidateIt != scoredCandidateEnd;
         scoredCandidateIt++)
    {
        const float &candidateScore = scoredCandidateIt->first;
        if (candidateScore > minScoreToRetain)
        {
            KeyFrame *p_candidateKeyFrame = scoredCandidateIt->second;
            if (p_candidateKeyFrame->getMap() != p_map_in)
                continue;
            if (!alreadyAddedKeyFrames.count(p_candidateKeyFrame))
            {
                relocalizationCandidateKeyFrames.push_back(p_candidateKeyFrame);
                alreadyAddedKeyFrames.insert(p_candidateKeyFrame);
            }
        }
    }

    return relocalizationCandidateKeyFrames;
}

} // namespace core
} // namespace vs_graphs

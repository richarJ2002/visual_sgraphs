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
#include "Map.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include <mutex>
#include <rclcpp/logging.hpp>

using namespace std;

namespace vs_graphs
{
namespace core
{

KeyFrameDatabaseStatus KeyFrameDatabase::detectCandidates(
    KeyFrame           *p_currentKeyFrame_in,
    float               minScore_in,
    vector<KeyFrame *> &loopCandidateKeyFrames_out,
    vector<KeyFrame *> &mergeCandidateKeyFrames_out)
{
    std::set<KeyFrame *> connectedKeyFrames{};
    if (p_currentKeyFrame_in->getConnectedKeyFrames(connectedKeyFrames) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getConnectedKeyFrames returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    list<KeyFrame *> keyFramesSharingWordsLoop, keyFramesSharingWordsMerge;

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
                KeyFrame *p_candidateKeyFrame    = *keyFrameIt;
                Map      *p_candidateKeyFrameMap = nullptr;
                if (p_candidateKeyFrame->getMap(p_candidateKeyFrameMap) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                Map *p_currentKeyFrameMap = nullptr;
                if (p_currentKeyFrame_in->getMap(p_currentKeyFrameMap) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (p_candidateKeyFrameMap ==
                    p_currentKeyFrameMap) // For consider a loop candidate it a
                                          // candidate it must be in the same
                                          // map
                {
                    if (p_candidateKeyFrame->loopQuery !=
                        p_currentKeyFrame_in->id)
                    {
                        p_candidateKeyFrame->loopWords = 0;
                        if (!connectedKeyFrames.count(p_candidateKeyFrame))
                        {
                            p_candidateKeyFrame->loopQuery =
                                p_currentKeyFrame_in->id;
                            keyFramesSharingWordsLoop.push_back(
                                p_candidateKeyFrame);
                        }
                    }
                    p_candidateKeyFrame->loopWords++;
                }
                else
                {
                    Map *p_candidateKeyFrameMap2 = nullptr;
                    if (p_candidateKeyFrame->getMap(p_candidateKeyFrameMap2) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getMap returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    bool isBad2{};
                    if (p_candidateKeyFrameMap2->isBad(isBad2) !=
                        MapStatus::MAP_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    if (!isBad2)
                    {
                        if (p_candidateKeyFrame->mergeQuery !=
                            p_currentKeyFrame_in->id)
                        {
                            p_candidateKeyFrame->mergeWords = 0;
                            if (!connectedKeyFrames.count(p_candidateKeyFrame))
                            {
                                p_candidateKeyFrame->mergeQuery =
                                    p_currentKeyFrame_in->id;
                                keyFramesSharingWordsMerge.push_back(
                                    p_candidateKeyFrame);
                            }
                        }
                        p_candidateKeyFrame->mergeWords++;
                    }
                }
            }
        }
    }

    if (keyFramesSharingWordsLoop.empty() && keyFramesSharingWordsMerge.empty())
        return KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS;

    if (!keyFramesSharingWordsLoop.empty())
    {
        list<pair<float, KeyFrame *>> scoredCandidates;

        // Only compare against those keyframes that share enough words
        int maxCommonWordCount = 0;
        for (list<KeyFrame *>::iterator
                 keyFrameIt  = keyFramesSharingWordsLoop.begin(),
                 keyFrameEnd = keyFramesSharingWordsLoop.end();
             keyFrameIt != keyFrameEnd;
             keyFrameIt++)
        {
            if ((*keyFrameIt)->loopWords > maxCommonWordCount)
                maxCommonWordCount = (*keyFrameIt)->loopWords;
        }

        int minCommonWordCount = maxCommonWordCount * 0.8f;

        int scoredCandidateCount = 0;

        // Compute similarity score. Retain the matches whose score is higher
        // than minScore
        for (list<KeyFrame *>::iterator
                 keyFrameIt  = keyFramesSharingWordsLoop.begin(),
                 keyFrameEnd = keyFramesSharingWordsLoop.end();
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

        if (!scoredCandidates.empty())
        {
            list<pair<float, KeyFrame *>> accumulatedScoredCandidates;
            float                         bestAccumulatedScore = minScore_in;

            // Lets now accumulate score by covisibility
            for (list<pair<float, KeyFrame *>>::iterator
                     scoredCandidateIt  = scoredCandidates.begin(),
                     scoredCandidateEnd = scoredCandidates.end();
                 scoredCandidateIt != scoredCandidateEnd;
                 scoredCandidateIt++)
            {
                KeyFrame *p_candidateKeyFrame = scoredCandidateIt->second;
                std::vector<KeyFrame *> covisibilityNeighborKeyFrames{};
                if (p_candidateKeyFrame->getBestCovisibilityKeyFrames(
                        10,
                        covisibilityNeighborKeyFrames) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getBestCovisibilityKeyFrames returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }

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
                    if (p_neighborKeyFrame->loopQuery ==
                            p_currentKeyFrame_in->id &&
                        p_neighborKeyFrame->loopWords > minCommonWordCount)
                    {
                        accumulatedScore += p_neighborKeyFrame->loopScore;
                        if (p_neighborKeyFrame->loopScore > bestGroupScore)
                        {
                            p_bestScoringKeyFrame = p_neighborKeyFrame;
                            bestGroupScore = p_neighborKeyFrame->loopScore;
                        }
                    }
                }

                accumulatedScoredCandidates.push_back(
                    make_pair(accumulatedScore, p_bestScoringKeyFrame));
                if (accumulatedScore > bestAccumulatedScore)
                    bestAccumulatedScore = accumulatedScore;
            }

            // Return all those keyframes with a score higher than
            // 0.75*bestScore
            float minScoreToRetain = 0.75f * bestAccumulatedScore;

            set<KeyFrame *> alreadyAddedKeyFrames;
            loopCandidateKeyFrames_out.reserve(
                accumulatedScoredCandidates.size());

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
                        loopCandidateKeyFrames_out.push_back(
                            p_candidateKeyFrame);
                        alreadyAddedKeyFrames.insert(p_candidateKeyFrame);
                    }
                }
            }
        }
    }

    if (!keyFramesSharingWordsMerge.empty())
    {
        list<pair<float, KeyFrame *>> scoredCandidates;

        // Only compare against those keyframes that share enough words
        int maxCommonWordCount = 0;
        for (list<KeyFrame *>::iterator
                 keyFrameIt  = keyFramesSharingWordsMerge.begin(),
                 keyFrameEnd = keyFramesSharingWordsMerge.end();
             keyFrameIt != keyFrameEnd;
             keyFrameIt++)
        {
            if ((*keyFrameIt)->mergeWords > maxCommonWordCount)
                maxCommonWordCount = (*keyFrameIt)->mergeWords;
        }

        int minCommonWordCount = maxCommonWordCount * 0.8f;

        int scoredCandidateCount = 0;

        // Compute similarity score. Retain the matches whose score is higher
        // than minScore
        for (list<KeyFrame *>::iterator
                 keyFrameIt  = keyFramesSharingWordsMerge.begin(),
                 keyFrameEnd = keyFramesSharingWordsMerge.end();
             keyFrameIt != keyFrameEnd;
             keyFrameIt++)
        {
            KeyFrame *p_candidateKeyFrame = *keyFrameIt;

            if (p_candidateKeyFrame->mergeWords > minCommonWordCount)
            {
                scoredCandidateCount++;

                float candidateScore =
                    p_vocabulary->score(p_currentKeyFrame_in->bowVector,
                                        p_candidateKeyFrame->bowVector);

                p_candidateKeyFrame->mergeScore = candidateScore;
                if (candidateScore >= minScore_in)
                    scoredCandidates.push_back(
                        make_pair(candidateScore, p_candidateKeyFrame));
            }
        }

        if (!scoredCandidates.empty())
        {
            list<pair<float, KeyFrame *>> accumulatedScoredCandidates;
            float                         bestAccumulatedScore = minScore_in;

            // Lets now accumulate score by covisibility
            for (list<pair<float, KeyFrame *>>::iterator
                     scoredCandidateIt  = scoredCandidates.begin(),
                     scoredCandidateEnd = scoredCandidates.end();
                 scoredCandidateIt != scoredCandidateEnd;
                 scoredCandidateIt++)
            {
                KeyFrame *p_candidateKeyFrame = scoredCandidateIt->second;
                std::vector<KeyFrame *> covisibilityNeighborKeyFrames{};
                if (p_candidateKeyFrame->getBestCovisibilityKeyFrames(
                        10,
                        covisibilityNeighborKeyFrames) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getBestCovisibilityKeyFrames returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }

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
                    if (p_neighborKeyFrame->mergeQuery ==
                            p_currentKeyFrame_in->id &&
                        p_neighborKeyFrame->mergeWords > minCommonWordCount)
                    {
                        accumulatedScore += p_neighborKeyFrame->mergeScore;
                        if (p_neighborKeyFrame->mergeScore > bestGroupScore)
                        {
                            p_bestScoringKeyFrame = p_neighborKeyFrame;
                            bestGroupScore = p_neighborKeyFrame->mergeScore;
                        }
                    }
                }

                accumulatedScoredCandidates.push_back(
                    make_pair(accumulatedScore, p_bestScoringKeyFrame));
                if (accumulatedScore > bestAccumulatedScore)
                    bestAccumulatedScore = accumulatedScore;
            }

            // Return all those keyframes with a score higher than
            // 0.75*bestScore
            float minScoreToRetain = 0.75f * bestAccumulatedScore;

            set<KeyFrame *> alreadyAddedKeyFrames;
            mergeCandidateKeyFrames_out.reserve(
                accumulatedScoredCandidates.size());

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
                        mergeCandidateKeyFrames_out.push_back(
                            p_candidateKeyFrame);
                        alreadyAddedKeyFrames.insert(p_candidateKeyFrame);
                    }
                }
            }
        }
    }

    for (DBoW2::BowVector::const_iterator
             wordIt  = p_currentKeyFrame_in->bowVector.begin(),
             wordEnd = p_currentKeyFrame_in->bowVector.end();
         wordIt != wordEnd;
         wordIt++)
    {
        list<KeyFrame *> &keyFramesForWord = invertedFile[wordIt->first];

        for (list<KeyFrame *>::iterator keyFrameIt  = keyFramesForWord.begin(),
                                        keyFrameEnd = keyFramesForWord.end();
             keyFrameIt != keyFrameEnd;
             keyFrameIt++)
        {
            KeyFrame *p_candidateKeyFrame   = *keyFrameIt;
            p_candidateKeyFrame->loopQuery  = -1;
            p_candidateKeyFrame->mergeQuery = -1;
        }
    }

    return KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs

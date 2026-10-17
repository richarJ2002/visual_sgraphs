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
 * @file            detectCandidates.cc
 *
 * @brief           Implements KeyFrameDatabase::detectCandidates(), declared in
 *                  KeyFrameDatabase.h.
 */

#include "keyframe_database/objects/KeyFrameDatabaseClass.h"
#include "keyframe/objects/KeyFrameStatusEnum.h"
#include "map/objects/MapStatusEnum.h"

#include "dbow2/bow_vector/bow_vector.h"
#include "keyframe/keyframe.h"
#include "map/map.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace keyframe_database
{

KeyFrameDatabaseStatus KeyFrameDatabase::detectCandidates(
    keyframe::KeyFrame                *p_currentKeyFrame_in,
    float                    minScore_in,
    std::vector<keyframe::KeyFrame *> &loopCandidateKeyFrames_out,
    std::vector<keyframe::KeyFrame *> &mergeCandidateKeyFrames_out)
{
    std::set<keyframe::KeyFrame *> connectedKeyFrames{};
    if (p_currentKeyFrame_in->getConnectedKeyFrames(connectedKeyFrames) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getConnectedKeyFrames returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::list<keyframe::KeyFrame *> keyFramesSharingWordsLoop, keyFramesSharingWordsMerge;

    /*!
     * Search all keyframes that share a word with current keyframes
     * Discard keyframes connected to the query keyframe
     */
    {
        std::unique_lock<std::mutex> lock(databaseMutex);

        for (dbow2::bow_vector::BowVector::const_iterator
                 wordIt  = p_currentKeyFrame_in->bowVector.begin(),
                 wordEnd = p_currentKeyFrame_in->bowVector.end();
             wordIt != wordEnd;
             wordIt++)
        {
            std::list<keyframe::KeyFrame *> &keyFramesForWord =
                invertedFile[wordIt->first];

            for (std::list<keyframe::KeyFrame *>::iterator
                     keyFrameIt  = keyFramesForWord.begin(),
                     keyFrameEnd = keyFramesForWord.end();
                 keyFrameIt != keyFrameEnd;
                 keyFrameIt++)
            {
                keyframe::KeyFrame *p_candidateKeyFrame    = *keyFrameIt;
                map::Map      *p_candidateKeyFrameMap = nullptr;
                if (p_candidateKeyFrame->getMap(p_candidateKeyFrameMap) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                map::Map *p_currentKeyFrameMap = nullptr;
                if (p_currentKeyFrame_in->getMap(p_currentKeyFrameMap) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                /*!
                 * For consider a loop candidate it a candidate it must be in
                 * the same map
                 */
                if (p_candidateKeyFrameMap == p_currentKeyFrameMap)
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
                    map::Map *p_candidateKeyFrameMap2 = nullptr;
                    if (p_candidateKeyFrame->getMap(p_candidateKeyFrameMap2) !=
                        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getMap returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    bool isBad2{};
                    if (p_candidateKeyFrameMap2->isBad(isBad2) !=
                        map::MapStatus::MAP_STATUS_SUCCESS)
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
        std::list<std::pair<float, keyframe::KeyFrame *>> scoredCandidates;

        /* Only compare against those keyframes that share enough words */
        int maxCommonWordCount = 0;
        for (std::list<keyframe::KeyFrame *>::iterator
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

        /*!
         * Compute similarity score. Retain the matches whose score is higher
         * than minScore
         */
        for (std::list<keyframe::KeyFrame *>::iterator
                 keyFrameIt  = keyFramesSharingWordsLoop.begin(),
                 keyFrameEnd = keyFramesSharingWordsLoop.end();
             keyFrameIt != keyFrameEnd;
             keyFrameIt++)
        {
            keyframe::KeyFrame *p_candidateKeyFrame = *keyFrameIt;

            if (p_candidateKeyFrame->loopWords > minCommonWordCount)
            {
                scoredCandidateCount++;

                float candidateScore =
                    p_vocabulary->computeScore(p_currentKeyFrame_in->bowVector,
                                               p_candidateKeyFrame->bowVector);

                p_candidateKeyFrame->loopScore = candidateScore;
                if (candidateScore >= minScore_in)
                    scoredCandidates.push_back(
                        std::make_pair(candidateScore, p_candidateKeyFrame));
            }
        }

        if (!scoredCandidates.empty())
        {
            std::list<std::pair<float, keyframe::KeyFrame *>> accumulatedScoredCandidates;
            float bestAccumulatedScore = minScore_in;

            /* Lets now accumulate score by covisibility */
            for (std::list<std::pair<float, keyframe::KeyFrame *>>::iterator
                     scoredCandidateIt  = scoredCandidates.begin(),
                     scoredCandidateEnd = scoredCandidates.end();
                 scoredCandidateIt != scoredCandidateEnd;
                 scoredCandidateIt++)
            {
                keyframe::KeyFrame *p_candidateKeyFrame = scoredCandidateIt->second;
                std::vector<keyframe::KeyFrame *> covisibilityNeighborKeyFrames{};
                if (p_candidateKeyFrame->getBestCovisibilityKeyFrames(
                        10,
                        covisibilityNeighborKeyFrames) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getBestCovisibilityKeyFrames returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }

                float     bestGroupScore        = scoredCandidateIt->first;
                float     accumulatedScore      = scoredCandidateIt->first;
                keyframe::KeyFrame *p_bestScoringKeyFrame = p_candidateKeyFrame;
                for (std::vector<keyframe::KeyFrame *>::iterator
                         wordIt  = covisibilityNeighborKeyFrames.begin(),
                         wordEnd = covisibilityNeighborKeyFrames.end();
                     wordIt != wordEnd;
                     wordIt++)
                {
                    keyframe::KeyFrame *p_neighborKeyFrame = *wordIt;
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
                    std::make_pair(accumulatedScore, p_bestScoringKeyFrame));
                if (accumulatedScore > bestAccumulatedScore)
                    bestAccumulatedScore = accumulatedScore;
            }

            /*!
             * Return all those keyframes with a score higher than
             * 0.75*bestScore
             */
            float minScoreToRetain = 0.75f * bestAccumulatedScore;

            std::set<keyframe::KeyFrame *> alreadyAddedKeyFrames;
            loopCandidateKeyFrames_out.reserve(
                accumulatedScoredCandidates.size());

            for (std::list<std::pair<float, keyframe::KeyFrame *>>::iterator
                     scoredCandidateIt  = accumulatedScoredCandidates.begin(),
                     scoredCandidateEnd = accumulatedScoredCandidates.end();
                 scoredCandidateIt != scoredCandidateEnd;
                 scoredCandidateIt++)
            {
                if (scoredCandidateIt->first > minScoreToRetain)
                {
                    keyframe::KeyFrame *p_candidateKeyFrame = scoredCandidateIt->second;
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
        std::list<std::pair<float, keyframe::KeyFrame *>> scoredCandidates;

        /* Only compare against those keyframes that share enough words */
        int maxCommonWordCount = 0;
        for (std::list<keyframe::KeyFrame *>::iterator
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

        /*!
         * Compute similarity score. Retain the matches whose score is higher
         * than minScore
         */
        for (std::list<keyframe::KeyFrame *>::iterator
                 keyFrameIt  = keyFramesSharingWordsMerge.begin(),
                 keyFrameEnd = keyFramesSharingWordsMerge.end();
             keyFrameIt != keyFrameEnd;
             keyFrameIt++)
        {
            keyframe::KeyFrame *p_candidateKeyFrame = *keyFrameIt;

            if (p_candidateKeyFrame->mergeWords > minCommonWordCount)
            {
                scoredCandidateCount++;

                float candidateScore =
                    p_vocabulary->computeScore(p_currentKeyFrame_in->bowVector,
                                               p_candidateKeyFrame->bowVector);

                p_candidateKeyFrame->mergeScore = candidateScore;
                if (candidateScore >= minScore_in)
                    scoredCandidates.push_back(
                        std::make_pair(candidateScore, p_candidateKeyFrame));
            }
        }

        if (!scoredCandidates.empty())
        {
            std::list<std::pair<float, keyframe::KeyFrame *>> accumulatedScoredCandidates;
            float bestAccumulatedScore = minScore_in;

            /* Lets now accumulate score by covisibility */
            for (std::list<std::pair<float, keyframe::KeyFrame *>>::iterator
                     scoredCandidateIt  = scoredCandidates.begin(),
                     scoredCandidateEnd = scoredCandidates.end();
                 scoredCandidateIt != scoredCandidateEnd;
                 scoredCandidateIt++)
            {
                keyframe::KeyFrame *p_candidateKeyFrame = scoredCandidateIt->second;
                std::vector<keyframe::KeyFrame *> covisibilityNeighborKeyFrames{};
                if (p_candidateKeyFrame->getBestCovisibilityKeyFrames(
                        10,
                        covisibilityNeighborKeyFrames) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getBestCovisibilityKeyFrames returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }

                float     bestGroupScore        = scoredCandidateIt->first;
                float     accumulatedScore      = scoredCandidateIt->first;
                keyframe::KeyFrame *p_bestScoringKeyFrame = p_candidateKeyFrame;
                for (std::vector<keyframe::KeyFrame *>::iterator
                         wordIt  = covisibilityNeighborKeyFrames.begin(),
                         wordEnd = covisibilityNeighborKeyFrames.end();
                     wordIt != wordEnd;
                     wordIt++)
                {
                    keyframe::KeyFrame *p_neighborKeyFrame = *wordIt;
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
                    std::make_pair(accumulatedScore, p_bestScoringKeyFrame));
                if (accumulatedScore > bestAccumulatedScore)
                    bestAccumulatedScore = accumulatedScore;
            }

            /*!
             * Return all those keyframes with a score higher than
             * 0.75*bestScore
             */
            float minScoreToRetain = 0.75f * bestAccumulatedScore;

            std::set<keyframe::KeyFrame *> alreadyAddedKeyFrames;
            mergeCandidateKeyFrames_out.reserve(
                accumulatedScoredCandidates.size());

            for (std::list<std::pair<float, keyframe::KeyFrame *>>::iterator
                     scoredCandidateIt  = accumulatedScoredCandidates.begin(),
                     scoredCandidateEnd = accumulatedScoredCandidates.end();
                 scoredCandidateIt != scoredCandidateEnd;
                 scoredCandidateIt++)
            {
                if (scoredCandidateIt->first > minScoreToRetain)
                {
                    keyframe::KeyFrame *p_candidateKeyFrame = scoredCandidateIt->second;
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

    for (dbow2::bow_vector::BowVector::const_iterator
             wordIt  = p_currentKeyFrame_in->bowVector.begin(),
             wordEnd = p_currentKeyFrame_in->bowVector.end();
         wordIt != wordEnd;
         wordIt++)
    {
        std::list<keyframe::KeyFrame *> &keyFramesForWord = invertedFile[wordIt->first];

        for (std::list<keyframe::KeyFrame *>::iterator
                 keyFrameIt  = keyFramesForWord.begin(),
                 keyFrameEnd = keyFramesForWord.end();
             keyFrameIt != keyFrameEnd;
             keyFrameIt++)
        {
            keyframe::KeyFrame *p_candidateKeyFrame   = *keyFrameIt;
            p_candidateKeyFrame->loopQuery  = -1;
            p_candidateKeyFrame->mergeQuery = -1;
        }
    }

    return KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS;
}

} /* namespace keyframe_database */
} /* namespace core */
} /* namespace vs_graphs */

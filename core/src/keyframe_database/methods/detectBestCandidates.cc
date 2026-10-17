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
 * @file            detectBestCandidates.cc
 *
 * @brief           Implements KeyFrameDatabase::detectBestCandidates(),
 *                  declared in
 * keyframe_database/objects/KeyFrameDatabaseClass.h.
 */

#include "keyframe_database/objects/KeyFrameDatabaseClass.h"
#include "keyframe/objects/KeyFrameStatusEnum.h"
#include "map/map.h"

#include "dbow2/bow_vector/bow_vector.h"
#include "keyframe/keyframe.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace keyframe_database
{

KeyFrameDatabaseStatus KeyFrameDatabase::detectBestCandidates(
    keyframe::KeyFrame                *p_currentKeyFrame_in,
    std::vector<keyframe::KeyFrame *> &loopCandidateKeyFrames_out,
    std::vector<keyframe::KeyFrame *> &mergeCandidateKeyFrames_out,
    int                      minWordCount_in)
{
    std::list<keyframe::KeyFrame *> keyFramesSharingWords;
    std::set<keyframe::KeyFrame *>  connectedKeyFrames;

    /* Search all keyframes that share a word with current frame */
    {
        std::unique_lock<std::mutex> lock(databaseMutex);

        std::set<keyframe::KeyFrame *> currentKeyFrameConnectedKeyFrames{};
        if (p_currentKeyFrame_in->getConnectedKeyFrames(
                currentKeyFrameConnectedKeyFrames) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getConnectedKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        connectedKeyFrames = currentKeyFrameConnectedKeyFrames;

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
                keyframe::KeyFrame *p_candidateKeyFrame = *keyFrameIt;
                if (connectedKeyFrames.find(p_candidateKeyFrame) !=
                    connectedKeyFrames.end())
                {
                    continue;
                }
                if (p_candidateKeyFrame->placeRecognitionQuery !=
                    p_currentKeyFrame_in->id)
                {
                    p_candidateKeyFrame->placeRecognitionWords = 0;
                    p_candidateKeyFrame->placeRecognitionQuery =
                        p_currentKeyFrame_in->id;
                    keyFramesSharingWords.push_back(p_candidateKeyFrame);
                }
                p_candidateKeyFrame->placeRecognitionWords++;
            }
        }
    }
    if (keyFramesSharingWords.empty())
        return KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS;

    /* Only compare against those keyframes that share enough words */
    int maxCommonWordCount = 0;
    for (std::list<keyframe::KeyFrame *>::iterator
             keyFrameIt  = keyFramesSharingWords.begin(),
             keyFrameEnd = keyFramesSharingWords.end();
         keyFrameIt != keyFrameEnd;
         keyFrameIt++)
    {
        if ((*keyFrameIt)->placeRecognitionWords > maxCommonWordCount)
            maxCommonWordCount = (*keyFrameIt)->placeRecognitionWords;
    }

    int minCommonWordCount = maxCommonWordCount * 0.8f;

    if (minCommonWordCount < minWordCount_in)
    {
        minCommonWordCount = minWordCount_in;
    }

    std::list<std::pair<float, keyframe::KeyFrame *>> scoredCandidates;

    int scoredCandidateCount = 0;

    /* Compute similarity score. */
    for (std::list<keyframe::KeyFrame *>::iterator
             keyFrameIt  = keyFramesSharingWords.begin(),
             keyFrameEnd = keyFramesSharingWords.end();
         keyFrameIt != keyFrameEnd;
         keyFrameIt++)
    {
        keyframe::KeyFrame *p_candidateKeyFrame = *keyFrameIt;

        if (p_candidateKeyFrame->placeRecognitionWords > minCommonWordCount)
        {
            scoredCandidateCount++;
            float candidateScore =
                p_vocabulary->computeScore(p_currentKeyFrame_in->bowVector,
                                           p_candidateKeyFrame->bowVector);
            p_candidateKeyFrame->placeRecognitionScore = candidateScore;
            scoredCandidates.push_back(
                std::make_pair(candidateScore, p_candidateKeyFrame));
        }
    }

    if (scoredCandidates.empty())
        return KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS;

    std::list<std::pair<float, keyframe::KeyFrame *>> accumulatedScoredCandidates;
    float                                   bestAccumulatedScore = 0;

    /* Lets now accumulate score by covisibility */
    for (std::list<std::pair<float, keyframe::KeyFrame *>>::iterator
             scoredCandidateIt  = scoredCandidates.begin(),
             scoredCandidateEnd = scoredCandidates.end();
         scoredCandidateIt != scoredCandidateEnd;
         scoredCandidateIt++)
    {
        keyframe::KeyFrame               *p_candidateKeyFrame = scoredCandidateIt->second;
        std::vector<keyframe::KeyFrame *> covisibilityNeighborKeyFrames{};
        if (p_candidateKeyFrame->getBestCovisibilityKeyFrames(
                10,
                covisibilityNeighborKeyFrames) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getBestCovisibilityKeyFrames returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        float     bestGroupScore        = scoredCandidateIt->first;
        float     accumulatedScore      = bestGroupScore;
        keyframe::KeyFrame *p_bestScoringKeyFrame = p_candidateKeyFrame;
        for (std::vector<keyframe::KeyFrame *>::iterator
                 wordIt  = covisibilityNeighborKeyFrames.begin(),
                 wordEnd = covisibilityNeighborKeyFrames.end();
             wordIt != wordEnd;
             wordIt++)
        {
            keyframe::KeyFrame *p_neighborKeyFrame = *wordIt;
            if (p_neighborKeyFrame->placeRecognitionQuery !=
                p_currentKeyFrame_in->id)
                continue;

            accumulatedScore += p_neighborKeyFrame->placeRecognitionScore;
            if (p_neighborKeyFrame->placeRecognitionScore > bestGroupScore)
            {
                p_bestScoringKeyFrame = p_neighborKeyFrame;
                bestGroupScore = p_neighborKeyFrame->placeRecognitionScore;
            }
        }
        accumulatedScoredCandidates.push_back(
            std::make_pair(accumulatedScore, p_bestScoringKeyFrame));
        if (accumulatedScore > bestAccumulatedScore)
            bestAccumulatedScore = accumulatedScore;
    }

    /* Return all those keyframes with a score higher than 0.75*bestScore */
    float                minScoreToRetain = 0.75f * bestAccumulatedScore;
    std::set<keyframe::KeyFrame *> alreadyAddedKeyFrames;
    loopCandidateKeyFrames_out.reserve(accumulatedScoredCandidates.size());
    mergeCandidateKeyFrames_out.reserve(accumulatedScoredCandidates.size());
    for (std::list<std::pair<float, keyframe::KeyFrame *>>::iterator
             scoredCandidateIt  = accumulatedScoredCandidates.begin(),
             scoredCandidateEnd = accumulatedScoredCandidates.end();
         scoredCandidateIt != scoredCandidateEnd;
         scoredCandidateIt++)
    {
        const float &candidateScore = scoredCandidateIt->first;
        if (candidateScore > minScoreToRetain)
        {
            keyframe::KeyFrame *p_candidateKeyFrame = scoredCandidateIt->second;
            if (!alreadyAddedKeyFrames.count(p_candidateKeyFrame))
            {
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
                map::Map *p_candidateKeyFrameMap = nullptr;
                if (p_candidateKeyFrame->getMap(p_candidateKeyFrameMap) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (p_currentKeyFrameMap == p_candidateKeyFrameMap)
                {
                    loopCandidateKeyFrames_out.push_back(p_candidateKeyFrame);
                }
                else
                {
                    mergeCandidateKeyFrames_out.push_back(p_candidateKeyFrame);
                }
                alreadyAddedKeyFrames.insert(p_candidateKeyFrame);
            }
        }
    }

    return KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS;
}

} /* namespace keyframe_database */
} /* namespace core */
} /* namespace vs_graphs */

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
 * @file            detectRelocalizationCandidates.cc
 *
 * @brief           Implements
 *                  KeyFrameDatabase::detectRelocalizationCandidates(), declared
 *                  in KeyFrameDatabase.h.
 */

#include "place_recognition_and_loop_closure/keyframe_database/objects/KeyFrameDatabaseClass.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"

#include "dependency_lineage/dbow2/bow_vector/bow_vector.h"
#include "map_state/frame/frame.h"
#include "map_state/keyframe/keyframe.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace keyframe_database
{

KeyFrameDatabaseStatus KeyFrameDatabase::detectRelocalizationCandidates(
    frame::Frame                   *p_frame_in,
    map::Map                     *p_map_in,
    std::vector<keyframe::KeyFrame *> &relocalizationCandidates_out)
{
    std::list<keyframe::KeyFrame *> keyFramesSharingWords;

    /* Search all keyframes that share a word with current frame */
    {
        std::unique_lock<std::mutex> lock(databaseMutex);

        for (dbow2::bow_vector::BowVector::const_iterator
                 wordIt  = p_frame_in->bowVector.begin(),
                 wordEnd = p_frame_in->bowVector.end();
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
    {
        relocalizationCandidates_out = std::vector<keyframe::KeyFrame *>();
        return KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS;
    }

    /* Only compare against those keyframes that share enough words */
    int maxCommonWordCount = 0;
    for (std::list<keyframe::KeyFrame *>::iterator
             keyFrameIt  = keyFramesSharingWords.begin(),
             keyFrameEnd = keyFramesSharingWords.end();
         keyFrameIt != keyFrameEnd;
         keyFrameIt++)
    {
        if ((*keyFrameIt)->relocWords > maxCommonWordCount)
            maxCommonWordCount = (*keyFrameIt)->relocWords;
    }

    int minCommonWordCount = maxCommonWordCount * 0.8f;

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

        if (p_candidateKeyFrame->relocWords > minCommonWordCount)
        {
            scoredCandidateCount++;
            float candidateScore =
                p_vocabulary->computeScore(p_frame_in->bowVector,
                                           p_candidateKeyFrame->bowVector);
            p_candidateKeyFrame->relocScore = candidateScore;
            scoredCandidates.push_back(
                std::make_pair(candidateScore, p_candidateKeyFrame));
        }
    }

    if (scoredCandidates.empty())
    {
        relocalizationCandidates_out = std::vector<keyframe::KeyFrame *>();
        return KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS;
    }

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
            std::make_pair(accumulatedScore, p_bestScoringKeyFrame));
        if (accumulatedScore > bestAccumulatedScore)
            bestAccumulatedScore = accumulatedScore;
    }

    /* Return all those keyframes with a score higher than 0.75*bestScore */
    float                   minScoreToRetain = 0.75f * bestAccumulatedScore;
    std::set<keyframe::KeyFrame *>    alreadyAddedKeyFrames;
    std::vector<keyframe::KeyFrame *> relocalizationCandidateKeyFrames;
    relocalizationCandidateKeyFrames.reserve(
        accumulatedScoredCandidates.size());
    for (std::list<std::pair<float, keyframe::KeyFrame *>>::iterator
             scoredCandidateIt  = accumulatedScoredCandidates.begin(),
             scoredCandidateEnd = accumulatedScoredCandidates.end();
         scoredCandidateIt != scoredCandidateEnd;
         scoredCandidateIt++)
    {
        const float &candidateScore = scoredCandidateIt->first;
        if (candidateScore > minScoreToRetain)
        {
            keyframe::KeyFrame *p_candidateKeyFrame    = scoredCandidateIt->second;
            map::Map      *p_candidateKeyFrameMap = nullptr;
            if (p_candidateKeyFrame->getMap(p_candidateKeyFrameMap) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_candidateKeyFrameMap != p_map_in)
                continue;
            if (!alreadyAddedKeyFrames.count(p_candidateKeyFrame))
            {
                relocalizationCandidateKeyFrames.push_back(p_candidateKeyFrame);
                alreadyAddedKeyFrames.insert(p_candidateKeyFrame);
            }
        }
    }

    relocalizationCandidates_out = relocalizationCandidateKeyFrames;
    return KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS;
}

} /* namespace keyframe_database */
} /* namespace core */
} /* namespace vs_graphs */

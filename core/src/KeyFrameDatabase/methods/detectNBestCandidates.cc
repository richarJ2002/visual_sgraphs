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
#include <rclcpp/logging.hpp>

#include "../private_functions.h"
#include "Map.h"

using namespace std;

namespace vs_graphs
{
namespace core
{

void KeyFrameDatabase::detectNBestCandidates(
    KeyFrame           *p_currentKeyFrame_in,
    vector<KeyFrame *> &loopCandidateKeyFrames_out,
    vector<KeyFrame *> &mergeCandidateKeyFrames_out,
    int                 candidateCount_in)
{
    list<KeyFrame *> keyFramesSharingWords;
    set<KeyFrame *>  connectedKeyFrames;

    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(databaseMutex);

        std::set<KeyFrame *> currentKeyFrameConnectedKeyFrames{};
        if (p_currentKeyFrame_in->getConnectedKeyFrames(
                currentKeyFrameConnectedKeyFrames) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getConnectedKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        connectedKeyFrames = currentKeyFrameConnectedKeyFrames;

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

                if (p_candidateKeyFrame->placeRecognitionQuery !=
                    p_currentKeyFrame_in->id)
                {
                    p_candidateKeyFrame->placeRecognitionWords = 0;
                    if (!connectedKeyFrames.count(p_candidateKeyFrame))
                    {

                        p_candidateKeyFrame->placeRecognitionQuery =
                            p_currentKeyFrame_in->id;
                        keyFramesSharingWords.push_back(p_candidateKeyFrame);
                    }
                }
                p_candidateKeyFrame->placeRecognitionWords++;
            }
        }
    }
    if (keyFramesSharingWords.empty())
        return;

    // Only compare against those keyframes that share enough words
    int maxCommonWordCount = 0;
    for (list<KeyFrame *>::iterator keyFrameIt  = keyFramesSharingWords.begin(),
                                    keyFrameEnd = keyFramesSharingWords.end();
         keyFrameIt != keyFrameEnd;
         keyFrameIt++)
    {
        if ((*keyFrameIt)->placeRecognitionWords > maxCommonWordCount)
            maxCommonWordCount = (*keyFrameIt)->placeRecognitionWords;
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

        if (p_candidateKeyFrame->placeRecognitionWords > minCommonWordCount)
        {
            scoredCandidateCount++;
            float candidateScore =
                p_vocabulary->score(p_currentKeyFrame_in->bowVector,
                                    p_candidateKeyFrame->bowVector);
            p_candidateKeyFrame->placeRecognitionScore = candidateScore;
            scoredCandidates.push_back(
                make_pair(candidateScore, p_candidateKeyFrame));
        }
    }

    if (scoredCandidates.empty())
        return;

    list<pair<float, KeyFrame *>> accumulatedScoredCandidates;
    float                         bestAccumulatedScore = 0;

    // Lets now accumulate score by covisibility
    for (list<pair<float, KeyFrame *>>::iterator
             scoredCandidateIt  = scoredCandidates.begin(),
             scoredCandidateEnd = scoredCandidates.end();
         scoredCandidateIt != scoredCandidateEnd;
         scoredCandidateIt++)
    {
        KeyFrame               *p_candidateKeyFrame = scoredCandidateIt->second;
        std::vector<KeyFrame *> covisibilityNeighborKeyFrames{};
        if (p_candidateKeyFrame->getBestCovisibilityKeyFrames(
                10,
                covisibilityNeighborKeyFrames) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getBestCovisibilityKeyFrames returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

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
            make_pair(accumulatedScore, p_bestScoringKeyFrame));
        if (accumulatedScore > bestAccumulatedScore)
            bestAccumulatedScore = accumulatedScore;
    }

    accumulatedScoredCandidates.sort(compFirst);

    loopCandidateKeyFrames_out.reserve(candidateCount_in);
    mergeCandidateKeyFrames_out.reserve(candidateCount_in);
    set<KeyFrame *>                         alreadyAddedKeyFrames;
    std::size_t                             candidateIndex = 0;
    list<pair<float, KeyFrame *>>::iterator scoredCandidateIt =
        accumulatedScoredCandidates.begin();
    // reserve() above already rejects a negative request, so nNumCandidates is
    // a non-negative candidate budget by the time it is used as a size here.
    const std::size_t candidateBudget =
        static_cast<std::size_t>(candidateCount_in);
    while (candidateIndex < accumulatedScoredCandidates.size() &&
           (loopCandidateKeyFrames_out.size() < candidateBudget ||
            mergeCandidateKeyFrames_out.size() < candidateBudget))
    {
        KeyFrame *p_candidateKeyFrame = scoredCandidateIt->second;
        bool      candidateKeyFrameIsBad{};
        if (p_candidateKeyFrame->isBad(candidateKeyFrameIsBad) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (candidateKeyFrameIsBad)
        {
            candidateIndex++;
            scoredCandidateIt++;
            continue;
        }

        if (!alreadyAddedKeyFrames.count(p_candidateKeyFrame))
        {
            Map *p_currentKeyFrameMap = nullptr;
            if (p_currentKeyFrame_in->getMap(p_currentKeyFrameMap) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            Map *p_candidateKeyFrameMap = nullptr;
            if (p_candidateKeyFrame->getMap(p_candidateKeyFrameMap) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_currentKeyFrameMap == p_candidateKeyFrameMap &&
                loopCandidateKeyFrames_out.size() < candidateBudget)
            {
                loopCandidateKeyFrames_out.push_back(p_candidateKeyFrame);
            }
            else
            {
                Map *p_currentKeyFrameMap2 = nullptr;
                if (p_currentKeyFrame_in->getMap(p_currentKeyFrameMap2) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
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
                Map *p_candidateKeyFrameMap3 = nullptr;
                if ((p_currentKeyFrameMap2 != p_candidateKeyFrameMap2 &&
                     mergeCandidateKeyFrames_out.size() < candidateBudget) &&
                    p_candidateKeyFrame->getMap(p_candidateKeyFrameMap3) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                bool isBad2{};
                if ((p_currentKeyFrameMap2 != p_candidateKeyFrameMap2 &&
                     mergeCandidateKeyFrames_out.size() < candidateBudget) &&
                    p_candidateKeyFrameMap3->isBad(isBad2) !=
                        MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_currentKeyFrameMap2 != p_candidateKeyFrameMap2 &&
                    mergeCandidateKeyFrames_out.size() < candidateBudget &&
                    !isBad2)
                {
                    mergeCandidateKeyFrames_out.push_back(p_candidateKeyFrame);
                }
            }
            alreadyAddedKeyFrames.insert(p_candidateKeyFrame);
        }
        candidateIndex++;
        scoredCandidateIt++;
    }
}

} // namespace core
} // namespace vs_graphs

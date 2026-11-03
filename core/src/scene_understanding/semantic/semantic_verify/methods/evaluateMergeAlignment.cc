/*!
 * @file            evaluateMergeAlignment.cc
 *
 * @brief           Implements SemanticVerify::evaluateMergeAlignment(),
 *                  declared in semantic_verify/objects/SemanticVerifyClass.h.
 */

#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticVerifyStatus SemanticVerify::evaluateMergeAlignment(
    const std::vector<SemanticMergeRoomEvidence> &survivingRooms_in,
    const std::vector<SemanticMergeRoomEvidence> &absorbedRooms_in,
    const g2o::Sim3            &mergeTransform_absorbedToSurviving_in,
    SemanticMergeGateResult    &result_out,
    const SemanticVerifyConfig &configuration_in)
{
    SemanticMergeGateResult result;

    const double          scale = mergeTransform_absorbedToSurviving_in.scale();
    const Eigen::Matrix3d rotation_absorbedToSurviving =
        mergeTransform_absorbedToSurviving_in.rotation()
            .toRotationMatrix()
            .cast<double>();
    const Eigen::Vector3d translation_absorbedToSurviving_m =
        mergeTransform_absorbedToSurviving_in.translation().cast<double>();
    if (!std::isfinite(scale) || scale <= 0.0 ||
        !rotation_absorbedToSurviving.allFinite() ||
        !translation_absorbedToSurviving_m.allFinite())
    {
        result.decision   = SemanticMergeDecision::REJECT;
        result.reason     = SemanticMergeReason::INVALID_INPUT;
        result.pairDetail = "transform=invalid";
        result_out        = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    /* Every comparison below happens in the surviving map's world frame. */
    std::vector<SemanticMergeRoomEvidence> absorbedRooms;
    absorbedRooms.reserve(absorbedRooms_in.size());
    for (const SemanticMergeRoomEvidence &absorbedRoom : absorbedRooms_in)
    {
        absorbedRooms.push_back(transformRoomEvidenceBetweenMaps(
            absorbedRoom,
            mergeTransform_absorbedToSurviving_in));
    }

    /* The anchor rooms are the only rooms matched by identity (their tag). */
    const std::vector<MergePair> anchorPairs =
        findAnchorRoomPairs(survivingRooms_in, absorbedRooms);
    result.sharedRoomCount = anchorPairs.size();
    if (anchorPairs.empty())
    {
        result.decision   = SemanticMergeDecision::DEFER;
        result.reason     = SemanticMergeReason::SHARED_ROOM_IDENTITY_MISSING;
        result.pairDetail = "anchors=none";
        result_out        = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    std::vector<int> absorbedRoomIds;
    for (const SemanticMergeRoomEvidence &absorbedRoom : absorbedRooms)
    {
        absorbedRoomIds.push_back(absorbedRoom.context.roomId);
    }
    std::vector<int> survivingRoomIds;
    for (const SemanticMergeRoomEvidence &survivingRoom : survivingRooms_in)
    {
        survivingRoomIds.push_back(survivingRoom.context.roomId);
    }

    const std::vector<MergePair> floorPairs =
        pairFloors(absorbedRooms, survivingRooms_in, configuration_in);
    const std::vector<RoomPair> roomPairs =
        pairRooms(absorbedRooms, survivingRooms_in, configuration_in);

    /* A contradiction rejects, missing evidence defers; the first one found
     * of each kind names the reason. */
    bool                hasContradiction    = false;
    SemanticMergeReason contradictionReason = SemanticMergeReason::ALIGNED;
    bool                hasMissingEvidence  = false;
    SemanticMergeReason missingReason       = SemanticMergeReason::ALIGNED;
    std::string         problemText;
    const auto          recordContradiction =
        [&](const SemanticMergeReason reason_in, const std::string &text_in)
    {
        if (!hasContradiction)
        {
            hasContradiction    = true;
            contradictionReason = reason_in;
        }
        problemText += " " + text_in;
    };
    const auto recordMissingEvidence =
        [&](const SemanticMergeReason reason_in, const std::string &text_in)
    {
        if (!hasMissingEvidence)
        {
            hasMissingEvidence = true;
            missingReason      = reason_in;
        }
        problemText += " " + text_in;
    };

    std::ostringstream     anchorText;
    std::ostringstream     anchorWallText;
    std::vector<MergePair> allRoomPairs;
    for (const MergePair &anchorPair : anchorPairs)
    {
        const SemanticMergeRoomEvidence &survivingRoom =
            survivingRooms_in[anchorPair.survivingIndex];
        const SemanticMergeRoomEvidence &absorbedRoom =
            absorbedRooms[anchorPair.absorbedIndex];
        bool isAnchorAligned = true;
        allRoomPairs.push_back(anchorPair);
        anchorText << (anchorText.tellp() == 0 ? "" : ",")
                   << survivingRoom.context.roomTag << ":a"
                   << absorbedRoom.context.roomId << "~s"
                   << survivingRoom.context.roomId;

        /* Floor: both anchor floors must exist and be coplanar. */
        if (!absorbedRoom.floorPlane_world.has_value() ||
            !survivingRoom.floorPlane_world.has_value())
        {
            recordMissingEvidence(SemanticMergeReason::FLOOR_EVIDENCE_MISSING,
                                  "floor_missing");
            isAnchorAligned = false;
        }
        else if (std::none_of(floorPairs.begin(),
                              floorPairs.end(),
                              [&anchorPair](const MergePair &floorPair)
                              {
                                  return floorPair.absorbedIndex ==
                                             anchorPair.absorbedIndex &&
                                         floorPair.survivingIndex ==
                                             anchorPair.survivingIndex;
                              }))
        {
            recordContradiction(SemanticMergeReason::FLOOR_CONTRADICTION,
                                "floor_not_coplanar");
            isAnchorAligned = false;
        }

        /* Walls: they must pair, without contradiction, in enough directions
         * to fix the translation. Walls seen on one side only give no
         * answer and are not counted against the pairing. */
        const MergePairing wallPairing = pairWalls(absorbedRoom.walls,
                                                   survivingRoom.walls,
                                                   configuration_in);
        std::vector<int>   absorbedWallIds;
        for (const VerifyWallObservation &wall : absorbedRoom.walls)
        {
            absorbedWallIds.push_back(wall.wallId);
        }
        std::vector<int> survivingWallIds;
        for (const VerifyWallObservation &wall : survivingRoom.walls)
        {
            survivingWallIds.push_back(wall.wallId);
        }
        result.matchedWallCount += wallPairing.pairs.size();
        anchorWallText << (anchorWallText.tellp() == 0 ? "" : ",")
                       << formatMergePairs(wallPairing.pairs,
                                           absorbedWallIds,
                                           survivingWallIds);
        if (!wallPairing.contradictions.empty())
        {
            recordContradiction(
                SemanticMergeReason::WALL_ALIGNMENT_CONTRADICTION,
                "wall_offset=" + formatMergePairs(wallPairing.contradictions,
                                                  absorbedWallIds,
                                                  survivingWallIds));
            isAnchorAligned = false;
        }
        if (hasRepeatedElement(wallPairing.pairs))
        {
            recordContradiction(SemanticMergeReason::PAIRING_CONTRADICTION,
                                "wall_paired_twice");
            isAnchorAligned = false;
        }
        const double wallSpread_deg =
            measureWallNormalSpread(survivingRoom.walls, wallPairing.pairs);
        if (wallPairing.pairs.size() < configuration_in.anchorMinWallPairs ||
            wallSpread_deg < configuration_in.anchorMinNormalSpread_deg)
        {
            std::ostringstream shortage;
            shortage << "anchor_walls=" << wallPairing.pairs.size() << "/"
                     << configuration_in.anchorMinWallPairs
                     << " spread_deg=" << wallSpread_deg;
            recordMissingEvidence(SemanticMergeReason::WALL_EVIDENCE_MISSING,
                                  shortage.str());
            isAnchorAligned = false;
        }
        if (isAnchorAligned)
        {
            result.alignedRoomCount++;
        }
    }

    /* Rooms beyond the anchors pair through their walls. A room that pairs
     * with two rooms, or two rooms that pair with one, contradict. */
    std::ostringstream otherWallText;
    for (const RoomPair &roomPair : roomPairs)
    {
        const bool isAnchor = std::any_of(
            anchorPairs.begin(),
            anchorPairs.end(),
            [&roomPair](const MergePair &anchorPair)
            {
                return anchorPair.absorbedIndex == roomPair.absorbedIndex &&
                       anchorPair.survivingIndex == roomPair.survivingIndex;
            });
        if (isAnchor)
        {
            continue;
        }
        allRoomPairs.push_back(
            {roomPair.absorbedIndex, roomPair.survivingIndex});
        result.matchedWallCount += roomPair.wallPairs.size();

        std::vector<int> absorbedWallIds;
        for (const VerifyWallObservation &wall :
             absorbedRooms[roomPair.absorbedIndex].walls)
        {
            absorbedWallIds.push_back(wall.wallId);
        }
        std::vector<int> survivingWallIds;
        for (const VerifyWallObservation &wall :
             survivingRooms_in[roomPair.survivingIndex].walls)
        {
            survivingWallIds.push_back(wall.wallId);
        }
        otherWallText << (otherWallText.tellp() == 0 ? "" : ",")
                      << formatMergePairs(roomPair.wallPairs,
                                          absorbedWallIds,
                                          survivingWallIds);
    }
    if (hasRepeatedElement(allRoomPairs))
    {
        recordContradiction(SemanticMergeReason::PAIRING_CONTRADICTION,
                            "room_paired_twice");
    }

    /* Passages pair across the whole overlap. */
    const std::vector<PassageContext> absorbedPassages =
        collectUniquePassages(absorbedRooms);
    const std::vector<PassageContext> survivingPassages =
        collectUniquePassages(survivingRooms_in);
    const MergePairing passagePairing =
        pairPassages(absorbedPassages, survivingPassages, configuration_in);
    std::vector<int> absorbedPassageIds;
    for (const PassageContext &passage : absorbedPassages)
    {
        absorbedPassageIds.push_back(passage.id);
    }
    std::vector<int> survivingPassageIds;
    for (const PassageContext &passage : survivingPassages)
    {
        survivingPassageIds.push_back(passage.id);
    }
    result.matchedPassageCount = passagePairing.pairs.size();
    if (hasRepeatedElement(passagePairing.pairs))
    {
        recordContradiction(SemanticMergeReason::PAIRING_CONTRADICTION,
                            "passage_paired_twice");
    }
    if (!passagePairing.contradictions.empty())
    {
        recordContradiction(SemanticMergeReason::PASSAGE_STATE_CONTRADICTION,
                            "passage_state=" +
                                formatMergePairs(passagePairing.contradictions,
                                                 absorbedPassageIds,
                                                 survivingPassageIds));
    }

    std::ostringstream detail;
    detail << "anchors=" << anchorText.str()
           << " anchor_walls=" << anchorWallText.str() << " other_walls="
           << (otherWallText.tellp() == 0 ? "-" : otherWallText.str())
           << " floors="
           << formatMergePairs(floorPairs, absorbedRoomIds, survivingRoomIds)
           << " rooms="
           << formatMergePairs(allRoomPairs, absorbedRoomIds, survivingRoomIds)
           << " passages="
           << formatMergePairs(passagePairing.pairs,
                               absorbedPassageIds,
                               survivingPassageIds)
           << " problems="
           << (problemText.empty() ? "none" : problemText.substr(1));
    result.pairDetail = detail.str();

    if (hasContradiction)
    {
        result.decision = SemanticMergeDecision::REJECT;
        result.reason   = contradictionReason;
    }
    else if (hasMissingEvidence)
    {
        result.decision = SemanticMergeDecision::DEFER;
        result.reason   = missingReason;
    }
    else
    {
        result.decision = SemanticMergeDecision::ACCEPT;
        result.reason   = SemanticMergeReason::ALIGNED;
    }
    result_out = result;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

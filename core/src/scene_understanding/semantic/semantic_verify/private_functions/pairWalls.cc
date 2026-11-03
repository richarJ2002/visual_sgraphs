/*!
 * @file            pairWalls.cc
 *
 * @brief           Implements pairWalls(), declared in private_functions.h.
 */

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

MergePairing pairWalls(
    const std::vector<VerifyWallObservation> &absorbedWalls_survivingWorld_in,
    const std::vector<VerifyWallObservation> &survivingWalls_in,
    const SemanticVerifyConfig               &configuration_in)
{
    MergePairing           pairing;
    std::vector<MergePair> offsetMismatches;
    std::vector<bool> isAbsorbedPaired(absorbedWalls_survivingWorld_in.size(),
                                       false);
    std::vector<bool> isSurvivingPaired(survivingWalls_in.size(), false);

    for (std::size_t absorbedIndex = 0U;
         absorbedIndex < absorbedWalls_survivingWorld_in.size();
         absorbedIndex++)
    {
        for (std::size_t survivingIndex = 0U;
             survivingIndex < survivingWalls_in.size();
             survivingIndex++)
        {
            const WallRelation relation = classifyWallRelation(
                absorbedWalls_survivingWorld_in[absorbedIndex],
                survivingWalls_in[survivingIndex],
                configuration_in);
            if (relation == WallRelation::PAIRED)
            {
                pairing.pairs.push_back({absorbedIndex, survivingIndex});
                isAbsorbedPaired[absorbedIndex]   = true;
                isSurvivingPaired[survivingIndex] = true;
            }
            else if (relation == WallRelation::OFFSET_MISMATCH)
            {
                offsetMismatches.push_back({absorbedIndex, survivingIndex});
            }
        }
    }

    /* A mismatch only contradicts when neither wall found an agreeing
     * partner; a duplicate plane beside a paired wall is not evidence. */
    for (const MergePair &mismatch : offsetMismatches)
    {
        if (!isAbsorbedPaired[mismatch.absorbedIndex] &&
            !isSurvivingPaired[mismatch.survivingIndex])
        {
            pairing.contradictions.push_back(mismatch);
        }
    }
    return pairing;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

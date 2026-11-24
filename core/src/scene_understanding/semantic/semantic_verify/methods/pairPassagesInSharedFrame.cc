/*!
 * @file            pairPassagesInSharedFrame.cc
 *
 * @brief           Implements SemanticVerify::pairPassagesInSharedFrame(),
 *                  declared in semantic_verify/objects/SemanticVerifyClass.h.
 */

#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

#include <limits>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

std::vector<std::pair<Passage *, Passage *>>
    SemanticVerify::pairPassagesInSharedFrame(
        const std::vector<Passage *> &importedPassages_in,
        const std::vector<Passage *> &retainedPassages_in,
        const SemanticVerifyConfig   &configuration_in)
{
    std::vector<Passage *>      importedAdmittedPassages{};
    std::vector<Passage *>      retainedAdmittedPassages{};
    std::vector<PassageContext> importedContexts;
    std::vector<PassageContext> retainedContexts;
    collectPassageContexts(importedPassages_in,
                           importedAdmittedPassages,
                           importedContexts);
    collectPassageContexts(retainedPassages_in,
                           retainedAdmittedPassages,
                           retainedContexts);

    /* Both lists already live in one frame, so no transform is applied. */
    const MergePairing pairing =
        pairPassages(importedContexts, retainedContexts, configuration_in);

    std::vector<std::pair<Passage *, Passage *>> duplicatePairs;
    for (std::size_t importedIndex = 0U;
         importedIndex < importedAdmittedPassages.size();
         importedIndex++)
    {
        /* The nearest centre wins when several passages lie in range. */
        std::size_t bestRetainedIndex = retainedAdmittedPassages.size();
        double      bestDistance_m    = std::numeric_limits<double>::infinity();
        for (const MergePair &passagePair : pairing.pairs)
        {
            if (passagePair.absorbedIndex != importedIndex)
            {
                continue;
            }
            const double centreDistance_m =
                (importedContexts[importedIndex].passageCentroid_world -
                 retainedContexts[passagePair.survivingIndex]
                     .passageCentroid_world)
                    .norm();
            if (centreDistance_m < bestDistance_m)
            {
                bestDistance_m    = centreDistance_m;
                bestRetainedIndex = passagePair.survivingIndex;
            }
        }
        if (bestRetainedIndex < retainedAdmittedPassages.size() &&
            retainedAdmittedPassages[bestRetainedIndex] !=
                importedAdmittedPassages[importedIndex])
        {
            duplicatePairs.push_back(
                {importedAdmittedPassages[importedIndex],
                 retainedAdmittedPassages[bestRetainedIndex]});
        }
    }
    return duplicatePairs;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

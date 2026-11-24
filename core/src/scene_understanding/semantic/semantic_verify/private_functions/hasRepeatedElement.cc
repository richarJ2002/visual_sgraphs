/*!
 * @file            hasRepeatedElement.cc
 *
 * @brief           Implements hasRepeatedElement(), declared in
 * private_functions.h.
 */

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

#include <set>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

bool hasRepeatedElement(const std::vector<MergePair> &pairs_in)
{
    std::set<std::size_t> absorbedIndices;
    std::set<std::size_t> survivingIndices;
    for (const MergePair &pair : pairs_in)
    {
        if (!absorbedIndices.insert(pair.absorbedIndex).second ||
            !survivingIndices.insert(pair.survivingIndex).second)
        {
            return true;
        }
    }
    return false;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

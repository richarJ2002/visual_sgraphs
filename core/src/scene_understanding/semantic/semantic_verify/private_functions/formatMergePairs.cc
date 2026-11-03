/*!
 * @file            formatMergePairs.cc
 *
 * @brief           Implements formatMergePairs(), declared in
 * private_functions.h.
 */

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

#include <sstream>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

std::string formatMergePairs(const std::vector<MergePair> &pairs_in,
                             const std::vector<int>       &absorbedIds_in,
                             const std::vector<int>       &survivingIds_in)
{
    if (pairs_in.empty())
    {
        return "-";
    }
    std::ostringstream text;
    for (std::size_t pairIndex = 0U; pairIndex < pairs_in.size(); pairIndex++)
    {
        text << (pairIndex == 0U ? "" : ",") << "a"
             << absorbedIds_in[pairs_in[pairIndex].absorbedIndex] << "~s"
             << survivingIds_in[pairs_in[pairIndex].survivingIndex];
    }
    return text.str();
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

/*!
 * @file            measureWallNormalSpread.cc
 *
 * @brief           Implements measureWallNormalSpread(), declared in
 * private_functions.h.
 */

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

#include <algorithm>
#include <cmath>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

double measureWallNormalSpread(
    const std::vector<VerifyWallObservation> &survivingWalls_in,
    const std::vector<MergePair>             &wallPairs_in)
{
    double widestAngle_deg = 0.0;
    for (const MergePair &firstPair : wallPairs_in)
    {
        for (const MergePair &secondPair : wallPairs_in)
        {
            /* Normals count as lines: parallel and opposite are the same
             * direction for fixing a translation. */
            const double lineCosine =
                std::abs(survivingWalls_in[firstPair.survivingIndex]
                             .wallNormal_world.normalized()
                             .dot(survivingWalls_in[secondPair.survivingIndex]
                                      .wallNormal_world.normalized()));
            widestAngle_deg =
                std::max(widestAngle_deg,
                         std::acos(std::clamp(lineCosine, 0.0, 1.0)) *
                             DEGREES_PER_RADIAN);
        }
    }
    return widestAngle_deg;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

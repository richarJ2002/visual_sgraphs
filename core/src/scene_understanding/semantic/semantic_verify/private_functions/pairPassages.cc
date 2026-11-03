/*!
 * @file            pairPassages.cc
 *
 * @brief           Implements pairPassages(), declared in private_functions.h.
 */

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

#include <algorithm>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

MergePairing pairPassages(
    const std::vector<PassageContext> &absorbedPassages_survivingWorld_in,
    const std::vector<PassageContext> &survivingPassages_in,
    const SemanticVerifyConfig        &configuration_in)
{
    MergePairing pairing;
    for (std::size_t absorbedIndex = 0U;
         absorbedIndex < absorbedPassages_survivingWorld_in.size();
         absorbedIndex++)
    {
        const PassageContext &absorbedPassage =
            absorbedPassages_survivingWorld_in[absorbedIndex];
        for (std::size_t survivingIndex = 0U;
             survivingIndex < survivingPassages_in.size();
             survivingIndex++)
        {
            const PassageContext &survivingPassage =
                survivingPassages_in[survivingIndex];

            /* A recovery proxy is historical topology, not a measured
             * opening, so it has no geometry to pair. */
            if (!absorbedPassage.hasPlane || !survivingPassage.hasPlane ||
                !absorbedPassage.isApertureValid ||
                !survivingPassage.isApertureValid ||
                absorbedPassage.isRecoveryProxy ||
                survivingPassage.isRecoveryProxy)
            {
                continue;
            }

            const PlaneGap planeGap =
                measurePlaneGap(absorbedPassage.planeEquation_world,
                                survivingPassage.planeEquation_world,
                                absorbedPassage.passageCentroid_world,
                                false);
            if (!(planeGap.normalAngle_deg <=
                  configuration_in.coplanarAngle_deg) ||
                !(planeGap.offset_m <= configuration_in.wallCoplanarOffset_m))
            {
                continue;
            }

            /* Distance between the centres inside the shared plane: the
             * component along the plane normal is removed, so the test is a
             * disc in the wall, not a sphere. */
            const Eigen::Vector3d planeNormal_world =
                survivingPassage.planeEquation_world.head<3>().normalized();
            const Eigen::Vector3d centreOffset_world =
                absorbedPassage.passageCentroid_world -
                survivingPassage.passageCentroid_world;
            const double inPlaneDistance_m =
                (centreOffset_world -
                 centreOffset_world.dot(planeNormal_world) * planeNormal_world)
                    .norm();
            const double halfNarrowerWidth_m =
                0.5 *
                std::min(absorbedPassage.width_m, survivingPassage.width_m);
            if (!(inPlaneDistance_m <= halfNarrowerWidth_m))
            {
                continue;
            }

            pairing.pairs.push_back({absorbedIndex, survivingIndex});
            if (absorbedPassage.isPassable != survivingPassage.isPassable)
            {
                pairing.contradictions.push_back(
                    {absorbedIndex, survivingIndex});
            }
        }
    }
    return pairing;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

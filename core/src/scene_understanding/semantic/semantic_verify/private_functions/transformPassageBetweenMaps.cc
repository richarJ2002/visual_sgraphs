/*!
 * @file            transformPassageBetweenMaps.cc
 *
 * @brief           Implements transformPassageBetweenMaps(), declared in
 * private_functions.h.
 */

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

PassageContext transformPassageBetweenMaps(
    const PassageContext &passage_absorbedWorld_in,
    const g2o::Sim3      &mergeTransform_absorbedToSurviving_in)
{
    PassageContext passage_survivingWorld = passage_absorbedWorld_in;
    passage_survivingWorld.passageCentroid_world =
        mergeTransform_absorbedToSurviving_in.map(
            passage_absorbedWorld_in.passageCentroid_world);
    if (passage_absorbedWorld_in.hasPlane)
    {
        passage_survivingWorld.planeEquation_world = transformPlaneBetweenMaps(
            passage_absorbedWorld_in.planeEquation_world,
            mergeTransform_absorbedToSurviving_in);
    }
    if (passage_absorbedWorld_in.hasKnownSideDirection)
    {
        passage_survivingWorld.knownSideDirection_world =
            mergeTransform_absorbedToSurviving_in.rotation()
                .toRotationMatrix() *
            passage_absorbedWorld_in.knownSideDirection_world;
    }
    passage_survivingWorld.width_m =
        mergeTransform_absorbedToSurviving_in.scale() *
        passage_absorbedWorld_in.width_m;
    passage_survivingWorld.height_m =
        mergeTransform_absorbedToSurviving_in.scale() *
        passage_absorbedWorld_in.height_m;
    return passage_survivingWorld;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

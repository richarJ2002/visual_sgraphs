/*!
 * @file            transformWallBetweenMaps.cc
 *
 * @brief           Implements transformWallBetweenMaps(), declared in
 * private_functions.h.
 */

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

VerifyWallObservation transformWallBetweenMaps(
    const VerifyWallObservation &wall_absorbedWorld_in,
    const g2o::Sim3             &mergeTransform_absorbedToSurviving_in)
{
    VerifyWallObservation wall_survivingWorld = wall_absorbedWorld_in;

    Eigen::Vector4d wallPlane_absorbedWorld;
    wallPlane_absorbedWorld << wall_absorbedWorld_in.wallNormal_world,
        wall_absorbedWorld_in.d;
    const Eigen::Vector4d wallPlane_survivingWorld =
        transformPlaneBetweenMaps(wallPlane_absorbedWorld,
                                  mergeTransform_absorbedToSurviving_in);
    wall_survivingWorld.wallNormal_world = wallPlane_survivingWorld.head<3>();
    wall_survivingWorld.d                = wallPlane_survivingWorld(3);
    wall_survivingWorld.wallCentroid_world =
        mergeTransform_absorbedToSurviving_in.map(
            wall_absorbedWorld_in.wallCentroid_world);
    for (Eigen::Vector3d &supportPoint_world :
         wall_survivingWorld.supportSample_world)
    {
        supportPoint_world =
            mergeTransform_absorbedToSurviving_in.map(supportPoint_world);
    }
    return wall_survivingWorld;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

/*!
 * @file            classifyWallRelation.cc
 *
 * @brief           Implements classifyWallRelation(), declared in
 * private_functions.h.
 */

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

WallRelation classifyWallRelation(
    const VerifyWallObservation &absorbedWall_survivingWorld_in,
    const VerifyWallObservation &survivingWall_in,
    const SemanticVerifyConfig  &configuration_in)
{
    Eigen::Vector4d absorbedPlane_world;
    absorbedPlane_world << absorbedWall_survivingWorld_in.wallNormal_world,
        absorbedWall_survivingWorld_in.d;
    Eigen::Vector4d survivingPlane_world;
    survivingPlane_world << survivingWall_in.wallNormal_world,
        survivingWall_in.d;

    /* Walls face their room, so opposite normals are different faces. The
     * negated comparisons also reject NaN. */
    const PlaneGap planeGap =
        measurePlaneGap(absorbedPlane_world,
                        survivingPlane_world,
                        absorbedWall_survivingWorld_in.wallCentroid_world,
                        true);
    if (!(planeGap.normalAngle_deg <= configuration_in.coplanarAngle_deg))
    {
        return WallRelation::UNRELATED;
    }
    if (!(measureWallExtentGap(absorbedWall_survivingWorld_in,
                               survivingWall_in) <=
          configuration_in.wallEdgeOverlap_m))
    {
        return WallRelation::UNRELATED;
    }
    return planeGap.offset_m <= configuration_in.wallCoplanarOffset_m
               ? WallRelation::PAIRED
               : WallRelation::OFFSET_MISMATCH;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

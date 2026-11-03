/*!
 * @file            transformRoomEvidenceBetweenMaps.cc
 *
 * @brief           Implements transformRoomEvidenceBetweenMaps(), declared in
 * private_functions.h.
 */

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticMergeRoomEvidence transformRoomEvidenceBetweenMaps(
    const SemanticMergeRoomEvidence &room_absorbedWorld_in,
    const g2o::Sim3                 &mergeTransform_absorbedToSurviving_in)
{
    SemanticMergeRoomEvidence room_survivingWorld = room_absorbedWorld_in;
    room_survivingWorld.context.centroid =
        mergeTransform_absorbedToSurviving_in.map(
            room_absorbedWorld_in.context.centroid);
    if (room_absorbedWorld_in.floorPlane_world.has_value())
    {
        room_survivingWorld.floorPlane_world =
            transformPlaneBetweenMaps(*room_absorbedWorld_in.floorPlane_world,
                                      mergeTransform_absorbedToSurviving_in);
    }
    for (VerifyWallObservation &wall_world : room_survivingWorld.walls)
    {
        wall_world =
            transformWallBetweenMaps(wall_world,
                                     mergeTransform_absorbedToSurviving_in);
    }
    for (PassageContext &passage_world :
         room_survivingWorld.context.passageContexts)
    {
        passage_world =
            transformPassageBetweenMaps(passage_world,
                                        mergeTransform_absorbedToSurviving_in);
    }
    return room_survivingWorld;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

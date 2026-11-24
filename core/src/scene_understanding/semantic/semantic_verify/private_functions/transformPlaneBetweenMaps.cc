/*!
 * @file            transformPlaneBetweenMaps.cc
 *
 * @brief           Implements transformPlaneBetweenMaps(), declared in
 * private_functions.h.
 */

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

Eigen::Vector4d transformPlaneBetweenMaps(
    const Eigen::Vector4d &plane_absorbedWorld_in,
    const g2o::Sim3       &mergeTransform_absorbedToSurviving_in)
{
    const Eigen::Matrix3d rotation_absorbedToSurviving =
        mergeTransform_absorbedToSurviving_in.rotation()
            .toRotationMatrix()
            .cast<double>();
    const Eigen::Vector3d translation_absorbedToSurviving_m =
        mergeTransform_absorbedToSurviving_in.translation().cast<double>();

    /* A point on the plane maps as p' = s R p + t. With n' = R n, the plane
     * n^T p + d = 0 becomes n'^T p' + (s d - n'^T t) = 0. */
    const Eigen::Vector3d normal_survivingWorld =
        rotation_absorbedToSurviving * plane_absorbedWorld_in.head<3>();
    Eigen::Vector4d plane_survivingWorld;
    plane_survivingWorld << normal_survivingWorld,
        mergeTransform_absorbedToSurviving_in.scale() *
                plane_absorbedWorld_in(3) -
            normal_survivingWorld.dot(translation_absorbedToSurviving_m);
    return plane_survivingWorld;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

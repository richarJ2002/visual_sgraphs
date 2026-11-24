/*!
 * @file            measurePlaneGap.cc
 *
 * @brief           Implements measurePlaneGap(), declared in
 * private_functions.h.
 */

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

PlaneGap measurePlaneGap(const Eigen::Vector4d &firstPlane_world_in,
                         const Eigen::Vector4d &secondPlane_world_in,
                         const Eigen::Vector3d &referencePoint_world_in,
                         bool                   isOrientationSignificant_in)
{
    PlaneGap gap;
    gap.normalAngle_deg = std::numeric_limits<double>::infinity();
    gap.offset_m        = std::numeric_limits<double>::infinity();

    const double firstNormalNorm  = firstPlane_world_in.head<3>().norm();
    const double secondNormalNorm = secondPlane_world_in.head<3>().norm();
    if (!firstPlane_world_in.allFinite() || !secondPlane_world_in.allFinite() ||
        !referencePoint_world_in.allFinite() || firstNormalNorm < 1e-8 ||
        secondNormalNorm < 1e-8)
    {
        return gap;
    }

    const Eigen::Vector4d firstPlane_unit =
        firstPlane_world_in / firstNormalNorm;
    Eigen::Vector4d secondPlane_unit = secondPlane_world_in / secondNormalNorm;
    double          normalCosine =
        firstPlane_unit.head<3>().dot(secondPlane_unit.head<3>());
    if (!isOrientationSignificant_in && normalCosine < 0.0)
    {
        /* The same plane described with the opposite normal. */
        secondPlane_unit = -secondPlane_unit;
        normalCosine     = -normalCosine;
    }
    gap.normalAngle_deg =
        std::acos(std::clamp(normalCosine, -1.0, 1.0)) * DEGREES_PER_RADIAN;
    gap.offset_m =
        std::abs(secondPlane_unit.head<3>().dot(referencePoint_world_in) +
                 secondPlane_unit(3));
    return gap;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

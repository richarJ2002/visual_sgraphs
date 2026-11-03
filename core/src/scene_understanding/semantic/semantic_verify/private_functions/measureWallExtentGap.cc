/*!
 * @file            measureWallExtentGap.cc
 *
 * @brief           Implements measureWallExtentGap(), declared in
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

double measureWallExtentGap(const VerifyWallObservation &firstWall_world_in,
                            const VerifyWallObservation &secondWall_world_in)
{
    if (firstWall_world_in.supportSample_world.empty() ||
        secondWall_world_in.supportSample_world.empty())
    {
        return std::numeric_limits<double>::infinity();
    }

    /* In-plane axes from the second wall's normal, built the same way as the
     * plane bounds (a fixed reference axis avoids a degenerate cross
     * product). */
    const Eigen::Vector3d normal_world =
        secondWall_world_in.wallNormal_world.normalized();
    const Eigen::Vector3d referenceAxis_world =
        std::abs(normal_world.x()) <= 0.90 ? Eigen::Vector3d::UnitX()
                                           : Eigen::Vector3d::UnitY();
    const Eigen::Vector3d axisFirst_world =
        normal_world.cross(referenceAxis_world).normalized();
    const Eigen::Vector3d axisSecond_world =
        normal_world.cross(axisFirst_world).normalized();

    double firstMin_m[2]  = {std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity()};
    double firstMax_m[2]  = {-std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity()};
    double secondMin_m[2] = {std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity()};
    double secondMax_m[2] = {-std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity()};
    for (const Eigen::Vector3d &point_world :
         firstWall_world_in.supportSample_world)
    {
        const double inPlane_m[2] = {point_world.dot(axisFirst_world),
                                     point_world.dot(axisSecond_world)};
        for (int axisIndex = 0; axisIndex < 2; axisIndex++)
        {
            firstMin_m[axisIndex] =
                std::min(firstMin_m[axisIndex], inPlane_m[axisIndex]);
            firstMax_m[axisIndex] =
                std::max(firstMax_m[axisIndex], inPlane_m[axisIndex]);
        }
    }
    for (const Eigen::Vector3d &point_world :
         secondWall_world_in.supportSample_world)
    {
        const double inPlane_m[2] = {point_world.dot(axisFirst_world),
                                     point_world.dot(axisSecond_world)};
        for (int axisIndex = 0; axisIndex < 2; axisIndex++)
        {
            secondMin_m[axisIndex] =
                std::min(secondMin_m[axisIndex], inPlane_m[axisIndex]);
            secondMax_m[axisIndex] =
                std::max(secondMax_m[axisIndex], inPlane_m[axisIndex]);
        }
    }

    double largestGap_m = 0.0;
    for (int axisIndex = 0; axisIndex < 2; axisIndex++)
    {
        const double axisGap_m =
            std::max(firstMin_m[axisIndex] - secondMax_m[axisIndex],
                     secondMin_m[axisIndex] - firstMax_m[axisIndex]);
        largestGap_m = std::max(largestGap_m, axisGap_m);
    }
    return largestGap_m;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

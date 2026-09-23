/*!
 * @file            private_functions.h
 *
 * @brief           Declares module-internal helpers shared between the
 *                  SemanticSegmentation translation units.
 *
 * @note            These helpers were file-scope entities inside the
 *                  anonymous namespace of SemanticSegmentation.cc;
 *                  external linkage here is module-internal only. Names
 *                  are kept verbatim (WP-01C renaming is a separate step).
 */

#ifndef VS_GRAPHS_CORE_SEMANTICSEGMENTATION_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_SEMANTICSEGMENTATION_PRIVATE_FUNCTIONS_H

#include <cstddef>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief        Describes the strongest spatially connected part of a wall
 *               cloud.
 */
struct WallComponentSupport
{
    /*! @brief Source-cloud indices forming the largest component. */
    std::vector<int> pointIndices;
    /*! @brief Number of finite points considered by clustering. */
    std::size_t      finitePointCount = 0U;
    /*! @brief Fraction of finite points in the largest component. */
    double           componentRatio = 0.0;
};

WallComponentSupport findLargestWallComponent(
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_wallCloud_in,
    const double                                        clusterTolerance_m_in);

} // namespace core
} // namespace vs_graphs

#endif /* VS_GRAPHS_CORE_SEMANTICSEGMENTATION_PRIVATE_FUNCTIONS_H */
